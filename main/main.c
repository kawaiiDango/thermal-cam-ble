#include <driver/gpio.h>
#include "config.h"
#include <prefs.h>
#include "bleprph.h"
#include <nvs_flash.h>
#include <esp_err.h>
#include <esp_pm.h>
#include "led_indicator.h"
#include "adc_stuff.h"
#include <esp_sleep.h>
#include <nimble/nimble_port.h>
#include <Adafruit_MLX90640.h>

#define TAG "main"

uint64_t timeit_t1 = 0;
bool mlx_powered_on = false;
bool mlx_inited = false;
bool bt_inited = false;
float emissivity = 0.95; // resets at boot
RTC_DATA_ATTR int boot_count = 0;
indicator_state current_indicator_state = INDICATOR_NOT_NOTIFYING;

void timeit(const char *msg)
{
  uint64_t timeit_t2 = esp_timer_get_time();
  int64_t time_diff = timeit_t2 - timeit_t1;
  ESP_LOGI(TAG, "\n%s: %llu\n", msg, time_diff / 1000);
  timeit_t1 = timeit_t2;
}

void go_to_deep_sleep()
{
  gpio_hold_dis((gpio_num_t)THERMAL_CAM_POWER_PIN);
  gpio_set_level((gpio_num_t)THERMAL_CAM_POWER_PIN, 0);
  gpio_hold_en((gpio_num_t)THERMAL_CAM_POWER_PIN);

  bt_inited = false;
  nimble_port_stop();
  fflush(stdout);
  // set pair button pin as the wake up source
  // nvm, Not an RTC IO: GPIO9
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_deep_sleep_start();
}

float battery_voltage_read_and_set_level()
{
  float battery_voltage = battery_voltage_read();
  // init it again later otherwise it continues to show a constant max value for future reads
  // the next light sleep will deactivate the ADC
  adc_deinit();

  if (battery_voltage < 3.6)
  {
    ESP_LOGE(TAG, "Battery voltage is too low: %.2fV", battery_voltage);
    go_to_deep_sleep();
  }

  float battery_level = (battery_voltage - 3.6) / (4.2 - 3.6) * 100;
  set_battery_level(battery_level);
  return battery_voltage;
}

void mlx_power_on()
{
  gpio_hold_dis((gpio_num_t)THERMAL_CAM_POWER_PIN);
  gpio_set_level((gpio_num_t)THERMAL_CAM_POWER_PIN, 1);
  gpio_hold_en((gpio_num_t)THERMAL_CAM_POWER_PIN);
  mlx_powered_on = true;
}

void mlx_power_off()
{
  gpio_hold_dis((gpio_num_t)THERMAL_CAM_POWER_PIN);
  gpio_set_level((gpio_num_t)THERMAL_CAM_POWER_PIN, 0);
  gpio_hold_en((gpio_num_t)THERMAL_CAM_POWER_PIN);
  mlx_powered_on = false;
}

void init_mlx()
{
  for (int i = 0; i < 10 && !mlx_inited; i++)
  {
    mlx_inited = mlx_begin() == 0;
    current_indicator_state = INDICATOR_READ_FAILED;
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  if (!mlx_inited)
  {
    ESP_LOGE(TAG, "MLX90640 not found");
    go_to_deep_sleep();
  }
  else
  {
    // mlx.setMode(MLX90640_CHESS); // default
    // mlx.setResolution(MLX90640_ADC_18BIT); // default
  }
}

void setup_io()
{
  initFromPrefs();

  if (esp_reset_reason() == ESP_RST_BROWNOUT && prefs.lastResetReason == ESP_RST_BROWNOUT)
  {
    ESP_LOGE(TAG, "Repeated brownouts, going to sleep");
    go_to_deep_sleep();
  }

  esp_reset_reason_t resetReason = esp_reset_reason();
  if (resetReason != prefs.lastResetReason && resetReason != ESP_RST_DEEPSLEEP)
  {
    prefs.lastResetReason = resetReason;
    savePrefs();
  }

  // from datasheet
  // "Except for GPIO12 and GPIO13 whose default drive strength is 40 mA, the default drive strength for all the other pins is 20 mA."
  // for the MLX90640, "Supply Current IDD 15 20 25 mA"
  // digitalWrite(THERMAL_CAM_POWER_PIN, HIGH);
  // No longer using this since there is a voltage drop to 2.97V when powering from gpio
  // Changed my mind, I put a MOSFET in there now, getting constant 3.26V
  gpio_config_t io_conf = {
      .intr_type = 0,
      .mode = GPIO_MODE_OUTPUT,
      .pin_bit_mask = 1ULL << THERMAL_CAM_POWER_PIN,
      .pull_down_en = 0,
      .pull_up_en = 0,
  };
  gpio_config(&io_conf);
  gpio_set_drive_capability((gpio_num_t)THERMAL_CAM_POWER_PIN, GPIO_DRIVE_CAP_0);
  gpio_set_level((gpio_num_t)THERMAL_CAM_POWER_PIN, 0);
  gpio_hold_en((gpio_num_t)THERMAL_CAM_POWER_PIN);
}

void indicator_task(void *pvParameter)
{
  gpio_config_t io_conf = (gpio_config_t){
      .intr_type = 0,
      .mode = GPIO_MODE_OUTPUT,
      .pin_bit_mask = 1ULL << USER_LED_PIN,
      .pull_down_en = 0,
      .pull_up_en = 0,
  };

  gpio_config(&io_conf);

  while (1)
  {
    switch (current_indicator_state)
    {
    case INDICATOR_NOT_NOTIFYING:
      gpio_set_level(USER_LED_PIN, 1);
      vTaskDelay(3 / portTICK_PERIOD_MS);
      gpio_set_level(USER_LED_PIN, 0);
      break;
    case INDICATOR_BONDING_REQ:
      gpio_set_level(USER_LED_PIN, 1);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      gpio_set_level(USER_LED_PIN, 0);
      break;
    case INDICATOR_NOTIFYING:
    case INDICATOR_TRANSFER_SUCCESS:
      gpio_set_level(USER_LED_PIN, 0);
      break;
    case INDICATOR_READ_FAILED:
    case INDICATOR_TRANSFER_FAILED:
      gpio_set_level(USER_LED_PIN, 1);
      break;
    default:
      break;
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void cam_read_loop(void *pvParameters)
{
  uint8_t temperatures_frame_arr_idx = 0;
  uint64_t last_status_notify = 0;
  uint64_t last_temps_notify = 0;
  // signed cuz this sometimes becomes negative for some reason
  int64_t time_diff = 0;
  uint64_t frame_start_time;
  struct status_t status;
  mlx90640_refreshrate_t last_hardware_refresh_rate = MLX90640_2_HZ;

  while (1)
  {
    frame_start_time = esp_timer_get_time();

    if (!at_least_one_subscribed())
    {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      time_diff = frame_start_time - last_temps_notify;

      if (time_diff / 1000 > CAM_IDLE_TIMEOUT && mlx_powered_on)
      {
        ESP_LOGW(TAG, "No one is subscribed to temps frame, powering off the camera");
        mlx_power_off();
      }

      if (time_diff / 1000 > BLE_IDLE_TIMEOUT)
      {
        ESP_LOGW(TAG, "No one is subscribed to temps frame, deep sleeping");
        go_to_deep_sleep();
      }

      continue;
    }

    if (!mlx_powered_on)
    {
      mlx_power_on();
    }

    if (!mlx_inited)
    {
      init_mlx();
      if (!mlx_inited)
      {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        continue;
      }

      mlx_setRefreshRate((mlx90640_refreshrate_t)prefs.refreshRate);
      last_hardware_refresh_rate = prefs.refreshRate;
    }

    if (last_hardware_refresh_rate != prefs.refreshRate)
    {
      mlx_setRefreshRate((mlx90640_refreshrate_t)prefs.refreshRate);
      last_hardware_refresh_rate = prefs.refreshRate = mlx_getRefreshRate();

      savePrefs();

      ESP_LOGI(TAG, "Set refresh rate to %u", last_hardware_refresh_rate);
    }

    // timeit("ignore");
    int res = mlx_getFrame(temps_frames[temperatures_frame_arr_idx], emissivity);

    if (res != 0)
    {
      ESP_LOGE(TAG, "Failed to get frame: %d", res);
      current_indicator_state = INDICATOR_READ_FAILED;
      continue;
    }
    // timeit("take_thermal");

    if (xQueueSend(temps_frame_queue, &temperatures_frame_arr_idx, 0) != pdTRUE)
    {
      ESP_LOGE(TAG, "Failed to send temps frame to queue");
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    last_temps_notify = esp_timer_get_time();

    temperatures_frame_arr_idx = (temperatures_frame_arr_idx + 1) % MAX_Q_ITEMS;

    // update status every STATUS_UPDATE_ITVL ms

    time_diff = frame_start_time - last_status_notify;

    if (time_diff / 1000 > STATUS_UPDATE_ITVL)
    {
      float battery_voltage = battery_voltage_read_and_set_level();
      float free_heap_k = (float)heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024;

      status = (struct status_t){
          .t_a = mlx_getTa(),
          .battery_voltage = battery_voltage,
          .free_heap_k = free_heap_k};

      if (xQueueSend(status_queue, &status, 0) != pdTRUE)
      {
        ESP_LOGE(TAG, "Failed to send status to queue");
      }
      last_status_notify = esp_timer_get_time();
    }

    // delay for (refresh rate) - time taken to get frame
    int64_t timeLeft = esp_timer_get_time() - frame_start_time;
    timeLeft = (long)(1000.0f / last_hardware_refresh_rate) - timeLeft / 1000;

    if (timeLeft > 0)
    {
      ESP_LOGI(TAG, "Delaying for %lld ms", timeLeft / 1000);
      vTaskDelay(timeLeft / 1000 / portTICK_PERIOD_MS);
    }
    else
      vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void app_main(void)
{
  // init queues
  temps_frame_queue = xQueueCreate(MAX_Q_ITEMS, sizeof(uint8_t));
  status_queue = xQueueCreate(1, sizeof(struct status_t));

  boot_count++;

  esp_log_level_set(TAG, ESP_LOG_INFO);
  esp_log_level_set(TAG, ESP_LOG_INFO);

#ifdef DEBUG_MODE
  vTaskDelay(1000 / portTICK_PERIOD_MS); // wait for serial monitor to reconnect before printing
  ESP_LOGI(TAG, "Boot count: %d", boot_count);
#endif

  /* Initialize NVS — it is used to store PHY calibration data */
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

#if CONFIG_PM_ENABLE
  // Configure dynamic frequency scaling:
  // maximum and minimum frequencies are set in sdkconfig,
  // automatic light sleep is enabled if tickless idle support is enabled.
  esp_pm_config_t pm_config = {
      .max_freq_mhz = MAX_FREQ_MHZ,
      .min_freq_mhz = MIN_FREQ_MHZ,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
#ifndef DEBUG_MODE
      .light_sleep_enable = true
#endif
#endif
  };
  ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
#endif // CONFIG_PM_ENABLE

  // set battery % and sleep forever if battery loo low
  battery_voltage_read_and_set_level();

  setup_io();

  // Initialize BLE
  bt_host_init();
  bt_io_init();
  bt_inited = true;

  xTaskCreate(indicator_task, "indicator_task", 2048, NULL, 1, NULL);
  xTaskCreate(cam_read_loop, "cam_read_loop", 4096 + 2048, NULL, 5, NULL);
}
