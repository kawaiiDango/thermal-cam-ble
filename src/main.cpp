#include <Arduino.h>
#include <Adafruit_MLX90640.h>
#include <driver/gpio.h>
#include "config.h"
#include "my_utils.hpp"
#include <prefs.h>
#include "bleprph.h"
#include <nvs_flash.h>
#include <esp_err.h>
#include <esp_pm.h>
#include "led_indicator.h"
#include "adc_stuff.h"
#include <esp_sleep.h>
#include <nimble/nimble_port.h>

#define TAG_MAIN "main"

Adafruit_MLX90640 mlx;
uint64_t timeit_t1 = 0;
bool mlx_inited = false;
bool bt_inited = false;
RTC_DATA_ATTR int boot_count = 0;

void timeit(const char *msg)
{
  uint64_t timeit_t2 = millis();
  int64_t time_diff = timeit_t2 - timeit_t1;
  ESP_LOGI(TAG_MAIN, "\n%s: %llu\n", msg, time_diff);
  timeit_t1 = timeit_t2;
}

void go_to_deep_sleep()
{
  gpio_hold_dis((gpio_num_t)THERMAL_CAM_POWER_PIN);
  digitalWrite(THERMAL_CAM_POWER_PIN, LOW);
  gpio_hold_en((gpio_num_t)THERMAL_CAM_POWER_PIN);

  bt_inited = false;
  nimble_port_stop();
  btStop();
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
    ESP_LOGE(TAG_MAIN, "Battery voltage is too low: %.2fV", battery_voltage);
    go_to_deep_sleep();
  }

  float battery_level = (battery_voltage - 3.6) / (4.2 - 3.6) * 100;
  set_battery_level(battery_level);
  return battery_voltage;
}

void init_mlx()
{
  gpio_hold_dis((gpio_num_t)THERMAL_CAM_POWER_PIN);
  digitalWrite(THERMAL_CAM_POWER_PIN, HIGH);
  gpio_hold_en((gpio_num_t)THERMAL_CAM_POWER_PIN);

  Wire.begin(THERMAL_CAM_SDA_PIN, THERMAL_CAM_SCL_PIN, 400000);

  for (int i = 0; i < 10 && !(mlx_inited = mlx.begin()); i++)
  {
    current_indicator_state = INDICATOR_MLX_NOT_INITED;
    delay(1000);
  }

  if (!mlx_inited)
  {
    ESP_LOGE(TAG_MAIN, "MLX90640 not found");
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
    ESP_LOGE(TAG_MAIN, "Repeated brownouts, going to sleep");
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
  pinMode(THERMAL_CAM_POWER_PIN, OUTPUT);
  gpio_set_drive_capability((gpio_num_t)THERMAL_CAM_POWER_PIN, GPIO_DRIVE_CAP_0);
  digitalWrite(THERMAL_CAM_POWER_PIN, LOW);
  gpio_hold_en((gpio_num_t)THERMAL_CAM_POWER_PIN);
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
  float last_hardware_refresh_rate = -1.0f;

  while (1)
  {
    frame_start_time = millis();

    if (!at_least_one_subscribed())
    {
      delay(1000);
      time_diff = frame_start_time - last_temps_notify;

      if (time_diff > CAM_IDLE_TIMEOUT && mlx_inited)
      {
        ESP_LOGI(TAG_MAIN, "No one is subscribed to temps frame, powering off the camera");
        gpio_hold_dis((gpio_num_t)THERMAL_CAM_POWER_PIN);
        digitalWrite(THERMAL_CAM_POWER_PIN, LOW);
        gpio_hold_en((gpio_num_t)THERMAL_CAM_POWER_PIN);
        mlx_inited = false;
      }

      if (time_diff > BLE_IDLE_TIMEOUT)
      {
        ESP_LOGI(TAG_MAIN, "No one is subscribed to temps frame, deep sleeping");
        go_to_deep_sleep();
      }

      continue;
    }

    if (!mlx_inited)
    {
      init_mlx();
      if (!mlx_inited)
      {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        continue;
      }

      mlx.setRefreshRate(fpsToRefreshRate(prefs.refreshRate));
      last_hardware_refresh_rate = prefs.refreshRate;
    }

    if (last_hardware_refresh_rate != prefs.refreshRate)
    {
      mlx.setRefreshRate(fpsToRefreshRate(prefs.refreshRate));
      last_hardware_refresh_rate = prefs.refreshRate = refreshRateToFps(mlx.getRefreshRate());

      savePrefs();

      ESP_LOGI(TAG_MAIN, "Set refresh rate to %.2f", last_hardware_refresh_rate);
    }

    // timeit("ignore");
    int res = mlx.getFrame(temps_frames[temperatures_frame_arr_idx]);

    if (res != 0)
    {
      ESP_LOGE(TAG_MAIN, "Failed to get frame: %d", res);
      current_indicator_state = INDICATOR_READ_FAILED;
      continue;
    }
    // timeit("take_thermal");

    if (xQueueSend(temps_frame_queue, &temperatures_frame_arr_idx, 0) != pdTRUE)
    {
      ESP_LOGE(TAG_MAIN, "Failed to send temps frame to queue");
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    last_temps_notify = millis();

    temperatures_frame_arr_idx = (temperatures_frame_arr_idx + 1) % MAX_Q_ITEMS;

    // update status every STATUS_UPDATE_ITVL ms

    time_diff = frame_start_time - last_status_notify;

    if (time_diff > STATUS_UPDATE_ITVL)
    {
      float battery_voltage = battery_voltage_read_and_set_level();
      float free_heap_k = (float)ESP.getFreeHeap() / 1024;

      status = {
          .t_a = mlx.getTa(false),
          .battery_voltage = battery_voltage,
          .free_heap_k = free_heap_k};

      if (xQueueSend(status_queue, &status, 0) != pdTRUE)
      {
        ESP_LOGE(TAG_MAIN, "Failed to send status to queue");
      }
      last_status_notify = millis();
    }

    // delay for (refresh rate) - time taken to get frame
    long timeLeft = millis() - frame_start_time;
    timeLeft = (long)(1000.0f / last_hardware_refresh_rate) - timeLeft;

    if (timeLeft > 0)
    {
      ESP_LOGI(TAG_MAIN, "Delaying for %ld ms", timeLeft);
      delay(timeLeft);
    }
    else
      yield();
  }
}

extern "C" void app_main(void)
{
  // init queues
  temps_frame_queue = xQueueCreate(MAX_Q_ITEMS, sizeof(uint8_t));
  status_queue = xQueueCreate(1, sizeof(struct status_t));

  boot_count++;

  esp_log_level_set(TAG_MAIN, ESP_LOG_INFO);
  esp_log_level_set(TAG_PREFS, ESP_LOG_INFO);

#ifdef DEBUG_MODE
  delay(1000); // wait for serial monitor to reconnect before printing
  ESP_LOGI(TAG_MAIN, "Boot count: %d", boot_count);
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
  xTaskCreate(cam_read_loop, "cam_read_loop", 2048 + 1024, NULL, 5, NULL);
}
