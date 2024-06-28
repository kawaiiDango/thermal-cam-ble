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

#define TAG_MAIN "main"

Adafruit_MLX90640 mlx;
unsigned long timeit_t1 = 0;
bool mlx_inited = false;

void timeit(const char *msg)
{
  unsigned long timeit_t2 = millis();
  ESP_LOGI(TAG_MAIN, "\n%s: %ld\n", msg, timeit_t2 - timeit_t1);
  timeit_t1 = timeit_t2;
}

float pollBatteryVoltage()
{
  float v = 0;
  int rounds = 10;

  for (int i = 0; i < rounds; i++)
  {
    v += (float)analogReadMilliVolts(BATTERY_VOLTAGE_PIN) * 2 / 1000;
  }

  v /= rounds;

  if (v < 3.6)
  {
    ESP_LOGE(TAG_MAIN, "Battery voltage is too low: %.2fV", v);
    Serial.flush();
    esp_deep_sleep_start();
  }
  // lithium ion battery voltage range is 3.6V to 4.2V
  uint8_t battery_level = (v - 3.6) / (4.2 - 3.6) * 100;
  set_battery_level(battery_level);

  return v;
}

void init_mlx()
{
  digitalWrite(THERMAL_CAM_POWER_PIN, HIGH);

  Wire.begin(THERMAL_CAM_SDA_PIN, THERMAL_CAM_SCL_PIN, 400000);

  for (int i = 0; i < 10 && !(mlx_inited = mlx.begin()); i++)
  {
    current_indicator_state = INDICATOR_MLX_NOT_INITED;
    delay(1000);
  }

  if (!mlx_inited)
  {
    ESP_LOGE(TAG_MAIN, "MLX90640 not found");
    Serial.flush();
    esp_deep_sleep_start();
  }
  else
  {
    // mlx.setMode(MLX90640_CHESS); // default
    // mlx.setResolution(MLX90640_ADC_18BIT); // default
  }
}

void setup_io()
{
  Serial.begin(115200);

  initFromPrefs();

  if (esp_reset_reason() == ESP_RST_BROWNOUT && prefs.lastResetReason == ESP_RST_BROWNOUT)
  {
    ESP_LOGE(TAG_MAIN, "Repeated brownouts, going to sleep");
    Serial.flush();
    esp_deep_sleep_start();
  }

  esp_reset_reason_t resetReason = esp_reset_reason();
  if (resetReason != prefs.lastResetReason && resetReason != ESP_RST_DEEPSLEEP)
  {
    prefs.lastResetReason = resetReason;
    savePrefs();
  }

  pinMode(BATTERY_VOLTAGE_PIN, INPUT);

  // from datasheet
  // "Except for GPIO12 and GPIO13 whose default drive strength is 40 mA, the default drive strength for all the other pins is 20 mA."
  // for the MLX90640, "Supply Current IDD 15 20 25 mA"
  // digitalWrite(THERMAL_CAM_POWER_PIN, HIGH);
  // No longer using this since there is a voltage drop to 2.97V when powering from gpio
  // Changed my mind, I put a MOSFET in there now, getting constant 3.26V
  pinMode(THERMAL_CAM_POWER_PIN, OUTPUT);
  gpio_set_drive_capability((gpio_num_t)THERMAL_CAM_POWER_PIN, GPIO_DRIVE_CAP_0);
}

void cam_read_loop(void *pvParameters)
{
  uint8_t temperatures_frame_arr_idx = 0;
  uint64_t last_status_notify = 0;
  uint64_t last_temps_notify = 0;
  uint64_t now;
  struct status_t status;
  float last_hardware_refresh_rate = -1.0f;

  while (1)
  {
    now = millis();

    if (!at_least_one_subscribed())
    {
      delay (1000);

      if (now - last_temps_notify > CAM_IDLE_TIMEOUT && mlx_inited)
      {
        ESP_LOGI(TAG_MAIN, "No one is subscribed to temps frame, powering off the camera");
        digitalWrite(THERMAL_CAM_POWER_PIN, LOW);
        mlx_inited = false;
      }

      if (now - last_temps_notify > BLE_IDLE_TIMEOUT)
      {
        ESP_LOGI(TAG_MAIN, "No one is subscribed to temps frame, deep sleeping");
        fflush(stdout);
        esp_deep_sleep_start();
      }

      continue;
    }

    if (!mlx_inited)
    {
      init_mlx();
      if (!mlx_inited)
      {
        vTaskDelay(500 / portTICK_PERIOD_MS);
        continue;
      }

      last_hardware_refresh_rate = refreshRateToFps(mlx.getRefreshRate());
      prefs.refreshRate = prefs.refreshRate;
    }

    if (last_hardware_refresh_rate != prefs.refreshRate)
    {
      mlx.setRefreshRate(fpsToRefreshRate(prefs.refreshRate));
      last_hardware_refresh_rate = prefs.refreshRate = refreshRateToFps(mlx.getRefreshRate());

      savePrefs();

      ESP_LOGI(TAG_MAIN, "Set refresh rate to %.2f", last_hardware_refresh_rate);
    }

    // timeit("ignore");
    int res = mlx.getFrame((float *)temps_frames[temperatures_frame_arr_idx]);

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

    if ((millis() - last_status_notify) > STATUS_UPDATE_ITVL)
    {
      last_status_notify = millis();
      status = {
          .t_a = mlx.getTa(false),
          .battery_voltage = (float)pollBatteryVoltage(),
          .free_heap_k = (float)ESP.getFreeHeap() / 1024};

      if (xQueueSend(status_queue, &status, 0) != pdTRUE)
      {
        ESP_LOGE(TAG_MAIN, "Failed to send status to queue");
      }
    }

    // delay for (refresh rate) - time taken to get frame
    long timeLeft = (long)(1000.0f / last_hardware_refresh_rate) - (millis() - now);

    if (timeLeft > 0)
    {
      ESP_LOGI(TAG_MAIN, "Delaying for %ld ms", timeLeft);
      delay(timeLeft);
    }
    else
      yield();
  }
}

// this function sends mock data to the temps frame queue every 500ms
static void temps_frame_mock_produce(void *arg)
{
  uint8_t temperatures_frame_arr_idx = 0;

  while (1)
  {
    if (!at_least_one_subscribed())
    {
      vTaskDelay(500 / portTICK_PERIOD_MS);
      continue;
    }

    for (int i = 0; i < NUM_TEMPS_PIXELS; i++)
    {
      float random_float = 5.0 + ((float)rand() / (float)RAND_MAX) * (100.0 - 5.0);
      temps_frames[temperatures_frame_arr_idx][i] = random_float;
    }

    if (xQueueSend(temps_frame_queue, &temperatures_frame_arr_idx, 0) != pdTRUE)
    {
      ESP_LOGE(TAG_MAIN, "Failed to send temps frame to queue");
    }

    vTaskDelay(500 / portTICK_PERIOD_MS);

    temperatures_frame_arr_idx = (temperatures_frame_arr_idx + 1) % MAX_Q_ITEMS;
  }
}

extern "C" void app_main(void)
{
  esp_log_level_set(TAG_MAIN, ESP_LOG_INFO);
  esp_log_level_set(TAG_PREFS, ESP_LOG_INFO);

  // delay(1000);

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
      .max_freq_mhz = 80,
      .min_freq_mhz = 10,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
  // couldnt get ble to work with this
  // .light_sleep_enable = true
#endif
  };
  ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
#endif // CONFIG_PM_ENABLE

  setup_io();

  // init queues
  temps_frame_queue = xQueueCreate(MAX_Q_ITEMS, sizeof(uint8_t));
  status_queue = xQueueCreate(1, sizeof(struct status_t));

  // Initialize BLE
  bt_host_init();
  bt_io_init();

  pollBatteryVoltage(); // set battery % and sleep forever if battery loo low

  xTaskCreate(indicator_task, "indicator_task", 2048, NULL, 1, NULL);
  // start reading the camera
  xTaskCreate(cam_read_loop, "cam_read_loop", 2048 + 1024, NULL, 5, NULL);
  // xTaskCreate(temps_frame_mock_produce, "temps_frame_mock_produce", 4096, NULL, 5, NULL);
}
