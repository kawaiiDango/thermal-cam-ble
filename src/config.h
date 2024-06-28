#ifndef H_CONFIG_
#define H_CONFIG_

#define BATTERY_VOLTAGE_PIN 0
#define THERMAL_CAM_POWER_PIN 7
#define THERMAL_CAM_SDA_PIN 19
#define THERMAL_CAM_SCL_PIN 20
#define PAIR_BUTTON_PIN 9
#define USER_LED_PIN 15

#define DEVICE_NAME "thermalCam" // used in bt_name and device info
#define PAIR_BUTTON_TIMEOUT 20000
#define CAM_IDLE_TIMEOUT 20000
#define BLE_IDLE_TIMEOUT 300000
#define STATUS_UPDATE_ITVL 10000 /* 10 seconds */
#define BLE_GAP_ADV_ITVL 800     // 1s
#define LL_PACKET_TIME 2120
#define LL_PACKET_LENGTH 251
#define PREFERRED_MTU_VALUE 512
#define NUM_TEMPS_PIXELS 32 * 24
#define NUM_BYTES_PER_PIXEL 2
#define NUM_STATUS_ITEMS 3
#define NUM_NOTIFY_PACKETS 4
#define NUM_NOTIFY_BYTES_TOTAL (NUM_TEMPS_PIXELS + NUM_NOTIFY_PACKETS) * NUM_BYTES_PER_PIXEL // +NUM_TEMPS_PACKETS for the part number
#define NUM_NOTIFY_BYTES_PER_PACKET NUM_NOTIFY_BYTES_TOTAL / NUM_NOTIFY_PACKETS
#define NUM_NOTIFY_BYTES_LAST_PACKET NUM_NOTIFY_BYTES_PER_PACKET + (NUM_STATUS_ITEMS * NUM_BYTES_PER_PIXEL)
#define MAX_Q_ITEMS 4

#endif