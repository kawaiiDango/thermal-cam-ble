#include "led_indicator.h"
#include "config.h"
#include <Arduino.h>

enum indicator_state current_indicator_state;

void indicator_task(void *pvParameter)
{
    pinMode(USER_LED_PIN, OUTPUT);

    while (1)
    {
        switch (current_indicator_state)
        {
        case INDICATOR_MLX_NOT_INITED:
            digitalWrite(USER_LED_PIN, HIGH);
            delay(500);
            digitalWrite(USER_LED_PIN, LOW);
            break;
        case INDICATOR_BLE_ADVERTISING:
            digitalWrite(USER_LED_PIN, HIGH);
            delay(50);
            digitalWrite(USER_LED_PIN, LOW);
            break;
        case INDICATOR_BLE_BONDING_REQ:
            digitalWrite(USER_LED_PIN, HIGH);
            delay(1000);
            digitalWrite(USER_LED_PIN, LOW);
            break;
        case INDICATOR_CONNECTED:
        case INDICATOR_TRANSFER_SUCCESS:
            digitalWrite(USER_LED_PIN, LOW);
            break;
        case INDICATOR_READ_FAILED:
        case INDICATOR_TRANSFER_FALED:
            digitalWrite(USER_LED_PIN, HIGH);
            break;
        default:
            break;
        }
        delay(1000);
    }
}