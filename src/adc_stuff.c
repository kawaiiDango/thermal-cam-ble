/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include <esp_sleep.h>
#include "adc_stuff.h"
#include "config.h"

adc_oneshot_unit_handle_t adc1_handle = NULL;
adc_cali_handle_t adc1_cali_chan0_handle = NULL;
bool do_calibration = false;
bool adc_inited = false;

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
void adc_init(adc_channel_t channel)
{
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    //-------------ADC1 Init---------------//
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, channel, &config));

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG_ADC, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = ADC_UNIT_1,
            .chan = channel,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_chan0_handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG_ADC, "Calibration Success");
    }
    else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated)
    {
        ESP_LOGW(TAG_ADC, "eFuse not burnt, skip software calibration");
    }
    else
    {
        ESP_LOGE(TAG_ADC, "Invalid arg or no memory");
    }
    adc_inited = true;
    do_calibration = calibrated;
}

void adc_deinit()
{
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG_ADC, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(adc1_cali_chan0_handle));
#endif
    adc_inited = false;
}

float battery_voltage_read(void)
{
    int samples = 8;
    int adc_raw;
    int milli_volts;
    float batt_voltage = 0;
    adc_unit_t unit = ADC_UNIT_1;
    adc_channel_t channel;

    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(BATTERY_VOLTAGE_PIN, &unit, &channel));

    if (!adc_inited)
    {
        adc_init(channel);
    }

    for (int i = 0; i < samples; i++)
    {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, channel, &adc_raw));
        if (do_calibration)
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw, &milli_volts));

        batt_voltage += (float)milli_volts * 2 / 1000;
    }

    batt_voltage /= samples;

    return batt_voltage;
}
