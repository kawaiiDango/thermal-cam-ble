#ifndef H_ADC_STUFF_
#define H_ADC_STUFF_

#include <stdint.h>
#include <hal/adc_types.h>
#include <esp_adc/adc_cali.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define TAG_ADC "adc"

void adc_deinit();
float battery_voltage_read(void);

#ifdef __cplusplus
}
#endif

#endif


