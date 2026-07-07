#include "bsp_adc.h"

#define BSP_ADC1_HANDLE &hadc1
// #define BSP_ADC2_HANDLE &hadc2




void bsp_adc_calibration_start(void)
{
    HAL_ADCEx_Calibration_Start(BSP_ADC1_HANDLE);
}

