/*
 * tcs3200.c
 *
 *  Created on: Dec 23, 2025
 *      Author: Mehmet Demirbaş
 */

#include "tcs3200.h"

static long map(long x, long in_min, long in_max, long out_min, long out_max) {
    if (x < in_min) x = in_min;
    if (x > in_max) x = in_max;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void TCS3200_Init(TCS3200_Handle_t *sensor, TCS3200_Hardware_t hardware_interface) {
    sensor->hw = hardware_interface;

    sensor->min_red = 0; sensor->max_red = 1000;
    sensor->min_green = 0; sensor->max_green = 1000;
    sensor->min_blue = 0; sensor->max_blue = 1000;

    if (sensor->hw.SetPin_OE) {
        sensor->hw.SetPin_OE(0);
    }
}


void TCS3200_SetScale(TCS3200_Handle_t *sensor, TCS3200_Scale_t scale) {
    switch (scale) {
        case TCS_SCALE_2_PERCENT:
            sensor->hw.SetPin_S0(0); sensor->hw.SetPin_S1(1); break;
        case TCS_SCALE_20_PERCENT:
            sensor->hw.SetPin_S0(1); sensor->hw.SetPin_S1(0); break;
        case TCS_SCALE_100_PERCENT:
            sensor->hw.SetPin_S0(1); sensor->hw.SetPin_S1(1); break;
        case TCS_POWER_DOWN:
        default:
            sensor->hw.SetPin_S0(0); sensor->hw.SetPin_S1(0); break;
    }
}

void TCS3200_SetCalibration(TCS3200_Handle_t *sensor,
                            uint32_t minR, uint32_t maxR,
                            uint32_t minG, uint32_t maxG,
                            uint32_t minB, uint32_t maxB) {
    sensor->min_red = minR; sensor->max_red = maxR;
    sensor->min_green = minG; sensor->max_green = maxG;
    sensor->min_blue = minB; sensor->max_blue = maxB;
}

void TCS3200_Process(TCS3200_Handle_t *sensor) {
    uint32_t raw_freq;

    sensor->hw.SetPin_S2(0);
    sensor->hw.SetPin_S3(0);
    sensor->hw.Delay_ms(10);
    raw_freq = sensor->hw.Get_Frequency();
    sensor->R_Value = (uint8_t)map(raw_freq, sensor->min_red, sensor->max_red, 0, 255);


    sensor->hw.SetPin_S2(1);
    sensor->hw.SetPin_S3(1);
    sensor->hw.Delay_ms(10);
    raw_freq = sensor->hw.Get_Frequency();
    sensor->G_Value = (uint8_t)map(raw_freq, sensor->min_green, sensor->max_green, 0, 255);


    sensor->hw.SetPin_S2(0);
    sensor->hw.SetPin_S3(1);
    sensor->hw.Delay_ms(10);
    raw_freq = sensor->hw.Get_Frequency();
    sensor->B_Value = (uint8_t)map(raw_freq, sensor->min_blue, sensor->max_blue, 0, 255);
}

