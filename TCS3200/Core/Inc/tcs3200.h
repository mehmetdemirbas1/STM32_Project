/*
 * tcs3200.h
 *
 *  Created on: Dec 23, 2025
 *      Author: Mehmet Demirbaş
 */

#ifndef INC_TCS3200_H_
#define INC_TCS3200_H_

#include "main.h"

typedef enum {
    TCS_FILTER_RED = 0,
    TCS_FILTER_GREEN,
    TCS_FILTER_BLUE,
    TCS_FILTER_CLEAR
} TCS3200_Filter_t;

typedef enum {
    TCS_SCALE_2_PERCENT,
    TCS_SCALE_20_PERCENT,
    TCS_SCALE_100_PERCENT,
    TCS_POWER_DOWN
} TCS3200_Scale_t;

typedef struct {
    void (*SetPin_S0)(uint8_t state);
    void (*SetPin_S1)(uint8_t state);
    void (*SetPin_S2)(uint8_t state);
    void (*SetPin_S3)(uint8_t state);
    void (*SetPin_OE)(uint8_t state);
    void (*Delay_ms)(uint32_t ms);
    uint32_t (*Get_Frequency)(void);
} TCS3200_Hardware_t;


typedef struct {
    TCS3200_Hardware_t hw;

    uint32_t min_red, max_red;
    uint32_t min_green, max_green;
    uint32_t min_blue, max_blue;

    uint8_t R_Value;
    uint8_t G_Value;
    uint8_t B_Value;

} TCS3200_Handle_t;

// Fonksiyon Prototipleri
void TCS3200_Init(TCS3200_Handle_t *sensor, TCS3200_Hardware_t hardware_interface);
void TCS3200_SetScale(TCS3200_Handle_t *sensor, TCS3200_Scale_t scale);
void TCS3200_SetCalibration(TCS3200_Handle_t *sensor,
                            uint32_t minR, uint32_t maxR,
                            uint32_t minG, uint32_t maxG,
                            uint32_t minB, uint32_t maxB);
void TCS3200_Process(TCS3200_Handle_t *sensor);


#endif /* INC_TCS3200_H_ */
