#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdint.h>

/*
 * This structure contains data nad status information about all of the differen parts of calibration and testing.
 * Statuses obey the following encoding:
 *  not started or not complete = -1
 *  complete and pass = 0
 *  complete and fail = 1
 
 * This strcuture will be stored to falsh and its data fields will be sent out over BLE
 */
typedef struct{
    uint8_t force_status;
    uint16_t force_data[5];
    uint16_t force_weights[5];
    uint8_t optical_status;
    float optical_data;
    float optical_length;
    uint8_t hall_status;
    uint8_t vib_status;
}cal_data_t;
extern cal_data_t cal_data;

uint8_t hall_effect_calibration(void);

#endif //CALIBRATION_H


