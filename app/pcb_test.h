#ifndef PCB_TEST_H
#define	PCB_TEST_H

#include <stdint.h>

#define NUM_ARM_PCB_TESTS 2
#define L3GD20H_ID 0xD7
#define LSM303D_ID 0x49

/******* function pointer typedef ********/
typedef uint8_t (*test_function)(void);

void run_pcb_tests(uint8_t * pcb_test_results);

uint8_t GYRO_deviceID_test(void);
uint8_t ACCEL_deviceID_test(void);


#endif //PCB_TEST_H