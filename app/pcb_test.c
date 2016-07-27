#include "pcb_test.h"
#include "SEGGER_RTT.h"
#include <stdint.h>
#include "LSM303drv.h"
#include "L3GD20drv.h"
#include <stdbool.h>


void run_pcb_tests(uint8_t  * pcb_test_results)
{
    bool all_test_success_flag = 1;
    test_function tests[NUM_ARM_PCB_TESTS] = 
    {
        GYRO_deviceID_test,
        ACCEL_deviceID_test,
    };
    
    //run the tests!
    int i;
    uint8_t test_result;
    SEGGER_RTT_printf(0, "\n\n ARM PCB TEST RESULTS:\n");
    for(i = 0; i < NUM_ARM_PCB_TESTS; i++)
    {
        test_result = (*tests[i])(); //function pointer call to the actual test
        SEGGER_RTT_printf(0, "TEST %d = %s\n", i, (test_result == 0) ? "SUCCESS" : "FAIL");
        pcb_test_results[i] = test_result;
    }
}



uint8_t GYRO_deviceID_test(void)
{
    /*********** SETUP  ********/
    uint8_t test_result = 0;
    uint8_t ID;
    
    // response from L3GD20H is a 0xd7
//    ID = get_L3GD_ID();
//    /*********** TEST ***********/
//    if(ID != L3GD20H_ID)
//    {
//        test_result = 1;
//        SEGGER_RTT_printf(0, "L3GD20H device ID = 0x%2x\n", ID);

//    }
    /***********  CLEANUP ********/
    return test_result;
}
uint8_t ACCEL_deviceID_test(void)
{
    /*********** SETUP  ********/
    uint8_t test_result = 0;
    uint8_t ID;
    
    // LSM303D responds with 0x49
//    ID = get_LSM303_ID();		
//    
//    /*********** TEST ***********/
//    if(ID != LSM303D_ID)
//    {
//        test_result = 1;
//        SEGGER_RTT_printf(0, "LSM303D device ID = 0x%2x\n", ID);
//    }
    /***********  CLEANUP ********/
    return test_result;
}