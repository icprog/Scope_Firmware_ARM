/*******************************************************************************
  Application Header File for Scope nRF51 code
  
  Company:
    AvaTech
  
  File Name:
    app.h
		
  Summary:
    This header file provides prototypes and definitions for the Scope nRF51
		firmware.
		
  Description:
    This header file provides function prototypes and data type definitions for
    the Scope nRF51 firmware.  Some of these are required by the 
    system (such as the "APP_Initialize" and "APP_Tasks" prototypes) and some of 
    them are only used internally by the application (such as the "APP_STATES" 
    definition).  Both are defined here for convenience.
     
  Processor:       
    nRF51822
		
  Toolchaing:
		MDK-Lite version 5.20
 
  Author(s): 
    Richard Kirby, Joe Trovato
 
  Created on:
    June 1, 2016
  
  Revision History:
    Development version      June 1, 2016
 *******************************************************************************/

#ifndef _APP_H
#define _APP_H

#include <stdint.h>
#include <stdbool.h>
#include <LSM303drv.h>
#include <L3GD20drv.h>

#define CALIBRATION 0


#define FW_VERSION "0.04.01"



// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
	/* Application's state machine's initial state. */
	APP_STATE_INIT = 0,
    APP_STATE_POLLING = 1,
    APP_STATE_FORCE_CAL_INIT,
    APP_STATE_FORCE_CAL_WEIGHT,
    APP_STATE_OPTICAL_CAL_LENGTH,
    APP_STATE_FORCE_CAL_DATA,
    APP_STATE_OPTICAL_CAL_DATA,
	APP_STATE_OPTICAL_CAL_RESULT,
    APP_STATE_FORCE_CAL_RDY,
    APP_STATE_VIB_CAL_RDY,
	APP_STATE_HALL_EFFECT_RESULT,
    APP_STATE_HALL_EFFECT_TEST,
    APP_STATE_PCB_TEST,
	APP_STATE_ACCELEROMETER,
    APP_STATE_DEVICE_INFO,
    APP_STATE_DEBUG_FILE,
    APP_STATE_DEBUG_REC_TEST,
    APP_STATE_PROFILE_TRANSFER,
    APP_STATE_RAW_DATA_RECEIVE,
    APP_STATE_PROBE_ERROR,
    APP_STATE_REQUEST_PROFILE,
    APP_STATE_SPIS_FAIL,
    APP_STATE_RAW_SUB_DATA_RECEIVE,
    APP_STATE_NEW_SN,
    APP_STATE_X_MODEM,
    APP_STATE_START_TEST,
    APP_STATE_NEW_ID,
    APP_STATE_PIC_FWU_START,
    APP_STATE_FWU_DATA_SEND,
    APP_STATE_FWU_ACK,
    APP_STATE_FWU_ERROR,
    APP_STATE_FWU_DONE,
    APP_STATE_RESTART,
    APP_STATE_START_ARM_FWU,
    APP_STATE_START_SQUAL_CAL,
    APP_STATE_SQUAL_CAL_RESULT,
    APP_STATE_START_OPTICAL_CAL,
    APP_STATE_SET_PIC_CAL,
    APP_STATE_START_VIB_CAL,
    APP_STATE_BATTERY_VOLTAGE,
    APP_STATE_TIME_AND_LOC,
} APP_STATES;





/****** DEVICE INFO STRUCT *****/
typedef struct{
    char serial_number[6];
    char device_name[32];
    char PIC_firmware_version[7]; //X.YY.ZZ,
    char ARM_firmware_version[7]; //X.YY.ZZ
    uint16_t number_of_tests;
    uint8_t battery_capacity;
    uint8_t current_flash_bank;
}device_info_t;
#define BYTES_OF_DEVICE_INFO sizeof(device_data_t)
extern device_info_t device_info;

#define BYTES_OF_METADATA sizeof(data_header_t)
#define MAX_BYTES_OF_METADATA 256

#define PROFILE_MAX_COUNT 3000
typedef struct profile_data
{
    uint8_t metadata[MAX_BYTES_OF_METADATA];
    uint8_t profile[PROFILE_MAX_COUNT];
}profile_data_t;
extern profile_data_t profile_data;



/******** RAW DATA STRUCT *******/
#define POINTS_PER_RAW_SIGNAL 1000
typedef struct raw_sub_data
{
    uint8_t metadata[MAX_BYTES_OF_METADATA];
    uint8_t test_number;
    uint8_t profile[1500];
    uint16_t force[POINTS_PER_RAW_SIGNAL];
    int8_t OpticalY[POINTS_PER_RAW_SIGNAL];
    uint8_t OpticalSqual[POINTS_PER_RAW_SIGNAL];
}subsampled_raw_data_t;
#define BYTES_RAW_SUB_DATA sizeof(subsampled_raw_data_t)
    
#define RAW_DATA_BUFFER_SIZE 2000
#define BYTES_RAW_TEST_DATA 60264
extern uint8_t raw_data_buff[RAW_DATA_BUFFER_SIZE];

typedef struct profile_id
{
    uint16_t type;
    uint16_t test_num;
}profile_id_t;

typedef struct imu_data
{
    int16_t    ax;
    int16_t    ay;
    int16_t    az;
//    int16_t    gx;
//    int16_t    gy;
//    int16_t    gz;
} imu_data_t;

typedef struct time_location
{
    uint32_t seconds_since_2001;
    float lat_long[2];
}time_location_t;

typedef struct
{
    /* The application's current state */
    APP_STATES state;
    APP_STATES prev_state;
    profile_id_t profile_id;
    uint16_t new_profile_num;
    uint16_t profile_size;
    uint16_t ble_status;
	bool ble_disconnect_flag;
    uint8_t status;
    time_location_t time_and_loc;
    uint8_t probe_error_code;
    bool send_imu_flag;
    bool imu_enabled;
    bool imu_paused;
    uint16_t data_counts;
    bool SPIS_timeout_flag;
    bool transfer_in_progress;
    volatile uint8_t ack;
    volatile uint8_t ack_retry;
    uint8_t fwu_data_buf[256];
} APP_DATA;
extern APP_DATA appData;
	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize (void)

  Summary:
     Flight Sensor Package application initialization routine.

  Description:
    This function initializes the Flight Sensor Package application.  It places 
    the application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APP_Initialize (void);


/*******************************************************************************
  Function:
    void APP_Tasks (void)

  Summary:
    Flght Sensor Package application tasks function

  Description:
    This routine is the Flight Sensor Package application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_Tasks(void);


/*******************************************************************************
  Function:
    void prompt(void);

  Summary:
    UART prompt.

  Description:
    This routine sends a prompt to UART.

  Precondition:
    Initialization of UART.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    prompt();
    </code>

  Remarks:
    None.
 */

void prompt(void);


/*******************************************************************************
  Function:
    void monitor(void);

  Summary:
    Terminal Interface.

  Description:
    This routine allows UART input and output to application.

  Precondition:
    Initialization of UART.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    monitor();
    </code>

  Remarks:
    None.
 */

void monitor(void);

#endif	/* APP_H */

/*******************************************************************************
 End of File
 */
