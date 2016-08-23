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
    Richard Kirby
 
  Created on:
    June 1, 2016
  
  Revision History:
    Development version      June 1, 2016
 *******************************************************************************/

#ifndef _APP_H
#define _APP_H

#define CALIBRATION 0

#include <stdint.h>

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
    APP_STATE_PROFILE_TRANSFER,
    APP_STATE_RAW_DATA_RECEIVE,
    APP_STATE_TRANSFER_PROFILE_IDS,
    APP_STATE_PROBE_ERROR,
    APP_STATE_SEND_PROFILE_ID,
} APP_STATES;

/****** DEVICE INFO STRUCT *****/
typedef struct{
    char serial_number[6];
    char device_name[32];
    uint16_t number_of_tests;
    uint8_t battery_capacity;
}device_info_t;
#define BYTES_OF_DEVICE_INFO sizeof(device_data_t)
extern device_info_t device_info;
    
/***** meta data struct *****/
typedef struct  data_header
{
    /**********  Environmental ************/
    int8_t      temperature; // in degrees C
    //Date and Time
    float       location[2];//Location
    
    /*********** Test Specific  ***********/
    uint16_t    test_num; //test number (device specific)
    uint16_t    profile_depth;
    uint8_t     battery_capacity;
    float       test_time; //seconds
    uint8_t     error_code;
    
    /********** Device Specific  *************/
    uint8_t     accel_FS; //full scale of accelerometer
    uint16_t    gyro_FS; 
    uint16_t    force_cal[5]; //in ADC counts
    uint16_t    optical_cal;
    
    /*************  User Settings ************/
    
    /*************  Versions and Revisions ************/
    char        serial_number[8]; // device serial #
    char        PIC_firmware_version[8]; //ex. 1.0.3
    char        ARM_firmware_version[8];
    uint8_t     main_PCB_rev; //ex. 3
    uint8_t     NRF_PCB_rev;
   
    //add the rest here...
} data_header_t;
#define BYTES_OF_METADATA sizeof(data_header_t)
extern data_header_t metadata;

#define PROFILE_MAX_COUNT 3000
typedef struct profile_data
{
    data_header_t metadata;
    uint8_t profile[PROFILE_MAX_COUNT];
}profile_data_t;
extern profile_data_t profile_data;



/******** RAW DATA STRUCT *******/
#define POINTS_PER_RAW_SIGNAL 1000
typedef struct
{
    //data_header_t metadata;
    uint8_t test_number;
    uint8_t profile[1500];
    uint16_t force[POINTS_PER_RAW_SIGNAL];
    int8_t OpticalY[POINTS_PER_RAW_SIGNAL];
    uint8_t OpticalSqual[POINTS_PER_RAW_SIGNAL];
}subsampled_raw_data_t;
#define BYTES_RAW_DATA sizeof(subsampled_raw_data_t)
    
#define RAW_DATA_BUFFER_SIZE 2000
#define BYTES_RAW_TEST_DATA 41076
extern uint8_t raw_data_buff[RAW_DATA_BUFFER_SIZE];

typedef struct profile_id
{
    uint16_t type;
    uint16_t test_num;
}profile_id_t;

typedef struct
{
    /* The application's current state */
    APP_STATES state;
    APP_STATES prev_state;
    profile_id_t profile_id;
    uint16_t ble_status;
    uint8_t status;
    uint8_t accelerometer_enable;

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
