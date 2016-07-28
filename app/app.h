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
    APP_STATE_DEVICE_INFO,
} APP_STATES;




typedef struct
{
    /* The application's current state */
    APP_STATES state;

} APP_DATA;
extern APP_DATA appData;


/****** DEVICE INFO STRUCT *****/
typedef struct{
    uint8_t number_of_tests;
    char serial_number[6];
    char device_name[32];
    uint8_t battery_capacity;
}device_info_t;
#define BYTES_OF_DEVICE_INFO sizeof(device_data_t)
extern device_info_t device_info;
	
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
