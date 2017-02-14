#ifndef TIMERS_H__
#define TIMERS_H__

#include "app_timer.h"


/*************************   DEFINITION   *************************/
#define TIMER_PRESCALER                      4 //4 corelates to 1us ticks
#define CC_DELAY                             50 //50 means every 50us the interrupt will trigger

#define APP_TIMER_PRESCALER                  3    /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE              4    /**< Size of timer operation queues. */

#define battery_LEVEL_MEAS_INTERVAL          APP_TIMER_TICKS(100, APP_TIMER_PRESCALER) /**< Battery level measurement interval (ticks). */                         
#define slope_LEVEL_MEAS_INTERVAL          	 APP_TIMER_TICKS(500, APP_TIMER_PRESCALER) /**< slope level measurement interval (ticks). */
#define status_LEVEL_MEAS_INTERVAL           APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER) /**< status level measurement interval (ticks). */
#define acc_LEVEL_MEAS_INTERVAL              APP_TIMER_TICKS(2, APP_TIMER_PRESCALER)


//defines variables to be used for app timers
APP_TIMER_DEF(m_acc_timer_id);
APP_TIMER_DEF(m_slope_timer_id);
APP_TIMER_DEF(m_battery_timer_id);     /**< Battery timer. */
APP_TIMER_DEF(m_status_timer_id);      /**< Status timer. */

/*************************  FUNCTIONS   ***************************/
void timers_init(void);
void application_timers_start(void);
void application_timers_stop(void);
void acc_timeout_handler(void *p_context);
void battery_timeout_handler(void *p_context);
void slope_timeout_handler(void *p_context);
void status_timeout_handler(void *p_context);
void enable_imu(void);
void disable_imu(void);

void start_ARM_RDY_timer(void);
void TIMER2_IRQHandler(void);

#endif //TIMERS_H__