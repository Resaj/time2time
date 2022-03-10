/**********************************************************************
 * Project: Time2time
 * 
 * File description: This code file contains the parameters and 
 * functions for controlling the scheduler of the program
 * 
 * Author: Rubén Espino San José
 * Puma Pride Robotics Team
 * 
 * License: Attribution-NonCommercial-ShareAlike 4.0
 * International (CC BY-NC-SA 4.0) 
 *********************************************************************/

/**********************************************************************
 * Includes
 *********************************************************************/

#include "buttons.h"
#include "buzzer.h"
#include "display.h"
#include "espnow_comm.h"
#include "led_rgb.h"
#include "pass_sensor.h"
#include "scheduler.h"
#include "state_machine.h"
#include "supply.h"

/**********************************************************************
 * Configuration parameters
 *********************************************************************/

#define TIMER_PERIOD 1u // ms

/**********************************************************************
 * Structs
 *********************************************************************/
 
typedef struct {
  const uint16_t period_ms;     // Timer period (ms)
  uint32_t t_last_execution;    // Instant of last execution (ms)
  void (*execute_tasks)(void);  // Pointer to a function which contains the tasks to execute
} s_timer;

/**********************************************************************
 * Local variables
 *********************************************************************/

uint32_t t_now_ms = 0;
hw_timer_t *timer = NULL;
portMUX_TYPE scheduler_critical_zone = portMUX_INITIALIZER_UNLOCKED;  // Needed for the interruption
uint8_t interruptCounter;

/**********************************************************************
 * Local functions declarations
 *********************************************************************/

void IRAM_ATTR scheduler_isr(void);
void scheduler_init(void);
void tasks_10ms(void);
void tasks_50ms(void);
void tasks_500ms(void);

/**********************************************************************
 * Local functions
 *********************************************************************/

/**********************************************************************
 * @brief Timer interruption. The program enters periodicaly for 
 * controlling the times of scheduler execution
 */
void IRAM_ATTR scheduler_isr(void)
{
  portENTER_CRITICAL_ISR(&scheduler_critical_zone);
  interruptCounter++;
  t_now_ms += TIMER_PERIOD;
  portEXIT_CRITICAL_ISR(&scheduler_critical_zone);
}

/**********************************************************************
 * @brief Configures the timer which will be control the scheduler 
 * execution
 */
void scheduler_init(void)
{
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &scheduler_isr, true);
  timerAlarmWrite(timer, TIMER_PERIOD*1000, true);
  timerAlarmEnable(timer);
}

/**********************************************************************
 * @brief Contains the tasks list that will be execute every 10 ms
 */
void tasks_10ms(void)
{
  buttons_task();
  buzzer_task();
  espnow_task();
  state_machine_task();
  led_task();
}

/**********************************************************************
 * @brief Contains the tasks list that will be execute every 50 ms
 */
void tasks_50ms(void)
{
  display_task();
}

/**********************************************************************
 * @brief Contains the tasks list that will be execute every 500 ms
 */
void tasks_500ms(void)
{
  supply_task();
}

/**********************************************************************
 * Global functions
 *********************************************************************/

/**********************************************************************
 * @brief Calls all the peripheral configuration functions. This 
 * function has to be called through initial setup function
 */
void init_setup_task(void)
{
  batt_monitor_init();
  buttons_init();
  pass_sensor_init();
  buzzer_init();
  display_init();
  rgb_led_init();
  espnow_comm_init();

  scheduler_init();
}

/**********************************************************************
 * @brief Controls the tasks execution. This function has to be called 
 * through main loop
 */
void scheduler_task(void)
{
  static s_timer timer_scheduler[] = {
    /* period_ms , t_last_execution , execute_tasks */
    {  10        , 0                , &tasks_10ms   },
    {  50        , 0                , &tasks_50ms   },
    {  500       , 0                , &tasks_500ms  }
  };
  static uint8_t num_timers = sizeof(timer_scheduler) / sizeof(s_timer);

  if (interruptCounter > 0)
  {
    uint32_t t_now = 0;
    
    portENTER_CRITICAL(&scheduler_critical_zone);
    interruptCounter--;
    portEXIT_CRITICAL(&scheduler_critical_zone);

    t_now = get_currentTimeMs();
    
    for(int8_t i=0; i < num_timers; i++)
    {
      if(t_now - timer_scheduler[i].t_last_execution >= timer_scheduler[i].period_ms)
      {
        timer_scheduler[i].t_last_execution = t_now;
        timer_scheduler[i].execute_tasks();
      }
    }
  }
}

/**********************************************************************
 * @brief Returns the current time since the start of the program, in
 * milliseconds
 * 
 * @return: time in milliseconds
 */
uint32_t get_currentTimeMs(void)
{
  return t_now_ms;
}
