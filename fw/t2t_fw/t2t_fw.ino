/**********************************************************************
 * Project: Time2time
 * 
 * File description: This program is used as a chronometer to measure 
 * times in robotics competitions.
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
#include "led_rgb.h"
#include "pass_sensor.h"
#include "state_machine.h"
#include "supply.h"

/**********************************************************************
 * Structs
 *********************************************************************/
 
typedef struct {
  const unsigned int period_ms;   // Timer period (ms)
  unsigned long t_last_execution; // Instant of last execution (ms)
  void (*execute_tasks)(void);    // Pointer to a function which contains the tasks to execute
} s_timer;

/**********************************************************************
 * Local functions
 *********************************************************************/
 
void tasks_10ms(void)
{
  buttons_task();
  buzzer_task();
  state_machine_task();
  leds_task();
  display_task();
}

void tasks_500ms(void)
{
  supply_task();
}

void setup() {
  batt_monitor_init();
  buttons_init();
  pass_sensor_init();
  buzzer_init();
  display_init();
  rgb_led_init();
}

void loop() {
  static unsigned long t_now;
  static s_timer timer_scheduler[2] = {
    /* period_ms , t_last_execution , execute_tasks */
    {  10        , 0                , &tasks_10ms   },
    {  500       , 0                , &tasks_500ms  }
  };
  static unsigned char num_timers = sizeof(timer_scheduler) / sizeof(s_timer);

  t_now = millis();

  for(int i = 0; i < num_timers; i++)
  {
    if(t_now - timer_scheduler[i].t_last_execution >= timer_scheduler[i].period_ms)
    {
      timer_scheduler[i].t_last_execution = t_now;
      timer_scheduler[i].execute_tasks();
    }
  }
  
  //todo: manage UART communication
  //todo: manage Wifi communication
}

void sleep(void)
{
  //todo: red led to blink state
  //      shut down the display
  //      disable sensor interrupt
  power_12v_disable();
}

void wake_up(void)
{
  power_12v_enable();
  //todo: disable sensor interrupt
  //      turn on the display
  //      shut down red led
}
