/**********************************************************************
 * Project: Time2time
 * 
 * File description: This code file contains the parameters and 
 * functions for controlling a buzzer
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

#include "Arduino.h"
#include "config/PINSEL.h"
#include "buzzer.h"
#include "scheduler.h"

/**********************************************************************
 * Defines & enums
 *********************************************************************/

#define BUZZER_STATE_BUFFER_SIZE   30

#define BUZZER_ON   HIGH
#define BUZZER_OFF  LOW

enum e_buzzer_state {
  BUZZER_BEEP_INIT,       // Inits the beep sound
  BUZZER_SHORT_BEEP,      // Maintains beep during SHORT_BEEP_TIME
  BUZZER_LARGE_BEEP,      // Maintains beep during LARGE_BEEP_TIME
  BUZZER_SHORT_MUTE,      // Maintains silence during SHORT_MUTE_TIME and beeps again
  BUZZER_LARGE_MUTE,      // Maintains silence during LARGE_MUTE_TIME and beeps again
  BUZZER_MUTE_AND_STOP    // Maintains silence during LARGE_MUTE_TIME and stops the actual buzzer mode from e_buzzer_mode
};

/**********************************************************************
 * Configuration parameters
 *********************************************************************/

#define PWM_FREQ_HZ         2000u
#define PWM_CHANNEL         0u
#define PWM_RESOLUTION      8u
#define PWM_DUTY_CYCLE_ON   (uint16_t)255/2 // Value between 0 and 255
#define PWM_DUTY_CYCLE_OFF  0u

#define SHORT_BEEP_TIME           50u   // ms
#define LARGE_BEEP_TIME           800u  // ms
#define MUTE_TIME_BETWEEN_BEEPS   90u   // ms
#define MIN_MUTE_TIME_AT_FINISH   400u  // ms
#define SHORT_MUTE_TIME           MUTE_TIME_BETWEEN_BEEPS
#define LARGE_MUTE_TIME           MIN_MUTE_TIME_AT_FINISH

/**********************************************************************
 * Local variables
 *********************************************************************/

e_buzzer_state buzzer_state[BUZZER_STATE_BUFFER_SIZE];
uint8_t buzzer_state_actual_index = 0;
uint8_t buzzer_state_next_empty_index = 0;

/**********************************************************************
 * Local functions
 *********************************************************************/

/**********************************************************************
 * @brief Sets on/off the buzzer
 * 
 * @param state: 1 (on) or 0 (off)
 */
void buzzer_pwm_set(uint8_t state)
{
  ledcWrite(PWM_CHANNEL, state? PWM_DUTY_CYCLE_ON : PWM_DUTY_CYCLE_OFF);
}

/**********************************************************************
 * @brief Adds the selected state to the buzzer task circular buffer 
 * and controls the index of the next empty buffer position
 * 
 * @param state: see the available values at e_buzzer_state
 */
void add_buzzer_state(e_buzzer_state state)
{
  buzzer_state[buzzer_state_next_empty_index] = state;
  buzzer_state_next_empty_index = (buzzer_state_next_empty_index == BUZZER_STATE_BUFFER_SIZE - 1) ? 0 : buzzer_state_next_empty_index + 1;
}

/**********************************************************************
 * @brief Execute the next task of the buzzer circular buffer. This 
 * function controls the beeps and their duration
 * 
 * The states are defined at e_buzzer_state.
 * 
 * @returns done: 1 = done; 0 = in progress
 */
uint8_t do_buzzer_task(void)
{
  static uint32_t t = 0;
  uint32_t t_now = 0;
  uint8_t done = 0;
  
  switch(buzzer_state[buzzer_state_actual_index])
  {
    case BUZZER_BEEP_INIT:
      buzzer_pwm_set(BUZZER_ON);
      t = t_now_ms;
      done = 1;
      break;
      
    case BUZZER_SHORT_BEEP:
      t_now = t_now_ms;
      if(t_now - t >= SHORT_BEEP_TIME)
      {
        buzzer_pwm_set(BUZZER_OFF);
        t = t_now;
        done = 1;
      }
      break;

    case BUZZER_LARGE_BEEP:
      t_now = t_now_ms;
      if(t_now - t >= LARGE_BEEP_TIME)
      {
        buzzer_pwm_set(BUZZER_OFF);
        t = t_now;
        done = 1;
      }
      break;

    case BUZZER_SHORT_MUTE:
      t_now = t_now_ms;
      if(t_now - t >= SHORT_MUTE_TIME)
      {
        buzzer_pwm_set(BUZZER_ON);
        t = t_now;
        done = 1;
      }
      break;

    case BUZZER_LARGE_MUTE:
      t_now = t_now_ms;
      if(t_now - t >= LARGE_MUTE_TIME)
      {
        buzzer_pwm_set(BUZZER_ON);
        t = t_now;
        done = 1;
      }
      break;

    case BUZZER_MUTE_AND_STOP:
      if(t_now_ms - t >= MIN_MUTE_TIME_AT_FINISH)
        done = 1;
      break;

    default:
      break;
  }

  return done;
}

/**********************************************************************
 * Global functions
 *********************************************************************/

/**********************************************************************
 * @brief Configures the buzzer pin as PWM output
 */
void buzzer_init(void)
{
  ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcAttachPin(PIN_BUZZER_PWM, PWM_CHANNEL);
}

/**********************************************************************
 * @brief Sets the buzzer sound mode
 * 
 * @param selected_mode: see the available values at e_buzzer_mode
 */
void set_buzzer_mode(e_buzzer_mode selected_mode)
{
  switch(selected_mode)
  {
    case SIMPLE_BEEP:
      add_buzzer_state(BUZZER_BEEP_INIT);
      add_buzzer_state(BUZZER_SHORT_BEEP);
      add_buzzer_state(BUZZER_MUTE_AND_STOP);
      break;
      
    case DOUBLE_BEEP:
      add_buzzer_state(BUZZER_BEEP_INIT);
      add_buzzer_state(BUZZER_SHORT_BEEP);
      add_buzzer_state(BUZZER_SHORT_MUTE);
      add_buzzer_state(BUZZER_SHORT_BEEP);
      add_buzzer_state(BUZZER_MUTE_AND_STOP);
      break;
      
    case TRIPLE_BEEP:
      add_buzzer_state(BUZZER_BEEP_INIT);
      add_buzzer_state(BUZZER_SHORT_BEEP);
      add_buzzer_state(BUZZER_SHORT_MUTE);
      add_buzzer_state(BUZZER_SHORT_BEEP);
      add_buzzer_state(BUZZER_SHORT_MUTE);
      add_buzzer_state(BUZZER_SHORT_BEEP);
      add_buzzer_state(BUZZER_MUTE_AND_STOP);
      break;
      
    case LARGE_BEEP:
      add_buzzer_state(BUZZER_BEEP_INIT);
      add_buzzer_state(BUZZER_LARGE_BEEP);
      add_buzzer_state(BUZZER_MUTE_AND_STOP);
      break;

    default:
      break;
  }
}

/**********************************************************************
 * @brief Makes the buzzer sounds. This function has to be called 
 * periodically through an scheduler
 */
void buzzer_task(void)
{
  if(buzzer_state_actual_index != buzzer_state_next_empty_index)
    if(do_buzzer_task())
      buzzer_state_actual_index = (buzzer_state_actual_index == BUZZER_STATE_BUFFER_SIZE - 1) ? 0 : buzzer_state_actual_index + 1;
}
