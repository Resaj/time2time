/**********************************************************************
 * Project: Time2time
 * 
 * File description: This code file contains the parameters and 
 * functions for controlling the state machine and the different modes 
 * of the chronometer
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
#include "buttons.h"
#include "buzzer.h"
#include "display.h"
#include "led_rgb.h"
#include "pass_sensor.h"
#include "state_machine.h"
#include "supply.h"

/**********************************************************************
 * Defines & enums
 *********************************************************************/

enum program_state {
  INIT_STATE,
  MAIN_MENU,
  GET_TIME
};

enum program_substate {
  INIT_SUBSTATE,
  WAIT_FOR_START,
  SHOW_TIME_WITHOUT_DETECTION,
  WAIT_FOR_DETECTION,
  SHOW_TIME_DETECTION,
  DETECTED_AND_SENSING_LOCKED,
  DETECTED_AND_SENSING_ACTIVED,
  SHOW_FINAL_TIME,
  FINISHED
};

enum get_time_mode {
  NORMAL_LAP_TIME_MODE,
  X_LAPS_TIME_MODE,
  START_STOP_MODE
};

/**********************************************************************
 * Local functions
 *********************************************************************/

program_substate substate = INIT_SUBSTATE;

/**********************************************************************
 * Local functions
 *********************************************************************/

/**********************************************************************
 * @brief Initializes all the systems of time2time device
 */
void init_t2t(void)
{
  batt_monitor_init();
  buttons_init();
  pass_sensor_init();
  buzzer_init();
  display_init();
  rgb_led_init();
}

/**********************************************************************
 * @brief Shows the main menu with the different t2t options
 */
void show_main_menu(void)
{
  s_display_text text[4] = {
    /* Text                   , pos_X , pos_Y , font      , aligment */
    { "Select mode:"          , 0     , 0     , MENU_FONT , ALIGN_LEFT },
    { "A: Normal lap time"    , 0     , 15    , MENU_FONT , ALIGN_LEFT },
    { "B: 3 laps time"        , 0     , 30    , MENU_FONT , ALIGN_LEFT },
    { "C: Star/Stop (2x t2t)" , 0     , 45    , MENU_FONT , ALIGN_LEFT }
  };

  display_set_data(text);
}

/**********************************************************************
 * @brief Executes the cyclic time measurement mode and shows the time 
 * of each lap
 */
void normal_lap_time_mode(void)
{
  unsigned long t_now = 0;
  static unsigned long time_last_lap_ms = 0;
  static unsigned long time_last_detection = 0;
  static unsigned long best_time_lap_ms = 1000000;
  static s_display_text text[2] = {
    /* Text , pos_X , pos_Y , font                , aligment */
    {  ""   , 0     , 0     , MAIN_TIME_FONT      , ALIGN_LEFT },
    {  ""   , 30    , 50    , SECONDARY_TIME_FONT , ALIGN_LEFT }
  };

  switch(substate)
  {
    case INIT_SUBSTATE:
      /* substate actions */
      sprintf(text[0].text, "0.000");
      sprintf(text[1].text, "Ready");
      display_set_data(text);
      
      release_sensor_detection();
      
      /* test substate changes */
      substate = WAIT_FOR_START;
      break;
      
    case WAIT_FOR_START:
      /* substate actions */

      /* test substate changes */
      if(sensor_interrupt_flag)
      {
        time_last_detection = time_detection;
        substate = SHOW_TIME_WITHOUT_DETECTION;
      }
      break;
      
    case SHOW_TIME_WITHOUT_DETECTION:
      /* substate actions */
      t_now = millis();
      sprintf(text[0].text, "%.3f", (t_now - time_last_detection)/1000.0);
      sprintf(text[1].text, "Last lap: %.3f", time_last_lap_ms/1000.0);
      display_set_data(text);
      
      set_buzzer_mode(SIMPLE_BEEP);

      /* test substate changes */
      if(t_now - time_last_detection > 1000)
      {
        release_sensor_detection();
        substate = WAIT_FOR_DETECTION;
      }
      break;
    
    case WAIT_FOR_DETECTION:
      /* substate actions */
      sprintf(text[0].text, "%.3f", (millis() - time_last_detection)/1000.0);
      sprintf(text[1].text, "Last lap: %.3f", time_last_lap_ms/1000.0);
      display_set_data(text);

      /* test substate changes */
      if(sensor_interrupt_flag)
        substate = SHOW_TIME_DETECTION;
      break;
    
    case SHOW_TIME_DETECTION:
      /* substate actions */
      sprintf(text[0].text, "%.3f", (time_detection - time_last_detection)/1000.0);
      sprintf(text[1].text, "Last lap: %.3f", time_last_lap_ms/1000.0);
      display_set_data(text);

      if((time_detection - time_last_detection) < best_time_lap_ms)
      {
        best_time_lap_ms = time_detection - time_last_detection;
        set_buzzer_mode(DOUBLE_BEEP);
      }
      else
        set_buzzer_mode(SIMPLE_BEEP);

      /* test substate changes */
      substate = DETECTED_AND_SENSING_LOCKED;
      break;
      
    case DETECTED_AND_SENSING_LOCKED:
      /* substate actions */

      /* test substate changes */
      if(millis() - time_detection > 1000)
      {
        time_last_lap_ms = time_detection - time_last_detection;
        time_last_detection = time_detection;
        release_sensor_detection();
        substate = DETECTED_AND_SENSING_ACTIVED;
      }
      break;
      
    case DETECTED_AND_SENSING_ACTIVED:
      /* substate actions */

      /* test substate changes */
      if(sensor_interrupt_flag)
        substate = SHOW_TIME_DETECTION;
      else if(millis() - time_last_detection > 1000)
        substate = WAIT_FOR_DETECTION;
      break;
      
    default:
      break;
  }
}

/**********************************************************************
 * @brief Executes the cyclic time measurement mode for the number of 
 * selected laps and shows the time of each lap and the total one
 */
void x_laps_time_mode(void)
{
  //todo: configure menu to chose number of laps
  
  unsigned long t_now = 0;
  static unsigned long time_init = 0;
  static unsigned long time_last_detection = 0;
  static unsigned long best_time_lap_ms = 1000000;
  static unsigned long best_time_total = 1000000;
  unsigned int target_laps = 3;
  unsigned int laps_to_go = 0;

  static s_display_text text[2] = {
    /* text , pos_X , pos_Y , font                , aligment */
    {  ""   , 0     , 0     , MAIN_TIME_FONT      , ALIGN_LEFT },
    {  ""   , 30    , 50    , SECONDARY_TIME_FONT , ALIGN_LEFT }
  };

  switch(substate)
  {
    case INIT_SUBSTATE:
      /* substate actions */
      laps_to_go = target_laps;
      
      sprintf(text[0].text, "0.000");
      sprintf(text[1].text, "Ready");
      display_set_data(text);
      
      release_sensor_detection();
      
      /* test substate changes */
      substate = WAIT_FOR_START;
      break;
      
    case WAIT_FOR_START:
      /* substate actions */

      /* test substate changes */
      if(sensor_interrupt_flag)
      {
        time_init = time_detection;
        time_last_detection = time_init;
        substate = SHOW_TIME_WITHOUT_DETECTION;
      }
      break;
      
    case SHOW_TIME_WITHOUT_DETECTION:
      /* substate actions */
      t_now = millis();
      sprintf(text[0].text, "%.3f", (t_now - time_init)/1000.0);
      sprintf(text[1].text, "Laps to go: %u", laps_to_go);
      display_set_data(text);

      set_buzzer_mode(SIMPLE_BEEP);

      /* test substate changes */
      if(t_now - time_detection > 1000)
      {
        release_sensor_detection();
        substate = WAIT_FOR_DETECTION;
      }
    
      break;
      
    case WAIT_FOR_DETECTION:
      /* substate actions */
      t_now = millis();
      sprintf(text[0].text, "%.3f", (t_now - time_init)/1000.0);
      sprintf(text[1].text, "Laps to go: %u", laps_to_go);
      display_set_data(text);

      /* test substate changes */
      if(sensor_interrupt_flag)
        substate = SHOW_TIME_DETECTION;
      break;
      
    case SHOW_TIME_DETECTION:
      /* substate actions */
      laps_to_go--;

      sprintf(text[0].text, "%.3f", (time_detection - time_last_detection)/1000.0);
      sprintf(text[1].text, "Laps to go: %u", laps_to_go);
      display_set_data(text);

      if((time_detection - time_last_detection) < best_time_lap_ms)
      {
        best_time_lap_ms = time_detection - time_last_detection;
        set_buzzer_mode(DOUBLE_BEEP);
      }
      else
        set_buzzer_mode(SIMPLE_BEEP);

      /* test substate changes */
      substate = DETECTED_AND_SENSING_LOCKED;
      break;
      
    case DETECTED_AND_SENSING_LOCKED:
      /* substate actions */

      /* test substate changes */
      if(millis() - time_detection > 1000)
      {
        if(laps_to_go == 0)
          substate = SHOW_FINAL_TIME;
        else
        {
          time_last_detection = time_detection;
          release_sensor_detection();
          substate = DETECTED_AND_SENSING_ACTIVED;
        }
      }
      break;
      
    case DETECTED_AND_SENSING_ACTIVED:
      /* substate actions */

      /* test substate changes */
      if(sensor_interrupt_flag)
        substate = SHOW_TIME_DETECTION;
      else if(millis() - time_last_detection > 1000)
        substate = WAIT_FOR_DETECTION;
      break;
      
    case SHOW_FINAL_TIME:
      /* substate actions */
      //todo: test if the final time is a new record and, in that case, save it and do triple beep with the buzzer
      
      sprintf(text[0].text, "%.3f", (time_detection - time_init)/1000.0);
      sprintf(text[1].text, "Press B to restart");
      display_set_data(text);

      /* test substate changes */
      substate = FINISHED;
      break;

    case FINISHED:
      /* substate actions */

      /* test substate changes */
      if(get_button_state(BUTTON_B))
        substate = INIT_SUBSTATE;
      break;
      
    default:
      break;
  }
}

/**********************************************************************
 * @brief Executes the start/stop time measurement mode and shows the 
 * time of the complete circuit
 */
void start_stop_mode(void)
{
  //todo: program start/stop operating mode
  set_rgb_blue(LED_ON);
}

/**********************************************************************
 * @brief Calls the function which is corresponded with the selected 
 * t2t functionament mode
 * 
 * @param t2t_mode: time measurement mode
 */
void get_time(get_time_mode t2t_mode)
{
  switch(t2t_mode)
  {
    case NORMAL_LAP_TIME_MODE:
      normal_lap_time_mode();
      break;
      
    case X_LAPS_TIME_MODE:
      x_laps_time_mode();
      break;
      
    case START_STOP_MODE:
      start_stop_mode();
      break;
  }
}

/**********************************************************************
 * Global functions
 *********************************************************************/

/**********************************************************************
 * @brief Controls the state machine, which selects the different modes 
 * of the chronometer and the states of each one. This function has to 
 * be called periodically through an scheduler
 */
void state_machine_task(void)
{
  static program_state state = INIT_STATE;
  static get_time_mode t2t_mode = NORMAL_LAP_TIME_MODE;

  /* Main state machine control */
  switch(state)
  {
    case INIT_STATE:
      /* state actions */
      init_t2t();
      show_main_menu();

      /* test state changes */
      state = MAIN_MENU;
      
      break;
      
    case MAIN_MENU:
      /* state actions */

      /* test state changes */
      if(get_button_state(BUTTON_A))
      {
        t2t_mode = NORMAL_LAP_TIME_MODE;
        state = GET_TIME;
      }
      else if(get_button_state(BUTTON_B))
      {
        t2t_mode = X_LAPS_TIME_MODE;
        state = GET_TIME;
      }
      else if(get_button_state(BUTTON_C))
      {
        t2t_mode = START_STOP_MODE;
        state = GET_TIME;
      }

      break;
      
    case GET_TIME:
      /* state actions */
      get_time(t2t_mode);

      /* test state changes */
      if(get_button_state(BUTTON_C))
      {
        state = INIT_STATE;
        substate = INIT_SUBSTATE;
      }
      
      break;
      
    default:
      break;
  }
}
