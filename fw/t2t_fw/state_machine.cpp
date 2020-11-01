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

#include "buttons.h"
#include "buzzer.h"
#include "display.h"
#include "led_rgb.h"
#include "pass_sensor.h"
#include "scheduler.h"
#include "state_machine.h"
#include "supply.h"

/**********************************************************************
 * Defines & enums
 *********************************************************************/

#define BEST_TIME_INIT_VALUE  1000000

enum program_state {
  INIT_STATE,
  MAIN_MENU,
  SET_NUM_LAPS,
  GET_TIME
};

enum program_substate {
  INIT_SUBSTATE,
  WAIT_FOR_LAPS_SELECTION,
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
  START_STOP_MODE,
  TOAST_MODE
};

typedef struct {
  get_time_mode     time_mode;
  char              text_in_menu[22];
  program_state     next_menu_state;
  program_substate  first_subtate;
} s_mode_data;

/**********************************************************************
 * Local variables
 *********************************************************************/

program_substate substate = INIT_SUBSTATE;

uint16_t x_time_mode_target_laps = 3;
uint32_t best_time_lap_ms = BEST_TIME_INIT_VALUE;
uint32_t best_time_total_ms = BEST_TIME_INIT_VALUE;

s_mode_data mode_data[] = {
  /* time_mode           , text_in_menu          , next_menu_state , first_subtate */
  { NORMAL_LAP_TIME_MODE , "Normal lap time"     , GET_TIME        , INIT_SUBSTATE },
  { X_LAPS_TIME_MODE     , "X laps time"         , SET_NUM_LAPS    , INIT_SUBSTATE },
  { START_STOP_MODE      , "Start/Stop (2x t2t)" , GET_TIME        , INIT_SUBSTATE },
  { TOAST_MODE           , "Get toast time"      , GET_TIME        , INIT_SUBSTATE }
};

/**********************************************************************
 * Local functions
 *********************************************************************/

/**********************************************************************
 * @brief Shows the main menu with the different t2t options
 * 
 * @param group_num: number of menu options group to display on
 *    the screen
 */
void show_main_menu(uint8_t group_num)
{
  s_display_text text[] = {
    /* text          , pos_X , pos_Y , font      , aligment */
    { "Select mode:" , 0     , 0     , MENU_FONT , ALIGN_LEFT },
    { ""             , 0     , 15    , MENU_FONT , ALIGN_LEFT },
    { ""             , 0     , 30    , MENU_FONT , ALIGN_LEFT },
    { "C: ..."       , 0     , 45    , MENU_FONT , ALIGN_LEFT }
  };

  sprintf(text[1].text, "A: %s", mode_data[group_num*2].text_in_menu);
  if((group_num*2 + 1) < sizeof(mode_data)/sizeof(s_mode_data))
    sprintf(text[2].text, "B: %s", mode_data[group_num*2 + 1].text_in_menu);
  else
    sprintf(text[2].text, "");
  if(2 < sizeof(mode_data)/sizeof(s_mode_data))
    sprintf(text[3].text, "C: ...");
  else
    sprintf(text[3].text, "");

  display_set_data(text, sizeof(text)/sizeof(s_display_text));
}

/**********************************************************************
 * @brief Executes the cyclic time measurement mode and shows the time 
 * of each lap
 */
void normal_lap_time_mode(void)
{
  uint32_t t_now = 0;
  static uint32_t time_last_lap_ms = 0;
  static uint32_t time_last_detection = 0;
  static s_display_text text[] = {
    /* Text , pos_X , pos_Y , font                , aligment   */
    {  ""   , 0     , 0     , MAIN_TIME_FONT      , ALIGN_LEFT  },
    {  ""   , 120   , 50    , SECONDARY_TIME_FONT , ALIGN_RIGHT }
  };

  switch(substate)
  {
    case INIT_SUBSTATE:
      /* substate actions */
      sprintf(text[0].text, "0.000");
      sprintf(text[1].text, "Ready");
      display_set_data(text, sizeof(text)/sizeof(s_display_text));
      
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
        set_buzzer_mode(SIMPLE_BEEP);
        substate = SHOW_TIME_WITHOUT_DETECTION;
      }
      break;
      
    case SHOW_TIME_WITHOUT_DETECTION:
      /* substate actions */
      t_now = t_now_ms;
      sprintf(text[0].text, "%.3f", (t_now - time_last_detection)/1000.0);
      sprintf(text[1].text, "Last lap: %.3f", time_last_lap_ms/1000.0);
      display_set_data(text, sizeof(text)/sizeof(s_display_text));
      
      /* test substate changes */
      if(t_now - time_last_detection > 1000)
      {
        release_sensor_detection();
        substate = WAIT_FOR_DETECTION;
      }
      break;
    
    case WAIT_FOR_DETECTION:
      /* substate actions */
      sprintf(text[0].text, "%.3f", (t_now_ms - time_last_detection)/1000.0);
      sprintf(text[1].text, "Last lap: %.3f", time_last_lap_ms/1000.0);
      display_set_data(text, sizeof(text)/sizeof(s_display_text));

      /* test substate changes */
      if(sensor_interrupt_flag)
        substate = SHOW_TIME_DETECTION;
      break;
    
    case SHOW_TIME_DETECTION:
      /* substate actions */
      sprintf(text[0].text, "%.3f", (time_detection - time_last_detection)/1000.0);
      sprintf(text[1].text, "Last lap: %.3f", time_last_lap_ms/1000.0);
      display_set_data(text, sizeof(text)/sizeof(s_display_text));

      if((time_detection - time_last_detection) < best_time_lap_ms)
      {
        best_time_lap_ms = time_detection - time_last_detection;
        set_buzzer_mode(DOUBLE_BEEP);
      }
      else
        set_buzzer_mode(SIMPLE_BEEP);
      time_last_lap_ms = time_detection - time_last_detection;
      time_last_detection = time_detection;

      /* test substate changes */
      substate = DETECTED_AND_SENSING_LOCKED;
      break;
      
    case DETECTED_AND_SENSING_LOCKED:
      /* substate actions */

      /* test substate changes */
      if(t_now_ms - time_detection > 1000)
      {
        release_sensor_detection();
        substate = DETECTED_AND_SENSING_ACTIVED;
      }
      break;
      
    case DETECTED_AND_SENSING_ACTIVED:
      /* substate actions */

      /* test substate changes */
      if(sensor_interrupt_flag)
        substate = SHOW_TIME_DETECTION;
      else if(t_now_ms - time_last_detection > 2000)
        substate = WAIT_FOR_DETECTION;
      break;
      
    default:
      break;
  }
}

/**********************************************************************
 * @brief Executes the selection of the number of laps for the 
 * x_laps_time_mode
 * 
 * @returns true if the number of laps has been selected; false if not
 */
uint16_t x_laps_time_mode_laps_selection(void)
{
  static uint16_t target_laps_selection = x_time_mode_target_laps;

  static s_display_text text[] = {
    /* text                   , pos_X , pos_Y , font          , aligment   */
    { "Select number of laps:", 0     , 0     , MENU_FONT     , ALIGN_LEFT },
    { "A -> +1"               , 0     , 15    , MENU_FONT     , ALIGN_LEFT },
    { "B -> OK"               , 0     , 30    , MENU_FONT     , ALIGN_LEFT },
    { "C -> -1"               , 0     , 45    , MENU_FONT     , ALIGN_LEFT },
    { ""                      , 60    , 10    , MAIN_TIME_FONT, ALIGN_LEFT }
  };

  switch(substate)
  {
    case INIT_SUBSTATE:
      /* substate actions */
      sprintf(text[4].text, "%u", target_laps_selection);
      display_set_data(text, sizeof(text)/sizeof(s_display_text));

      /* test substate changes */
      substate = WAIT_FOR_LAPS_SELECTION;
      break;

    case WAIT_FOR_LAPS_SELECTION:
      /* substate actions */

      /* test substate changes */
      if(get_button_state(BUTTON_A))
      {
        target_laps_selection++;
        substate = INIT_SUBSTATE;
      }
      else if(get_button_state(BUTTON_B))
      {
        if(target_laps_selection != x_time_mode_target_laps)
        {
          best_time_total_ms = BEST_TIME_INIT_VALUE;
          x_time_mode_target_laps = target_laps_selection;
        }

        return true;
      }
      else if(get_button_state(BUTTON_C))
      {
        target_laps_selection = target_laps_selection > 1 ? target_laps_selection - 1 : target_laps_selection;
        substate = INIT_SUBSTATE;
      }
      break;

    default:
      break;
  }

  return false;
}

/**********************************************************************
 * @brief Executes the cyclic time measurement mode for the number of 
 * selected laps and shows the time of each lap and the total one
 */
void x_laps_time_mode(void)
{
  static uint32_t time_init = 0;
  static uint32_t time_last_detection = 0;
  static uint16_t laps_to_go = 0;
  uint32_t t_now = 0;

  static s_display_text text[] = {
    /* text , pos_X , pos_Y , font                , aligment   */
    {  ""   , 0     , 0     , MAIN_TIME_FONT      , ALIGN_LEFT  },
    {  ""   , 120   , 50    , SECONDARY_TIME_FONT , ALIGN_RIGHT }
  };

  switch(substate)
  {
    case INIT_SUBSTATE:
      /* substate actions */
      laps_to_go = x_time_mode_target_laps;
      
      sprintf(text[0].text, "0.000");
      sprintf(text[1].text, "Ready");
      display_set_data(text, sizeof(text)/sizeof(s_display_text));
      
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
        set_buzzer_mode(SIMPLE_BEEP);
        substate = SHOW_TIME_WITHOUT_DETECTION;
      }
      break;
      
    case SHOW_TIME_WITHOUT_DETECTION:
      /* substate actions */
      t_now = t_now_ms;
      sprintf(text[0].text, "%.3f", (t_now - time_init)/1000.0);
      sprintf(text[1].text, "Laps to go: %u", laps_to_go);
      display_set_data(text, sizeof(text)/sizeof(s_display_text));

      /* test substate changes */
      if(t_now - time_detection > 1000)
      {
        release_sensor_detection();
        substate = WAIT_FOR_DETECTION;
      }
    
      break;
      
    case WAIT_FOR_DETECTION:
      /* substate actions */
      t_now = t_now_ms;
      sprintf(text[0].text, "%.3f", (t_now - time_init)/1000.0);
      sprintf(text[1].text, "Laps to go: %u", laps_to_go);
      display_set_data(text, sizeof(text)/sizeof(s_display_text));

      /* test substate changes */
      if(sensor_interrupt_flag)
        substate = SHOW_TIME_DETECTION;
      break;
      
    case SHOW_TIME_DETECTION:
      /* substate actions */
      laps_to_go--;

      sprintf(text[0].text, "%.3f", (time_detection - time_last_detection)/1000.0);
      sprintf(text[1].text, "Laps to go: %u", laps_to_go);
      display_set_data(text, sizeof(text)/sizeof(s_display_text));

      if((time_detection - time_last_detection) < best_time_lap_ms)
      {
        best_time_lap_ms = time_detection - time_last_detection;
        set_buzzer_mode(DOUBLE_BEEP);
      }
      else
        set_buzzer_mode(SIMPLE_BEEP);

      if(laps_to_go == 0)
      {
        if(time_detection - time_init < best_time_total_ms)
        {
          best_time_total_ms = time_detection - time_init;
          set_buzzer_mode(TRIPLE_BEEP);
        }
        
        set_buzzer_mode(LARGE_BEEP);
      }

      time_last_detection = time_detection;

      /* test substate changes */
      substate = DETECTED_AND_SENSING_LOCKED;
      break;
      
    case DETECTED_AND_SENSING_LOCKED:
      /* substate actions */

      /* test substate changes */
      if(t_now_ms - time_detection > 1000)
      {
        if(laps_to_go == 0)
          substate = SHOW_FINAL_TIME;
        else
        {
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
      else if(t_now_ms - time_last_detection > 2000)
        substate = WAIT_FOR_DETECTION;
      break;
      
    case SHOW_FINAL_TIME:
      /* substate actions */
      sprintf(text[0].text, "%.3f", (time_detection - time_init)/1000.0);
      sprintf(text[1].text, "Press B to restart");
      display_set_data(text, sizeof(text)/sizeof(s_display_text));

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
 * total time spent on the circuit
 */
void start_stop_mode(void)
{
  //todo: program start/stop operating mode

  static s_display_text text[] = {
    /* text                 , pos_X , pos_Y , font      , aligment */
    {  "Mode not ready yet" , 0     , 0     , MENU_FONT , ALIGN_LEFT },
    {  "Press C to exit"    , 0     , 15    , MENU_FONT , ALIGN_LEFT }
  };

  switch(substate)
  {
    case INIT_SUBSTATE:
      /* substate actions */
      display_set_data(text, sizeof(text)/sizeof(s_display_text));
      
      /* test substate changes */
      substate = FINISHED;
      break;
      
    case FINISHED:
      /* substate actions */

      /* test substate changes */
      break;
      
    default:
      break;
  }
}

/**********************************************************************
 * @brief Executes the toast time measurement mode and shows the total
 * time spent on toasting the slice
 */
void toast_mode(void)
{
  //todo: program toast operating mode

  static s_display_text text[] = {
    /* text                 , pos_X , pos_Y , font      , aligment */
    {  "Mode not ready yet" , 0     , 0     , MENU_FONT , ALIGN_LEFT },
    {  "Press C to exit"    , 0     , 15    , MENU_FONT , ALIGN_LEFT }
  };

  switch(substate)
  {
    case INIT_SUBSTATE:
      /* substate actions */
      display_set_data(text, sizeof(text)/sizeof(s_display_text));
      
      /* test substate changes */
      substate = FINISHED;
      break;
      
    case FINISHED:
      /* substate actions */

      /* test substate changes */
      break;
      
    default:
      break;
  }
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

    case TOAST_MODE:
      toast_mode();
      break;

    default:
      break;
  }
}

/**********************************************************************
 * @brief Set the status of the led according to the supply diagnostics
 */
void set_led_from_diags(void)
{
  switch(batt_charger_diag)
  {
    case TEMP_OR_TIMER_FAULT: // 0 - Temperature fault or timer fault
      set_rgb_led_blink_mode(RGB_RED, MAX_BRIGHTNESS, 200, 100);
      break;
      
    case PRECONDITIONING:     // 2 - Preconditioning, constant current or constant voltage
      set_rgb_led_blink_mode(RGB_YELLOW, MAX_BRIGHTNESS, 2000, 100);
      break;
      
    case LOW_BATTERY_OUTPUT:  // 3 - Low battery output
      set_rgb_led_blink_mode(RGB_RED, MAX_BRIGHTNESS, 2000, 100);
      break;
      
    case CHARGE_COMPLETE:     // 4 - Charge complete
      set_rgb_led_blink_mode(RGB_GREEN, MAX_BRIGHTNESS, 2000, 100);
      break;
      
    case NO_BATTERY:          // 6 - Shutdown (VDD = VIN) or no battery present
      set_rgb_led_on_mode(RGB_BLUE, MAX_BRIGHTNESS);
      break;
      
    case NO_INPUT_POWER:      // 7 - Shutdown (VDD = VBAT) or no input power present
      set_rgb_led_blink_mode(RGB_CYAN, MAX_BRIGHTNESS, 2000, 100);
      break;
      
    default:
      set_rgb_led_off_mode();
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
  static uint8_t menu_screen_num = 0;

  /* Main state machine control */
  switch(state)
  {
    case INIT_STATE:
      /* state actions */
      show_main_menu(menu_screen_num);

      /* test state changes */
      state = MAIN_MENU;
      break;
      
    case MAIN_MENU:
      /* state actions */

      /* test state changes */
      if(get_button_state(BUTTON_A))
      {
        t2t_mode = mode_data[menu_screen_num*2].time_mode;
        state = mode_data[menu_screen_num*2].next_menu_state;
        substate = mode_data[menu_screen_num*2].first_subtate;
      }
      else if(get_button_state(BUTTON_B))
      {
        if((menu_screen_num*2 + 1) < sizeof(mode_data)/sizeof(s_mode_data))
        {
          t2t_mode = mode_data[menu_screen_num*2 + 1].time_mode;
          state = mode_data[menu_screen_num*2 + 1].next_menu_state;
          substate = mode_data[menu_screen_num*2 + 1].first_subtate;
        }
      }
      else if(get_button_state(BUTTON_C))
      {
        if(2 < sizeof(mode_data)/sizeof(s_mode_data))
        {
          menu_screen_num++;
          if(menu_screen_num >= (uint8_t)(ceil(((float)sizeof(mode_data)/sizeof(s_mode_data))/2)))
            menu_screen_num = 0;
          show_main_menu(menu_screen_num);
        }
      }
      break;

    case SET_NUM_LAPS:
      /* state actions */

      /* test state changes */
      if(x_laps_time_mode_laps_selection())
      {
        state = GET_TIME;
        substate = INIT_SUBSTATE;
      }
      break;

    case GET_TIME:
      /* state actions */
      get_time(t2t_mode);

      /* test state changes */
      if(get_button_state(BUTTON_C))
        state = INIT_STATE;
      break;
      
    default:
      break;
  }

  /* General function for the RGB led control */
  set_led_from_diags();
}
