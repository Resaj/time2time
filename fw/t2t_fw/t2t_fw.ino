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

#include "buttons.h"
#include "buzzer.h"
#include "display.h"
#include "led_rgb.h"
#include "pass_sensor.h"
#include "supply.h"

char measureMode = 0;
#define NORMAL_LAP_TIME   0
#define X_LAPS_TIME       1
#define START_STOP        2

unsigned long batt_count = 0; //todo: change the counter and create define in supply module to set the battery monitor period. Manage with timer and scheduler

bool timeInit = false;
float tShow = 0;
bool get_time = false;
bool update_time = true;
unsigned int best_time = 10000;
unsigned long time_count = 0;
unsigned long time_lap = 0;
float last_lap_time = 0;
unsigned long time_sum = 0;
unsigned int target_laps = 3;
unsigned int laps = 0;

void setup() {
  s_display_text text[4] = {
    /* Text                   , pos_X , pos_Y , font      , aligment */
    { "Select mode:"          , 0     , 0     , MENU_FONT , ALIGN_LEFT },
    { "A: Normal lap time"    , 0     , 15    , MENU_FONT , ALIGN_LEFT },
    { "B: 3 laps time"        , 0     , 30    , MENU_FONT , ALIGN_LEFT },
    { "C: Star/Stop (2x t2t)" , 0     , 45    , MENU_FONT , ALIGN_LEFT }
  };

  batt_monitor_init();
  buttons_init();
  buzzer_init();
  display_init();

  display_set_data(text);

  while(1)
  {
    read_buttons();

    if(get_button_state(BUTTON_A))
    {
      measureMode = NORMAL_LAP_TIME;
      break;
    }
    if(get_button_state(BUTTON_B))
    {
      measureMode = X_LAPS_TIME;
      break;
    }
    if(get_button_state(BUTTON_C))
    {
      measureMode = START_STOP;
      break;
    }
  }

  pass_sensor_init();

  get_time = true;
}

void loop() {
// todo: scheduler with timer
// tasks:
//    read buttons
//    read sensor
//    read battery and charger
//    manage UART communication
//    manage Wifi communication
//    control buzzer
//    control leds
//    control display
//    control t2t mode

  read_buttons();
  
  switch(measureMode)
  {
    case NORMAL_LAP_TIME:
      normal_lap_time_method();
      break;
      
    case X_LAPS_TIME:
      x_laps_time_method();
      break;
      
    case START_STOP:
      start_stop_method();
      break;
  }

  buzzer_task();
  display_task();

  if(millis() - batt_count > 500)
  {
    batt_count = millis();
    set_rgb_red(g_batt_voltage < BATT_VOLT_ALARM ? LED_ON : LED_OFF);
  }
}

void normal_lap_time_method()
{
  static s_display_text text[2] = {
    /* Text , pos_X , pos_Y , font                , aligment */
    {  ""   , 0     , 0     , MAIN_TIME_FONT      , ALIGN_LEFT },
    {  ""   , 30    , 50    , SECONDARY_TIME_FONT , ALIGN_LEFT }
  };

  if(sensor_interrupt_flag == true)
  {
    release_sensor_detection();

    if(timeInit == false)
    {
      timeInit = true;
      time_count = millis();

      set_buzzer_mode(SIMPLE_BEEP);
      get_time = false;
    }
    
    if(get_time == true)
    {
      time_lap = millis() - time_count;
      time_count = millis();
      if(time_lap < best_time)
      {
        best_time = time_lap;
        set_buzzer_mode(DOUBLE_BEEP);
      }
      else
        set_buzzer_mode(SIMPLE_BEEP);

      get_time = false;
      update_time = false;
    }
  }
  
  if(update_time == false && millis() - time_count >= 2000 && timeInit == true)
  {
    update_time = true;
    last_lap_time = time_lap;
  }
  if(get_time == false && millis() - time_count >= 1000)
    get_time = true;

  if(timeInit == false)
  {
    sprintf(text[0].text, "0.000");
    sprintf(text[1].text, "");
    display_set_data(text);
  }
  else if(update_time == true)
  {
    sprintf(text[0].text, "%.3f", time_lap/1000.0);
    sprintf(text[1].text, "Last lap: %.3f", time_lap/1000.0);
    display_set_data(text);
  }
}

void x_laps_time_method(void)
{
  static s_display_text text[2] = {
    /* Text , pos_X , pos_Y , font                , aligment */
    {  ""   , 0     , 0     , MAIN_TIME_FONT      , ALIGN_LEFT },
    {  ""   , 30    , 50    , SECONDARY_TIME_FONT , ALIGN_LEFT }
  };

  if(sensor_interrupt_flag == true)
  {
    release_sensor_detection();

    if(timeInit == false)
    {
      timeInit = true;
      time_count = millis();

      set_buzzer_mode(SIMPLE_BEEP);
      get_time = false;
    }
    
    if(get_time == true && laps < target_laps)
    {
      laps++;

      time_lap = millis() - time_count;
      time_count = millis();
      time_sum += time_lap;
      if(time_lap < best_time)
      {
        best_time = time_lap;
        set_buzzer_mode(DOUBLE_BEEP);
      }
      else
        set_buzzer_mode(SIMPLE_BEEP);

      sprintf(text[0].text, "%.3f", time_lap/1000.0);
      sprintf(text[1].text, "Laps to go: %u", target_laps - laps);
      display_set_data(text);

      get_time = false;
      update_time = false;
    }
  }
  
  if(update_time == false && millis() - time_count >= 2000 && timeInit == true && laps < target_laps)
  {
    update_time = true;
    last_lap_time = time_lap;
  }
  if(get_time == false && millis() - time_count >= 1000 && laps < target_laps)
    get_time = true;
  if(laps == target_laps && millis() - time_count >= 1000)
  {
    sprintf(text[0].text, "%.3f", time_sum/1000.0);
    sprintf(text[1].text, "Press C to restart");
    display_set_data(text);
  }

  if(timeInit == false)
  {
    sprintf(text[0].text, "0.000");
    sprintf(text[1].text, "Laps to go: %u", target_laps);
    display_set_data(text);
  }
  else if(update_time == true)
  {
    sprintf(text[0].text, "%.3f", (millis() - time_count + time_sum)/1000.0);
    display_set_data(text);
  }

  if(get_button_state(BUTTON_C))
  {
    timeInit = false;
    update_time = true;
    get_time = true;
    time_sum = 0;
    laps = 0;

    release_sensor_detection();
  }
}

void start_stop_method()
{
  //todo: program start/stop operating mode
  set_rgb_blue(LED_ON);
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
