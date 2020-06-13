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

#include "config/PINSEL.h"
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
  batt_monitor_init();
  buttons_init();
  buzzer_init();
  
  g_display.init();
  g_display.flipScreenVertically();
  g_display.setFont(Dialog_plain_11);
  g_display.setTextAlignment(TEXT_ALIGN_LEFT);

  g_display.clear();
  g_display.drawString(0, 0, "Select mode:");
  g_display.drawString(0, 15, "A: Normal lap time");
  g_display.drawString(0, 30, "B: 3 laps time");
  g_display.drawString(0, 45, "C: Star/Stop (2x t2t)");
  g_display.display();

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
      g_display.flipScreenVertically();
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

  if(millis() - batt_count > 500)
  {
    batt_count = millis();
    set_rgb_red(g_batt_voltage < BATT_VOLT_ALARM ? LED_ON : LED_OFF);
  }

}

void drawTime(unsigned long TimeDisp) {
  g_display.clear();
  switch(measureMode)
  {
    case NORMAL_LAP_TIME:
      g_display.setFont(Crushed_Regular_50);
      g_display.drawString(0, 0, String(TimeDisp/1000.0, 3));
      g_display.setFont(Dialog_plain_11);
      g_display.drawString(30, 50, "Last lap: " + String(last_lap_time/1000.0, 3));
      break;
      
    case X_LAPS_TIME:
      g_display.setFont(Crushed_Regular_50);
      g_display.drawString(0, 0, String(TimeDisp/1000.0, 3));
      g_display.setFont(Dialog_plain_11);
      if(laps < target_laps)
        g_display.drawString(30, 50, "Laps to go: " + String(target_laps - laps));
      else
        g_display.drawString(10, 50, "Press C to restart: ");
      break;
      
    case START_STOP:
      start_stop_method();
      break;
  }
  g_display.display();
}

void normal_lap_time_method()
{
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
      drawTime(time_lap);
  
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
    drawTime(0);
  else if(update_time == true)
    drawTime(millis() - time_count);
}

void x_laps_time_method()
{
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
      drawTime(time_lap);
  
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
    drawTime(time_sum);

  if(timeInit == false)
    drawTime(0);
  else if(update_time == true)
    drawTime(millis() - time_count + time_sum);

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
  //todo: shut down the display
  //todo: disable sensor interrupt
  power_12v_enable(POWER_12V_OFF);
}

void wake_up(void)
{
  power_12v_enable(POWER_12V_ON);
  //todo: disable sensor interrupt
  //todo: turn on the display
  //todo: shut down red led
}
