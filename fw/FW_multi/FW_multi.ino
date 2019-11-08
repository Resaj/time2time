/*****************************************************
 * Author: Rubén Espino San José
 * Puma Pride Robotics Team
 * 
 * This program is used as a chronometer to measure lap
 * times in line-following robots competitions.
 ****************************************************/

#include "PINSEL.h"
#include "SSD1306Wire.h"
#include "Crushed_Regular_50.h"
#include "Dialog_plain_11.h"

#define DISPLAY_ADDRESS 0x3C
SSD1306Wire  display(DISPLAY_ADDRESS, I2C_SDA_DISP, I2C_SCL_DISP);
#define TIMER_DISPLAY_PERIOD 1000 // us

bool button_A, button_B, button_C;


float batt = 4.2;
unsigned long batt_count = 0;
#define MAX_ADC_VALUE     4095	// ADC decimal value
#define MAX_ADC_VOLT      3.3		// Volts
#define BATT_MONITOR_R1   47	  // kOhmios
#define BATT_MONITOR_R2   120	  // kOhmios
#define BATT_VOLT_ALARM   3.5	  // Volts

int buzzer_pwm_freq = 2000;
int buzzer_pwm_channel = 0;
int buzzer_pwm_resolution = 8;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
bool sensor_interrupt_flag = false;

void IRAM_ATTR Sensor_isr()
{
  portENTER_CRITICAL_ISR(&mux);
  sensor_interrupt_flag = true;
  portEXIT_CRITICAL_ISR(&mux);
}

char measureMode = 0;
#define NORMAL_LAP_TIME   0
#define X_LAPS_TIME       1
#define START_STOP        2


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

bool enable_buzzer = false;
bool enable_double_beep = false;
unsigned long buzzer_count = 0;


void setup() {
  pinMode(BUTTON_A, INPUT);
  pinMode(BUTTON_B, INPUT);
  pinMode(BUTTON_C, INPUT);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  digitalWrite(LED_R, HIGH);
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_B, HIGH);

  ledcSetup(buzzer_pwm_channel,buzzer_pwm_freq,buzzer_pwm_resolution);
  ledcAttachPin(BUZZER_PWM, buzzer_pwm_channel);
  
  display.init();
  display.flipScreenVertically();
  display.setFont(Dialog_plain_11);
  display.setTextAlignment(TEXT_ALIGN_LEFT);

  display.clear();
  display.drawString(0, 0, "Select mode:");
  display.drawString(0, 15, "A: Normal lap time");
  display.drawString(0, 30, "B: 3 laps time");
  display.drawString(0, 45, "C: Star/Stop (2x t2t)");
  display.display();

  while(1)
  {
    readButtons();

    if(!button_A)
    {
      measureMode = NORMAL_LAP_TIME;
      break;
    }
    if(!button_B)
    {
      measureMode = X_LAPS_TIME;
      break;
    }
    if(!button_C)
    {
      measureMode = START_STOP;
      break;
    }
  }

  pinMode(SLEEP_12V, OUTPUT);
  digitalWrite(SLEEP_12V, HIGH);
  pinMode(SENSOR, INPUT);
  attachInterrupt(digitalPinToInterrupt(SENSOR), Sensor_isr, FALLING);
  
  batt = analogRead(BATT_MONITOR)*4.051/3679;
  
  get_time = true;
}

void loop() {
  readButtons();
  
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

  if(enable_buzzer == true && millis() - buzzer_count > 50)
  {
    enable_buzzer = false;
    ledcWrite(buzzer_pwm_channel, 0);

    if(enable_double_beep == true)
      buzzer_count = millis();
  }

  buzzer_function();
}

void buzzer_function(void)
{
  if(enable_double_beep == true && enable_buzzer == false && millis() - buzzer_count > 90)
  {
    enable_double_beep = false;
    enable_buzzer = true;
    ledcWrite(buzzer_pwm_channel, 125);
    buzzer_count = millis();
  }
  
  if(millis() - batt_count > 500)
  {
    batt_count = millis();
    batt = batt*4/5 + (analogRead(BATT_MONITOR)*4.051/3679)/5;
    digitalWrite(LED_R, batt < BATT_VOLT_ALARM ? LOW : HIGH);
  }
}

void readButtons(void)
{
  button_A = digitalRead(BUTTON_A);
  button_B = digitalRead(BUTTON_B);
  button_C = digitalRead(BUTTON_C);
}

void drawTime(unsigned long TimeDisp) {
  display.clear();
  switch(measureMode)
  {
    case NORMAL_LAP_TIME:
      display.setFont(Crushed_Regular_50);
      display.drawString(0, 0, String(TimeDisp/1000.0, 3));
      display.setFont(Dialog_plain_11);
      display.drawString(30, 50, "Last lap: " + String(last_lap_time/1000.0, 3));
      break;
      
    case X_LAPS_TIME:
      display.setFont(Crushed_Regular_50);
      display.drawString(0, 0, String(TimeDisp/1000.0, 3));
      display.setFont(Dialog_plain_11);
      if(laps < target_laps)
        display.drawString(30, 50, "Laps to go: " + String(target_laps - laps));
      else
        display.drawString(10, 50, "Press C to restart: ");
      break;
      
    case START_STOP:
      start_stop_method();
      break;
  }
  display.display();
}

void normal_lap_time_method()
{
  if(sensor_interrupt_flag == true)
  {
    portENTER_CRITICAL(&mux);
    sensor_interrupt_flag = false;
    portEXIT_CRITICAL(&mux);

    if(timeInit == false)
    {
      timeInit = true;
      time_count = millis();

      enable_buzzer = true;
      ledcWrite(buzzer_pwm_channel, 125);
      buzzer_count = millis();
      get_time = false;
    }
    
    if(get_time == true)
    {
      time_lap = millis() - time_count;
      time_count = millis();
      if(time_lap < best_time)
      {
        best_time = time_lap;
        enable_double_beep = true;
      }
      enable_buzzer = true;
      ledcWrite(buzzer_pwm_channel, 125);
      buzzer_count = millis();
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
  if (get_time == false && millis() - time_count >= 1000)
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
    portENTER_CRITICAL(&mux);
    sensor_interrupt_flag = false;
    portEXIT_CRITICAL(&mux);

    if(timeInit == false)
    {
      timeInit = true;
      time_count = millis();

      enable_buzzer = true;
      ledcWrite(buzzer_pwm_channel, 125);
      buzzer_count = millis();
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
        enable_double_beep = true;
      }
      enable_buzzer = true;
      ledcWrite(buzzer_pwm_channel, 125);
      buzzer_count = millis();
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
  if (get_time == false && millis() - time_count >= 1000 && laps < target_laps)
    get_time = true;
  if(laps == target_laps && millis() - time_count >= 1000)
    drawTime(time_sum);

  if(timeInit == false)
    drawTime(0);
  else if(update_time == true)
    drawTime(millis() - time_count + time_sum);

  if(!button_C)
  {
    timeInit = false;
    update_time = true;
    get_time = true;
    time_sum = 0;
    laps = 0;
    
    portENTER_CRITICAL(&mux);
    sensor_interrupt_flag = false;
    portEXIT_CRITICAL(&mux);
  }
}

void start_stop_method()
{
  digitalWrite(LED_B, LOW);
}
