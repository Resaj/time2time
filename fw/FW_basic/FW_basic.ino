/*****************************************************
 * Author: Rubén Espino San José
 * Puma Pride Robotics Team
 * 
 * This program is used as a chronometer to measure lap
 * times in line-following robots competitions.
 ****************************************************/

#include "PINSEL.h"
#include "SSD1306Wire.h"
#include "Crushed_Regular_65.h"

#define DISPLAY_ADDRESS 0x3C
SSD1306Wire  display(DISPLAY_ADDRESS, I2C_SDA_DISP, I2C_SCL_DISP);
#define TIMER_DISPLAY_PERIOD 1000 // us

float tShow = 0;
bool get_time = false;
bool update_time = true;
unsigned int best_time = 10000;
unsigned long time_count = 0;

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
bool enable_buzzer = false;
bool enable_double_beep = false;
unsigned long buzzer_count = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
bool sensor_interrupt_flag = false;

void IRAM_ATTR Sensor_isr()
{
  portENTER_CRITICAL_ISR(&mux);
  sensor_interrupt_flag = true;
  portEXIT_CRITICAL_ISR(&mux);
}

void setup() {
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  digitalWrite(LED_R, HIGH);
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_B, HIGH);

  ledcSetup(buzzer_pwm_channel,buzzer_pwm_freq,buzzer_pwm_resolution);
  ledcAttachPin(BUZZER_PWM, buzzer_pwm_channel);
  
  pinMode(SLEEP_12V, OUTPUT);
  digitalWrite(SLEEP_12V, HIGH);
  pinMode(SENSOR, INPUT);
  attachInterrupt(digitalPinToInterrupt(SENSOR), Sensor_isr, FALLING);
  
  batt = analogRead(BATT_MONITOR)*4.051/3679;
  
  display.init();
  display.flipScreenVertically();
  display.setFont(Crushed_Regular_65);
  display.setTextAlignment(TEXT_ALIGN_LEFT);

  bool get_time = true;
}

void loop() {
  if(sensor_interrupt_flag == true)
  {
    portENTER_CRITICAL(&mux);
    sensor_interrupt_flag = false;
    portEXIT_CRITICAL(&mux);
      
    if(get_time == true)
    {
      unsigned long time_lap = 0;
      
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
  
  if(millis() - batt_count > 500)
  {
    batt_count = millis();
    batt = batt*4/5 + (analogRead(BATT_MONITOR)*4.051/3679)/5;
    digitalWrite(LED_R, batt < BATT_VOLT_ALARM ? LOW : HIGH);
  }

  if(enable_buzzer == true && millis() - buzzer_count > 50)
  {
    enable_buzzer = false;
    ledcWrite(buzzer_pwm_channel, 0);

    if(enable_double_beep == true)
      buzzer_count = millis();
  }

  if(enable_double_beep == true && enable_buzzer == false && millis() - buzzer_count > 90)
  {
    enable_double_beep = false;
    enable_buzzer = true;
    ledcWrite(buzzer_pwm_channel, 125);
    buzzer_count = millis();
  }
  
  if(update_time == false && millis() - time_count >= 2000)
	  update_time = true;
  if (get_time == false && millis() - time_count >= 1000)
	  get_time = true;
  
  if(update_time == true)
    drawTime(millis() - time_count);
}

void drawTime(unsigned long TimeDisp) {
  display.clear();
  display.drawString(0, 0, String(TimeDisp/1000.0, 3));
  display.display();
}
