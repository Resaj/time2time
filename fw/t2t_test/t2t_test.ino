/*****************************************************
 * Project: Time2time
 * 
 * File description: General test program for
 * #time2time project. This program tests the
 * peripherals of the system, except the ESPnow
 * communication. It's guided all the time with the
 * display.
 * Use this program to get the MAC address of your
 * #time2time nodes.
 * 
 * Author: Rubén Espino San José
 * Puma Pride Robotics Team
 * 
 * License: Attribution-NonCommercial-ShareAlike 4.0
 * International (CC BY-NC-SA 4.0) 
 ****************************************************/

#include "SSD1306Wire.h"
#include "WiFi.h"
#include "config/PINSEL.h"
#include "fonts/Dialog_plain_Font.h"

#define DISPLAY_ADDRESS 0x3C
SSD1306Wire display(DISPLAY_ADDRESS, PIN_I2C_SDA_DISP, PIN_I2C_SCL_DISP);

int state = 0;
String msg;

bool button1 = 0;
bool button2 = 0;
bool button3 = 0;
bool sensor = 0, sensor_ant = 0;
int detection_counter = 0;
bool power_good = 0, stat1 = 0, stat2 = 0;

void setup() {
  pinMode(PIN_JP_ADD1, INPUT);
  pinMode(PIN_JP_ADD2, INPUT);
  pinMode(PIN_JP_ADD3, INPUT);
  
  pinMode(PIN_BUTTON_A, INPUT);
  pinMode(PIN_BUTTON_B, INPUT);
  pinMode(PIN_BUTTON_C, INPUT);

  pinMode(PIN_POWER_GOOD, INPUT);
  pinMode(PIN_STAT1, INPUT);
  pinMode(PIN_STAT2, INPUT);

  pinMode(PIN_SENSOR, INPUT);
  pinMode(PIN_SLEEP_12V, OUTPUT);
  digitalWrite(PIN_SLEEP_12V, LOW);

  pinMode(PIN_BUZZER_PWM, OUTPUT);
  
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  digitalWrite(PIN_LED_R, HIGH);
  digitalWrite(PIN_LED_G, HIGH);
  digitalWrite(PIN_LED_B, HIGH);

  WiFi.mode(WIFI_MODE_STA);

  display.init();
  display.flipScreenVertically();
  display.setFont(Dialog_plain_11);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
}

void loop() {
  display.clear();

  button1 = !digitalRead(PIN_BUTTON_A);
  button2 = !digitalRead(PIN_BUTTON_B);
  button3 = !digitalRead(PIN_BUTTON_C);

  switch(state)
  {
    case 0:
      display.drawString(0, 15, "#time2time test\nPress C to continue" );
      if(button3)
        state++;
      break;

    case 1:
      msg =  String(analogRead(PIN_BATT_MONITOR)*4.051/3679, 3);
      display.drawString(0, 15, "Battery voltage: " + msg + "\nPress B to continue");
      if(button2)
        state++;
      break;

    case 2:
      display.drawString(0, 10, "Press A to turn on\nthe buzzer");
      if(button1)
        state++;
      break;

    case 3:
      if(button1)
        beep(PIN_BUZZER_PWM, 1000, 300);
      display.drawString(0, 10, "Press A to turn on\nthe buzzer\nPress C to continue");
      if(button3)
        state++;
      break;

    case 4:
      display.drawString(0, 10, "Press B to turn on\nthe led");
      if(button2)
        state++;
      break;

    case 5:
      if(button2)
      {
        digitalWrite(PIN_LED_B, HIGH);
        digitalWrite(PIN_LED_R, LOW);
        delay(500);
        digitalWrite(PIN_LED_R, HIGH);
        digitalWrite(PIN_LED_G, LOW);
        delay(500);
        digitalWrite(PIN_LED_G, HIGH);
        digitalWrite(PIN_LED_B, LOW);
        delay(500);
      }
      display.drawString(0, 10, "Press B to turn on\nthe led\nPress C to continue");
      if(button3)
      {
        digitalWrite(PIN_LED_R, HIGH);
        digitalWrite(PIN_LED_G, HIGH);
        digitalWrite(PIN_LED_B, HIGH);
        state++;
      }
      break;

    case 6:
      msg =  String(digitalRead(PIN_JP_ADD1)*4 + digitalRead(PIN_JP_ADD2)*2 + digitalRead(PIN_JP_ADD3));
      display.drawString(0, 3, "Node address: " + msg + "\nMAC address:\n " + WiFi.macAddress() + "\nPress B to continue");
      if(button2)
        state++;
      break;

    case 7:
      display.drawString(0, 15, "Press A to turn on\nthe distance sensor");
      if(button1)
      {
        digitalWrite(PIN_SLEEP_12V, HIGH);
        state++;
      }
      break;

    case 8:
      sensor = !digitalRead(PIN_SENSOR);
      if(!sensor && sensor != sensor_ant)
        detection_counter++;
      sensor_ant = sensor;
      display.drawString(0, 0, "Change the sensor\ndetection repeatedly\nSensor: " + String(sensor) + "\nDetection counter: " + String(detection_counter));
      if(detection_counter >= 5)
        state++;
      break;

    case 9:
      display.drawString(0, 15, "Sensor state changes\nPress B to continue");
      if(button2)
      {
        digitalWrite(PIN_SLEEP_12V, LOW);
        state++;
      }
      break;

    case 10:
      power_good = digitalRead(PIN_POWER_GOOD);
      stat1 = digitalRead(PIN_STAT1);
      stat2 = digitalRead(PIN_STAT2);
      display.drawString(0, 0, "PG: "
      + String(power_good) + " S1: " + String(stat1) + " S2: " + String(stat2)
      + "\nConnect battery,\nswitch on and\ndisconnect the USB");
      if(power_good && stat1 && stat2)
        state++;
      break;

    case 11:
      power_good = digitalRead(PIN_POWER_GOOD);
      stat1 = digitalRead(PIN_STAT1);
      stat2 = digitalRead(PIN_STAT2);
      display.drawString(0, 0, "PG: "
      + String(power_good) + " S1: " + String(stat1) + " S2: " + String(stat2)
      + "\nConnect the USB");
      if(!power_good && !stat1 && stat2)
        state++;
      break;

    case 12:
      power_good = digitalRead(PIN_POWER_GOOD);
      stat1 = digitalRead(PIN_STAT1);
      stat2 = digitalRead(PIN_STAT2);
      display.drawString(0, 0, "PG: "
      + String(power_good) + " S1: " + String(stat1) + " S2: " + String(stat2)
      + "\nTurn off the switch");
      if(!power_good && stat1 && !stat2)
        state++;
      break;

    default:
      display.drawString(0, 15, "Test finished\nPress R to restart");
      break;
  }
  
  display.display();
  delay(50);
}

void beep (unsigned char speakerPin, int frequencyInHertz, long timeInMilliseconds)
{
  int x;
  long delayAmount = (long)(1000000/frequencyInHertz);
  long loopTime = (long)((timeInMilliseconds*1000)/(delayAmount*2));
  for (x=0;x<loopTime;x++)
  {
    digitalWrite(speakerPin,HIGH);
    delayMicroseconds(delayAmount);
    digitalWrite(speakerPin,LOW);
    delayMicroseconds(delayAmount);
  }
}
