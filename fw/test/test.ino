/*****************************************************
 * Author: Rubén Espino San José
 * Puma Pride Robotics Team
 * 
 * Test program for time2time project.
 * The test is guided all the time with the display.
 ****************************************************/

#include "SSD1306Wire.h"
#include "Dialog_plain_Font.h"
#include "PINSEL.h"

#define DISPLAY_ADDRESS 0x3C
SSD1306Wire display(DISPLAY_ADDRESS, I2C_SDA_DISP, I2C_SCL_DISP);

int state = 0;
String msg;

bool button1 = 0;
bool button2 = 0;
bool button3 = 0;
bool sensor = 0, sensor_ant = 0;
int detection_counter = 0;
bool power_good = 0, stat1 = 0, stat2 = 0;

void setup() {
  pinMode(JP_ADD1, INPUT);
  pinMode(JP_ADD2, INPUT);
  pinMode(JP_ADD3, INPUT);
  
  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);
  pinMode(BUTTON3, INPUT);

  pinMode(POWER_GOOD, INPUT);
  pinMode(STAT1, INPUT);
  pinMode(STAT2, INPUT);

  pinMode(SENSOR, INPUT);
  pinMode(SLEEP_12V, OUTPUT);
  digitalWrite(SLEEP_12V, LOW);

  pinMode(BUZZER_PWM, OUTPUT);
  
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  digitalWrite(LED_R, HIGH);
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_B, HIGH);

  display.init();
  display.flipScreenVertically();
  display.setFont(Dialog_plain_11);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
}

void loop() {
  display.clear();

  button1 = !digitalRead(BUTTON1);
  button2 = !digitalRead(BUTTON2);
  button3 = !digitalRead(BUTTON3);

  switch(state)
  {
    case 0:
      display.drawString(0, 15, "time2time test\nPress C to continue" );
      if(button3)
        state++;
      break;

    case 1:
      msg =  String(analogRead(BATT_MONITOR)*4.051/3679, 3);
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
        beep(BUZZER_PWM, 1000, 300);
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
        digitalWrite(LED_B, HIGH);
        digitalWrite(LED_R, LOW);
        delay(500);
        digitalWrite(LED_R, HIGH);
        digitalWrite(LED_G, LOW);
        delay(500);
        digitalWrite(LED_G, HIGH);
        digitalWrite(LED_B, LOW);
        delay(500);
      }
      display.drawString(0, 10, "Press B to turn on\nthe led\nPress C to continue");
      if(button3)
      {
        digitalWrite(LED_R, HIGH);
        digitalWrite(LED_G, HIGH);
        digitalWrite(LED_B, HIGH);
        state++;
      }
      break;

    case 6:
      msg =  String(digitalRead(JP_ADD1)*4 + digitalRead(JP_ADD2)*2 + digitalRead(JP_ADD3));
      display.drawString(0, 15, "Node address: " + msg + "\nPress B to continue");
      if(button2)
        state++;
      break;

    case 7:
      display.drawString(0, 15, "Press A to turn on\nthe distance sensor");
      if(button1)
      {
        digitalWrite(SLEEP_12V, HIGH);
        state++;
      }
      break;

    case 8:
      sensor = !digitalRead(SENSOR);
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
        digitalWrite(SLEEP_12V, LOW);
        state++;
      }
      break;

    case 10:
      power_good = digitalRead(POWER_GOOD);
      stat1 = digitalRead(STAT1);
      stat2 = digitalRead(STAT2);
      display.drawString(0, 0, "PG: "
      + String(power_good) + " S1: " + String(stat1) + " S2: " + String(stat2)
      + "\nConnect battery,\nswitch on and\ndisconnect the USB");
      if(power_good && stat1 && stat2)
        state++;
      break;

    case 11:
      power_good = digitalRead(POWER_GOOD);
      stat1 = digitalRead(STAT1);
      stat2 = digitalRead(STAT2);
      display.drawString(0, 0, "PG: "
      + String(power_good) + " S1: " + String(stat1) + " S2: " + String(stat2)
      + "\nConnect the USB");
      if(!power_good && !stat1 && stat2)
        state++;
      break;

    case 12:
      power_good = digitalRead(POWER_GOOD);
      stat1 = digitalRead(STAT1);
      stat2 = digitalRead(STAT2);
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
