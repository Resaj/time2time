/*****************************************************
 * Project: Time2time
 * 
 * File description: Test program for the ESPnow
 * communication between two #time2time nodes.
 * Each node send to the other one its own detection
 * sensor state. The other one print the state on its
 * display.
 * 
 * IMPORTANT NOTE: Before running this ESPnow test,
 * run the test program to get the MAC addresses of
 * run #time2time nodes and modify MACaddr[].
 * 
 * Author: Rubén Espino San José
 * Puma Pride Robotics Team
 * 
 * License: Attribution-NonCommercial-ShareAlike 4.0
 * International (CC BY-NC-SA 4.0) 
 ****************************************************/

#include <esp_now.h>
#include <WiFi.h>
#include "SSD1306Wire.h"
#include "config/PINSEL.h"
#include "fonts/Crushed_Regular_50.h"

#define DISPLAY_ADDRESS 0x3C
SSD1306Wire g_display(DISPLAY_ADDRESS, PIN_I2C_SDA_DISP, PIN_I2C_SCL_DISP);

// Put the MAC address of the other #time2time node here
uint8_t MACaddr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

typedef struct {
    char sensor_state;
} s_msg;

s_msg s_msg2send;
s_msg s_msgReceived;
char remoteSensorState;

void OnDataSent(const uint8_t *MAC_Addr, esp_now_send_status_t status) {
}

void OnDataRecv(const uint8_t *MAC_Addr, const uint8_t *receivedData, int len) {
  memcpy(&s_msgReceived, receivedData, sizeof(s_msgReceived));
  remoteSensorState = s_msgReceived.sensor_state;
}
 
void setup() {
  pinMode(PIN_SLEEP_12V, OUTPUT);
  digitalWrite(PIN_SLEEP_12V, HIGH);
  pinMode(PIN_SENSOR, INPUT);

  g_display.init();
  g_display.flipScreenVertically();
  g_display.setFont(Crushed_Regular_50);
  g_display.setTextAlignment(TEXT_ALIGN_LEFT);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK)
    return;

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, MACaddr, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
    return;
}
 
void loop() {
  static int sensor_state = -1;
  s_msg2send.sensor_state = digitalRead(27);

  if(s_msg2send.sensor_state != sensor_state)
  {
    esp_err_t result = esp_now_send(MACaddr, (uint8_t *)&s_msg2send, sizeof(s_msg2send));
    s_msg2send.sensor_state = sensor_state;
  }

  updateDisplay();
}

void updateDisplay(){
  char text[25];

  g_display.clear();
  if(remoteSensorState == 0)
  {
    g_display.invertDisplay();
    sprintf(text, ":-)");
  }
  else
  {
    g_display.normalDisplay();
    sprintf(text, ":-(");
  }
  
  g_display.drawString(50, 5, text);
  g_display.display();
}
