/**********************************************************************
 * Project: Time2time
 * 
 * File description: This code file contains the parameters and 
 * functions for for the ESP-Now communication between ESP32 modules
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

#include <esp_now.h>
#include <WiFi.h>
#include "config/PINSEL.h"
#include "espnow_comm.h"

/**********************************************************************
 * Structs
 *********************************************************************/

typedef struct {
  uint8_t nodeAddr;     // Node address (0-7). Defined with the jumpers in the PCB
  uint8_t MACAddr[6];   // MAC address of the ESP node
  bool    linked;       // Flag which indicates the link status
  uint8_t circuit_pos;  // Position of the node in the circuit. It has to be calculated automaticaly at the begining of the race
} s_t2t_node;

/**********************************************************************
 * Local configuration parameters
 *********************************************************************/

/* Up to 8 nodes linked, positioned in the next matrix according to the JP_ADD jumpers
 * Define the MAC address of each module. You can read it following this tutorial:
 * https://randomnerdtutorials.com/get-change-esp32-esp8266-mac-address-arduino/
 */
s_t2t_node t2t_node[] = {
  /* nodeAddr, MACAddr                             , linked, circuit_pos */
  {  255     , {0x24, 0x0A, 0xC4, 0x2B, 0x44, 0x2C}, false , 255         },
  {  255     , {0x24, 0x0A, 0xC4, 0x2A, 0x4C, 0x48}, false , 255         },
  {  255     , {0x24, 0x0A, 0xC4, 0x2B, 0x44, 0xCC}, false , 255         },
  {  255     , {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, false , 255         },
  {  255     , {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, false , 255         },
  {  255     , {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, false , 255         },
  {  255     , {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, false , 255         },
  {  255     , {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, false , 255         }
};

uint8_t broadcastMacAddr[] = {0x24, 0x0A, 0xC4, 0x2B, 0x44, 0x2C};

/**********************************************************************
 * Defines
 *********************************************************************/


/**********************************************************************
 * Global variables
 *********************************************************************/

uint8_t moduleNumber = 0;
extern s_espnow_msg incoming_msg = { 0, 0 };
uint8_t espnow_error = 0;
uint8_t espnow_add_peer_error = 0;

/**********************************************************************
 * Local functions
 *********************************************************************/

void config_node(void)
{
  pinMode(PIN_JP_ADD1, INPUT);
  pinMode(PIN_JP_ADD2, INPUT);
  pinMode(PIN_JP_ADD3, INPUT);
}

uint8_t read_moduleNumber(void)
{
  uint8_t add_bit1 = digitalRead(PIN_JP_ADD1);
  uint8_t add_bit2 = digitalRead(PIN_JP_ADD2);
  uint8_t add_bit3 = digitalRead(PIN_JP_ADD3);
  return (add_bit3<<2) + (add_bit2<<1) + (add_bit1);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t state) {
//  if (state == 0){
//    success = "Delivery Success :)";
//  }
//  else{
//    success = "Delivery Fail :(";
//  }
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int32_t len)
{
  memcpy(&incoming_msg, incomingData, sizeof(incoming_msg));
}

/**********************************************************************
 * Global functions
 *********************************************************************/

void espnow_comm_init(void)
{
  //uint8_t espnow_error = 0;
  
  config_node();
  moduleNumber = read_moduleNumber();

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK)
    espnow_error = 1;

  esp_now_register_send_cb(OnDataSent);
  
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastMacAddr, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  //while(esp_now_add_peer(&peerInfo) != ESP_OK);
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
    espnow_add_peer_error = 1;

  esp_now_register_recv_cb(OnDataRecv);

//Enlaces a nodos concretos en otra función que se llame al entrar en el modo correspondiente
  //todo: read the MAC address to get it automaticaly and save it in the struct
  //todo: send the MAC address and the moduleNumber to the broadcast address
  //todo: when receiving a message of a node: register the node and send the own MAC address to stablish the communication

/*  for(uint8_t node = 0; node<sizeof(t2t_node)/sizeof(t2t_node[0]); node++)
  {
    uint16_t sum = 0;
    for(uint8_t i=0; i<sizeof(t2t_node[0].MACAddr)/sizeof(t2t_node[0].MACAddr[0]); i++)
    {
      sum += t2t_node[node].MACAddr[i];
    }
    
    if(node != moduleNumber && sum != 0)
    {
      for(uint8_t i=0; i<sizeof(t2t_node[0].MACAddr)/sizeof(t2t_node[0].MACAddr[0]); i++)
      {
        esp_now_peer_info_t peerInfo;
        memcpy(peerInfo.peer_addr, broadcastAddress, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        
//      if (esp_now_add_peer(&peerInfo) != ESP_OK)
//      {
//        //Indicar que no se ha enlazado
//      }
      }
    }
  }
  
//  esp_now_register_recv_cb(OnDataRecv);*/
}

void sendESPNowData(s_espnow_msg msg)
{
  /*esp_err_t result = */(void)esp_now_send(broadcastMacAddr, (uint8_t *) &msg, sizeof(msg));
}
