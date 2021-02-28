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
#include "config/MyMACAddrList.h"
#include "espnow_comm.h"

/**********************************************************************
 * Structs
 *********************************************************************/

typedef struct s_t2t_node {
  uint8_t    *MACAddr;  // MAC address of the ESP node
  s_t2t_node *prevNode; // Previous node in the circuit. It is calculated automaticaly when the functionament mode is selected
  s_t2t_node *nextNode; // Next node in the circuit. It is calculated automaticaly when the functionament mode is selected
} s_t2t_node;

typedef struct {
  uint8_t type  : 3;
  uint8_t info  : 5;
} s_espnow_default_msg;

typedef struct {
  uint8_t type      : 3;
  uint8_t nodeAddr  : 3;  // Node address (0-7). Defined by the jumpers in the PCB
  uint8_t ask4Ack   : 1;  // Ask for the nodeAddr of the other node. 1 = yes; 0 = no
  uint8_t reserved  : 1;
} s_espnow_link_msg;

typedef struct {
  uint8_t type          : 3;
  uint8_t func_mode     : 3;
  uint8_t isRxNodeUsed  : 1;
  uint8_t reserved      : 1;
} s_espnow_mode_msg;

/**********************************************************************
 * Defines & enums
 *********************************************************************/

typedef enum {
  LINK_MSG,
  SYNC_MSG,
  MODE_MSG,
  DETECTION_MSG,
  TIME_MSG,
  LOWBATTERY_MSG
} msg_type;

/**********************************************************************
 * Local variables
 *********************************************************************/

s_t2t_node t2t_node[8];      // Matrix to store the information of the nodes
s_t2t_node *thisNode = NULL; // Pointer to the node position at t2t_node
s_t2t_node *mainNode = NULL; // Pointer to the main node position at t2t_node.
                             // Determines the node that manages the total times and the functionament mode
                             // It it set when selecting the functionament mode.
uint8_t thisNodeAddr;        // Node address (0-7). Defined by the jumpers in the PCB
s_espnow_default_msg s_receivedMsg;

/**********************************************************************
 * Local functions
 *********************************************************************/

uint8_t read_moduleNumber(void)
{
  uint8_t add_bit1 = digitalRead(PIN_JP_ADD1);
  uint8_t add_bit2 = digitalRead(PIN_JP_ADD2);
  uint8_t add_bit3 = digitalRead(PIN_JP_ADD3);
  return (add_bit3<<2) + (add_bit2<<1) + (add_bit1);
}

uint8_t config_node(void)
{
  bool isNodeListed = false;
  uint8_t MACAddr[6];
  
  pinMode(PIN_JP_ADD1, INPUT);
  pinMode(PIN_JP_ADD2, INPUT);
  pinMode(PIN_JP_ADD3, INPUT);

  thisNodeAddr = read_moduleNumber();
  
  WiFi.macAddress(MACAddr);
  for(uint8_t num=0; num<(sizeof(MyMACAddrList)/sizeof(MyMACAddrList[0])); num++)
  {
    bool isEqual = true;
    
    for(uint8_t pos=0; pos<(sizeof(MACAddr)/sizeof(uint8_t)); pos++)
    {
      if(MACAddr[pos] != MyMACAddrList[num][pos])
      {
        isEqual = false;
        break;
      }
    }
    
    if(isEqual)
    {
      t2t_node[thisNodeAddr].MACAddr = MyMACAddrList[num];
      isNodeListed = true;
      break;
    }
  }
  if(!isNodeListed)
    return false;

  t2t_node[thisNodeAddr].prevNode = NULL;
  t2t_node[thisNodeAddr].nextNode = NULL;
  
  thisNode = &t2t_node[thisNodeAddr];
  mainNode = &t2t_node[thisNodeAddr];

  return true;
}

void OnDataSent(const uint8_t *MAC_Addr, esp_now_send_status_t state) {
}

void OnDataRecv(const uint8_t *MAC_Addr, const uint8_t *receivedData, int32_t len)
{
  memcpy(&s_receivedMsg, receivedData, sizeof(s_receivedMsg));
}

/**********************************************************************
 * Global functions
 *********************************************************************/

void espnow_comm_init(void)
{
  if(!config_node())
    return; // Node not found in the list. Avoid Wifi initialization

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK){
    return; // ESP_now not initialyzed. Avoid linking the nodes
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  for(uint8_t num=0; num<(sizeof(MyMACAddrList)/sizeof(MyMACAddrList[0])); num++)
  {
    if(MyMACAddrList[num] != thisNode->MACAddr)
    {
      esp_now_peer_info_t peerInfo;
      memcpy(peerInfo.peer_addr, MyMACAddrList[num], 6);
      peerInfo.channel = 0;  
      peerInfo.encrypt = false;
      if (esp_now_add_peer(&peerInfo) != ESP_OK)
        return;

      sendESPNowLinkMsg(MyMACAddrList[num], true);
    }
  }
}

bool isThisTheMainNode(void)
{
  return (thisNode == mainNode);
}

void sendESPNowLinkMsg(uint8_t *MACAddr, bool ask4Ack)
{
  s_espnow_link_msg msg;
  msg.type = (uint8_t)LINK_MSG;
  msg.nodeAddr = thisNodeAddr;
  msg.ask4Ack = ask4Ack ? 1u : 0u;

  (void)esp_now_send(MACAddr, (uint8_t *) &msg, sizeof(msg));
}

void sendESPNowModeMsg(uint8_t *MACAddr, uint8_t func_mode, bool isRxNodeUsed)
{
  s_espnow_mode_msg msg;
  msg.type = (uint8_t)MODE_MSG;
  msg.func_mode = func_mode;
  msg.isRxNodeUsed = isRxNodeUsed ? 1u : 0u;

  (void)esp_now_send(MACAddr, (uint8_t *) &msg, sizeof(msg));
}

void sendESPNowLowBattMsg(uint8_t *MACAddr)
{
  s_espnow_default_msg msg;
  msg.type = (uint8_t)LOWBATTERY_MSG;

  (void)esp_now_send(MACAddr, (uint8_t *) &msg, sizeof(msg));
}

void sendESPNowDetectionMsg(uint8_t *MACAddr)
{
  s_espnow_default_msg msg;
  msg.type = (uint8_t)DETECTION_MSG;

  (void)esp_now_send(MACAddr, (uint8_t *) &msg, sizeof(msg));
}

//todo: add descriptions of the functions
//todo: measure the tx time
//todo: if the node receives a message with the address of another node, answer with the node address
//todo: choose the nodes to use (with nodeAddr) when selecting the mode. Set prev/nextNode in the main
//todo: if the node is not the main, but it's selected for the mode, set "mainNode" with the main
//todo: when exit from the mode, release the nodes sending a message with isRxNodeUsed = 0
