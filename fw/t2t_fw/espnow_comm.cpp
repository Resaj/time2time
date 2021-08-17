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
  bool       linked;    // Linked flag
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
s_t2t_node *mainNode = NULL; // Pointer to the main node position at t2t_node
                             // Determines the node that manages the total times and the functionament mode
                             // It it set when selecting the functionament mode.
uint8_t thisNodeAddr;        // Node address (0-7). Defined by the jumpers in the PCB
s_espnow_default_msg s_receivedMsg;

/**********************************************************************
 * Local functions
 *********************************************************************/

/**********************************************************************
 * @brief Read the number/address of the node from the peripherals.
 * 
 * @returns number of the node. Range [0..7]
 */
uint8_t read_moduleNumber(void)
{
  uint8_t add_bit1 = digitalRead(PIN_JP_ADD1);
  uint8_t add_bit2 = digitalRead(PIN_JP_ADD2);
  uint8_t add_bit3 = digitalRead(PIN_JP_ADD3);
  return (add_bit1<<2) + (add_bit2<<1) + (add_bit3);
}

/**********************************************************************
 * @brief Checks if the MAC address is listed in MyMACAddrList.
 * 
 * @param mac: MAC address of the node
 * @return: the position in the list if the address is found; -1 if not
 */
int8_t isMacAddressListed(uint8_t *mac)
{
  for(int8_t index=0; index<(sizeof(MyMACAddrList)/sizeof(MyMACAddrList[0])); index++)
  {
    bool isEqual = true;

    for(uint8_t pos=0; pos<sizeof(MyMACAddrList[0])/sizeof(uint8_t); pos++)
    {
      if(mac[pos] != MyMACAddrList[index][pos])
      {
        isEqual = false;
        break;
      }
    }
    
    if(isEqual)
    {
      return index;
    }
  }

  return -1;
}

/**********************************************************************
 * @brief ESPNow node configuration. This function reads the node
 * addresses and returns wether or not the node is in the MAC address
 * list.
 * 
 * @return: true if the MAC address is in the list; false if not
 */
bool config_node(void)
{
  uint8_t MACAddr[6];
  int8_t pos = -1;
  
  pinMode(PIN_JP_ADD1, INPUT);
  pinMode(PIN_JP_ADD2, INPUT);
  pinMode(PIN_JP_ADD3, INPUT);

  thisNodeAddr = read_moduleNumber();
  
  t2t_node[thisNodeAddr].prevNode = NULL;
  t2t_node[thisNodeAddr].nextNode = NULL;

  WiFi.macAddress(MACAddr);
  pos = isMacAddressListed(MACAddr);
  if(pos >= 0)
    t2t_node[thisNodeAddr].MACAddr = MyMACAddrList[pos];
  else
    return false;
  
  thisNode = &t2t_node[thisNodeAddr];
  mainNode = &t2t_node[thisNodeAddr];

  return true;
}

/**********************************************************************
 * @brief Handler executed when there is a ESPNow transmission. For now
 * it does nothing.
 */
void OnDataSent(const uint8_t *MAC_Addr, esp_now_send_status_t state) {
}

/**********************************************************************
 * @brief Handler executed when there is a ESPNow reception. Copy the
 * received message to an internal buffer to save the information.
 */
void OnDataRecv(const uint8_t *MAC_Addr, const uint8_t *receivedData, int32_t len)
{
  memcpy(&s_receivedMsg, receivedData, sizeof(s_receivedMsg));
}

/**********************************************************************
 * Global functions
 *********************************************************************/

/**********************************************************************
 * @brief ESPNow initialization. This function configures de node,
 * initializes the ESPNow communication and tries to link with the nodes
 * in the MAC list.
 */
void espnow_comm_init(void)
{
  if(!config_node())
    return; // Node not found in the list. Avoid Wi-Fi initialization

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
      if(esp_now_add_peer(&peerInfo) != ESP_OK)
        return;

      sendESPNowLinkMsg(MyMACAddrList[num], true);
    }
  }
}

/**********************************************************************
 * @brief Returns wether or not this is the main node.
 * 
 * @return: true if this is the main node; false if not
 */
bool isThisTheMainNode(void)
{
  return (thisNode == mainNode);
}

/**********************************************************************
 * @brief Sends an ESPNow message to link to another node.
 * 
 * @param MACAddr: MAC address of the remote node
 * @param ask4Ack: the transmitter requires or not and ACK message
 */
void sendESPNowLinkMsg(uint8_t *MACAddr, bool ask4Ack)
{
  s_espnow_link_msg msg;
  msg.type = (uint8_t)LINK_MSG;
  msg.nodeAddr = thisNodeAddr;
  msg.ask4Ack = ask4Ack ? 1u : 0u;

  (void)esp_now_send(MACAddr, (uint8_t *) &msg, sizeof(msg));
}

/**********************************************************************
 * @brief Sends an ESPNow message to set the mode of a remote linked
 * node.
 * 
 * @param MACAddr: MAC address of the remote node
 * @param func_mode: functionament mode. Range [0..7]
 * @param isRxNodeUsed: the remote node is or isn't used
 */
void sendESPNowModeMsg(uint8_t *MACAddr, uint8_t func_mode, bool isRxNodeUsed)
{
  s_espnow_mode_msg msg;
  msg.type = (uint8_t)MODE_MSG;
  msg.func_mode = func_mode;
  msg.isRxNodeUsed = isRxNodeUsed ? 1u : 0u;

  (void)esp_now_send(MACAddr, (uint8_t *) &msg, sizeof(msg));
}

/**********************************************************************
 * @brief Sends an ESPNow message with a low battery warning. The
 * message includes the voltage of the battery.
 * 
 * @param MACAddr: MAC address of the remote node
 */
void sendESPNowLowBattMsg(uint8_t *MACAddr)
{
  s_espnow_default_msg msg;
  msg.type = (uint8_t)LOWBATTERY_MSG;
  //todo: create a new structure for this message
  //todo: set the low battery flag and the battery voltage

  (void)esp_now_send(MACAddr, (uint8_t *) &msg, sizeof(msg));
}

/**********************************************************************
 * @brief Sends an ESPNow message with the detection of the sensor and
 * the attempt number. It requires an answer message.
 * 
 * @param MACAddr: MAC address of the remote node
 */
void sendESPNowDetectionMsg(uint8_t *MACAddr)
{
  s_espnow_default_msg msg;
  msg.type = (uint8_t)DETECTION_MSG;
  //todo: create a new structure for this message
  //todo: set the attemp number

  (void)esp_now_send(MACAddr, (uint8_t *) &msg, sizeof(msg));
}

/**********************************************************************
 * @brief Returns the number/address of this node.
 * 
 * @return: number of the node. Range [0..7]
 */
uint8_t getThisNodeAddr(void)
{
  return thisNodeAddr;
}

/**********************************************************************
 * @brief Returns the MAC address of this node if it is listed in the
 * configuration.
 * 
 * @return: MAC address the node
 */
void getNcheckMACAddr(char macAddr[18])
{
  if(NULL != t2t_node[thisNodeAddr].MACAddr)
    WiFi.macAddress().toCharArray(macAddr, 18);
  else
    sprintf(macAddr, "MAC not listed");
}

//crear función en ESPnow para conocer el estado de la estructura t2t_node
//al intentar el link al configurar, no devolver con return porque deja de intentar enlazar con otros nodos. Utilizar break
//actualizar periódicamente el estado de link de la estructura en t2t_node con los mensajes de sincronización si no llegan otros mensajes

//todo: read link messages to know the linked nodes
//todo: check the link status between the nodes by writing the linked nodes in the info menu

//todo: if the node receives a message with the address of another node, answer with the node address
//todo: measure the tx time
//todo: choose the nodes to use (with nodeAddr) when selecting the mode. Set prev/nextNode in the main
//todo: if the main node enters in a multiple-node mode, get the required nodes out from the state their are in
//todo: if the node is not the main, but it's selected for the mode, set "mainNode" with the main
//todo: when exit from the mode, release the nodes sending a message with isRxNodeUsed = 0
