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
#include "scheduler.h"

/**********************************************************************
 * Structs
 *********************************************************************/

typedef struct s_t2t_node {
  uint8_t     *MACAddr;       // MAC address of the ESP node
  bool        linked;         // Linked flag
  uint32_t    lastTxTime_ms;  // Last time that a message has been transmitted to the remote node
  uint32_t    lastRxTime_ms;  // Last time that a message has been received from the remote node
  s_t2t_node  *prevNode;      // Previous node in the circuit. It is calculated automaticaly when the functionament mode is selected
  s_t2t_node  *nextNode;      // Next node in the circuit. It is calculated automaticaly when the functionament mode is selected
} s_t2t_node;

typedef struct {
  uint8_t nodeAddr  : 4;
  uint8_t msgLength : 4;
  uint8_t msg[3];
  uint32_t rxTime;
} s_rxBuffer;

typedef struct {
  uint8_t type  : 3;
  uint8_t info  : 5;
} s_espnow_default_msg;

typedef struct {
  uint8_t type        : 3;
  uint8_t ackExpected : 1;  // Waiting for an ACK. 1 = true; 0 = false
  uint8_t reserved    : 4;
} s_espnow_link_msg;

typedef struct {
  uint8_t type      : 3;
  uint8_t work_mode : 3;  // Working mode (0-7)
  uint8_t request   : 1;  // Request from main (1), answer to accept from secondary (0) or release node from main (0)
  uint8_t reserved  : 1;
} s_espnow_mode_msg;

typedef struct {
  uint8_t type        : 3;
  uint8_t attempt_num : 3;  // Up to 8 attempts to send the message
  uint8_t reserved    : 2;
} s_espnow_detection_msg;

typedef struct {
  uint8_t type      : 3;
  uint8_t reserved  : 7;
  uint16_t time2show;     // Partial time in milliseconds. Up to 65 sec
} s_espnow_time_msg;

typedef struct {
  uint8_t type        : 3;
  uint8_t lowBatt     : 1;  // Low battery flag. 1 = true; 0 = false
  uint8_t battVoltage : 4;  // Sent as: (batt_volt – 2.7) * 10 where 3V < batt_volt < 4.2V
} s_espnow_lowBattery_msg;

/**********************************************************************
 * Defines & enums
 *********************************************************************/

typedef enum {
  LINK_MSG,
  MODE_MSG,
  DETECTION_MSG,
  TIME_MSG,
  LOWBATTERY_MSG
} msg_type;

/**********************************************************************
 * Local variables
 *********************************************************************/

const uint8_t nNodes = sizeof(MyMACAddrList)/sizeof(MyMACAddrList[0]); // Number of nodes available in the MAC list
s_t2t_node t2t_node[nNodes];    // Matrix to store the information of the nodes. Up to 8 nodes
s_t2t_node *thisNode = NULL;    // Pointer to the node position at t2t_node
s_t2t_node *mainNode = NULL;    // Pointer to the main node position at t2t_node
                                // Determines the node that manages the total times and the functionament mode
                                // It it set when selecting the functionament mode.
uint8_t thisNodeAddr;           // Node address (0-7). Defined by the jumpers in the PCB
s_rxBuffer rxBuffer[8];         // Cyclic buffer to store the received messages
uint8_t nextMsgBuff2write = 0;  // Next byte to write in the rx buffer
uint8_t nextMsgBuff2read = 0;   // Next byte to read from the rx buffer
int8_t flag_modeReq[nNodes];    // Variable to save the mode message request. >=0 indicates a mode; -1 is nothing; -2 is ACK to join the mode; -3 is release node

/**********************************************************************
 * Local functions
 *********************************************************************/

/**********************************************************************
 * @brief Read the number/address of the node from the peripherals.
 * 
 * @return: number of the node. Range [0..7]
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
  for(int8_t index=0; index<nNodes; index++)
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
      return index;
  }

  return -1;
}

/**********************************************************************
 * @brief Initialyze the data of the specified node in the s_t2t_node
 * struct and mode request buffer
 * 
 * @param nodeAddress: address of the remote node. Range: [0..7]
 */
void init_node_data(uint8_t nodeAddress)
{
  t2t_node[nodeAddress].MACAddr = MyMACAddrList[nodeAddress];
  t2t_node[nodeAddress].linked = false;
  t2t_node[nodeAddress].prevNode = NULL;
  t2t_node[nodeAddress].nextNode = NULL;
  t2t_node[nodeAddress].lastTxTime_ms = 0;
  t2t_node[nodeAddress].lastRxTime_ms = 0;

  flag_modeReq[nodeAddress] = -1;
}

/**********************************************************************
 * @brief ESPNow node configuration. This function reads the node
 * addresses and returns wether or not the node is in the MAC address
 * list.
 * 
 * @return: 0 if the MAC address is in the list and in the right 
 *          position; -1 if the MAC is not founded; -2 if the MAC is in
 *          the wrong position
 */
uint8_t config_node(void)
{
  uint8_t MACAddr[6];
  int8_t pos = -1;
  
  pinMode(PIN_JP_ADD1, INPUT);
  pinMode(PIN_JP_ADD2, INPUT);
  pinMode(PIN_JP_ADD3, INPUT);

  thisNodeAddr = read_moduleNumber();

  WiFi.macAddress(MACAddr);
  pos = isMacAddressListed(MACAddr);

  if(pos < 0)
    return -1;
  else if(thisNodeAddr != pos)
    return -2;

  init_node_data(thisNodeAddr);

  thisNode = &t2t_node[thisNodeAddr];
  mainNode = &t2t_node[thisNodeAddr];

  return 0;
}

/**********************************************************************
 * @brief Handler executed when there is a ESPNow transmission
 */
void OnDataSent(const uint8_t *MAC_Addr, esp_now_send_status_t state) {
  int8_t posNodeInList = isMacAddressListed((uint8_t *)MAC_Addr);
  if(posNodeInList >= 0)
    t2t_node[posNodeInList].lastTxTime_ms = get_currentTimeMs(); // Save tx time
}

/**********************************************************************
 * @brief Handler executed when there is a ESPNow reception. Copy the
 * received message to an internal buffer to save the information.
 */
void OnDataRecv(const uint8_t *MAC_Addr, const uint8_t *receivedData, int32_t len)
{
  int32_t t_now = get_currentTimeMs();
  int8_t posNodeInList = isMacAddressListed((uint8_t *)MAC_Addr);

  if(posNodeInList != -1)
  {
    // Save incoming message in the buffer
    rxBuffer[nextMsgBuff2write].nodeAddr = posNodeInList;
    rxBuffer[nextMsgBuff2write].msgLength = len;
    rxBuffer[nextMsgBuff2write].rxTime = t_now;
    for(int8_t index=0; index<len; index++)
      rxBuffer[nextMsgBuff2write].msg[index] = receivedData[index];
      
    nextMsgBuff2write++;
    nextMsgBuff2write = nextMsgBuff2write % (sizeof(rxBuffer)/sizeof(s_rxBuffer));
  }
}

/**********************************************************************
 * @brief Sends an ESPNow message to link or maintain the communication
 * with another node
 * 
 * @param nodeAddress: address of the remote node. Range: [0..7]
 * @param ackExpected: requires or not an ACK for the message
 */
void sendESPNowLinkMsg(uint8_t nodeAddress, bool ackExpected)
{
  s_espnow_link_msg msg;
  msg.type = (uint8_t)LINK_MSG;
  msg.ackExpected = ackExpected ? 1u : 0u;
  
  (void)esp_now_send(t2t_node[nodeAddress].MACAddr, (uint8_t *) &msg, sizeof(msg));
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
  if(config_node() != 0)
    return; // Node not found in the list or wrong placed. Avoid Wi-Fi initialization

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK){
    return; // ESP_now not initialyzed. Avoid linking the nodes
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  for(uint8_t index=0; index<nNodes; index++)
  {
    if(index != thisNodeAddr)
    {
      init_node_data(index);
      esp_now_peer_info_t peerInfo = {};
      memcpy(peerInfo.peer_addr, MyMACAddrList[index], 6);
      peerInfo.channel = 0;
      peerInfo.encrypt = false;
      if(esp_now_add_peer(&peerInfo) != ESP_OK)
        return;
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
 * @brief Sends an ESPNow message to set the mode of a remote linked
 * node.
 * 
 * @param nodeAddress: address of the remote node. Range: [0..7]
 * @param work_mode: working mode. Range [0..7]
 */
void sendESPNowModeMsg(uint8_t nodeAddress, uint8_t work_mode)
{
  s_espnow_mode_msg msg;
  msg.type = (uint8_t)MODE_MSG;
  msg.work_mode = work_mode;

  (void)esp_now_send(t2t_node[nodeAddress].MACAddr, (uint8_t *) &msg, sizeof(msg));
}

/**********************************************************************
 * @brief Sends an ESPNow message with the detection of the sensor and
 * the attempt number. It requires an answer message.
 * 
 * @param nodeAddress: address of the remote node. Range: [0..7]
 */
void sendESPNowDetectionMsg(uint8_t nodeAddress)
{
  s_espnow_default_msg msg;
  msg.type = (uint8_t)DETECTION_MSG;
  //todo: create a new structure for this message
  //todo: set the attemp number

  (void)esp_now_send(t2t_node[nodeAddress].MACAddr, (uint8_t *) &msg, sizeof(msg));
}

/**********************************************************************
 * @brief Sends an ESPNow message with a low battery warning. The
 * message includes the voltage of the battery.
 * 
 * @param nodeAddress: address of the remote node. Range: [0..7]
 * @param battLow_flag: true if the battery is undervoltage; false if not
 * @param battVoltage: voltage of the battery (millivolts)
 */
void sendESPNowLowBattMsg(uint8_t nodeAddress, bool battLow_flag, uint16_t battVoltage)
{
  s_espnow_lowBattery_msg msg;
  msg.type = (uint8_t)LOWBATTERY_MSG;
  msg.lowBatt = battLow_flag? 1 : 0;
  msg.battVoltage = (uint8_t)((battVoltage - 2700)/100);

  (void)esp_now_send(t2t_node[nodeAddress].MACAddr, (uint8_t *) &msg, sizeof(msg));
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
 * @param macAddr: char array to store the MAC address
 * @return: true is the MAC address has been listed; false if not
 */
bool getNcheckMACAddr(char macAddr[18])
{
  if(NULL != t2t_node[thisNodeAddr].MACAddr)
  {
    WiFi.macAddress().toCharArray(macAddr, 18);
    return true;
  }
  else
  {
    sprintf(macAddr, "MAC bad or not listed");
    return false;
  }
}

/**********************************************************************
 * @brief Returns the number of listed nodes
 * 
 * @return: number of listed nodes. Range: [0..7]
 */
uint8_t getNumberOfNodes(void)
{
  return nNodes;
}

/**********************************************************************
 * @brief Returns the amount of linked nodes and saves in the array
 * passed as reference which their are.
 * 
 * @return: number of linked nodes. Range: [0..total-1]. The actual
 *          node can't be counted
 */
uint8_t getLinkedNodes(uint8_t *nodes)
{
  uint8_t linkedNodes = 0;
  
  for(uint8_t index=0; index<nNodes; index++)
  {
    if(t2t_node[index].linked)
    {
      nodes[linkedNodes] = index;
      linkedNodes++;
    }
  }

  return linkedNodes;
}

/**********************************************************************
 * @brief Check if there is any change in the working mode flag because
 * of a remote mode message
 * 
 * @param nodeAddress: address of the remote node. Range: [0..7]
 * @return: >=0 indicates a mode; -1 is nothing; -2 is ACK to join the
 *          mode; -3 is release node
 */
int8_t isAnyModeRequest(uint8_t nodeAddress)
{
  if(thisNode == &t2t_node[nodeAddress])
    return -1;

  int8_t request = flag_modeReq[nodeAddress];
  flag_modeReq[nodeAddress] = -1; // Reset flag
  return request;
}

/**********************************************************************
 * @brief Controls the ESPnow periodical communication. This function
 * has to be called periodically through an scheduler.
 */
void espnow_task(void)
{
  enum e_comm_state {
    INIT_LINK,
    STANDBY
  };

  uint32_t t_now = 0;
  
  // Check incoming messages
  if(nextMsgBuff2write != nextMsgBuff2read)
  {
    uint8_t nodeAddress = 0;
    uint8_t msgLength = 0;

    s_espnow_default_msg defaultMsg;

    nodeAddress = rxBuffer[nextMsgBuff2read].nodeAddr;
    msgLength = rxBuffer[nextMsgBuff2read].msgLength;
    memcpy(&defaultMsg, rxBuffer[nextMsgBuff2read].msg, 1);
    
    t2t_node[nodeAddress].linked = true;
    t2t_node[nodeAddress].lastRxTime_ms = rxBuffer[nextMsgBuff2read].rxTime;

    switch(defaultMsg.type)
    {
      case LINK_MSG:
        s_espnow_link_msg msg_link;
        memcpy(&msg_link, rxBuffer[nextMsgBuff2read].msg, msgLength);
        
        if(msg_link.ackExpected) // If the remote node asks for an ACK, send a link msg without asking for another ACK
          sendESPNowLinkMsg(nodeAddress, false);
        break;

      case MODE_MSG:
        s_espnow_mode_msg msg_mode;
        memcpy(&msg_mode, rxBuffer[nextMsgBuff2read].msg, msgLength);

        if(msg_mode.request)
        {
          if(thisNode == mainNode)
            flag_modeReq[nodeAddress] = msg_mode.work_mode; // Request from the future main node to join the working mode
        }
        else // msg_mode.request == 0
        {
          if(thisNode == mainNode)
            flag_modeReq[nodeAddress] = -2; // ACK from the secondary node to accept the main node request
          else // This is a secondary node
            flag_modeReq[nodeAddress] = -3; // Request from main node to release this node from the multi-node working mode
        }
      
        break;
        
      case DETECTION_MSG:
        s_espnow_detection_msg msg_detection;
        memcpy(&msg_detection, rxBuffer[nextMsgBuff2read].msg, msgLength);

        //todo: fill
      
        break;
        
      case TIME_MSG:
        s_espnow_time_msg msg_time;
        memcpy(&msg_time, rxBuffer[nextMsgBuff2read].msg, msgLength);

        //todo: fill
      
        break;
        
      case LOWBATTERY_MSG:
        s_espnow_lowBattery_msg msg_lowBattery;
        memcpy(&msg_lowBattery, rxBuffer[nextMsgBuff2read].msg, msgLength);

        //todo: redirect to the remote device (PC or mobile App)

        break;

      default:
        break;
    }

    nextMsgBuff2read += 1;
    nextMsgBuff2read = nextMsgBuff2read % (sizeof(rxBuffer)/sizeof(s_rxBuffer));
  }

  // Check the communication with the other nodes is alive in case that this is the main node
  if(thisNode == mainNode)
  {
    t_now = get_currentTimeMs();
  
    for(uint8_t index=0; index<nNodes; index++)
    {
      if(&t2t_node[index] != thisNode)
      {
        // When this node is waiting orders in the main menu, send link periodical messages
        if(thisNode->nextNode == NULL && (t2t_node[index].linked || t_now < 4500))
        {
          if(t_now - t2t_node[index].lastRxTime_ms > 4500)
            t2t_node[index].linked = false;
          else if(t_now - t2t_node[index].lastTxTime_ms > 2000)
            sendESPNowLinkMsg(index, false);
        }
        
        // When this node is working in a mode with multiple nodes, send link messages if there is no communication the last 2 seconds
        else if(thisNode->nextNode != NULL && t2t_node[index].prevNode != NULL)
        {
          if(t_now - t2t_node[index].lastRxTime_ms > 4500)
            t2t_node[index].linked = false;
          else if(t_now - t2t_node[index].lastRxTime_ms > 2000)
            sendESPNowLinkMsg(index, true); // Request ACK. The secondary node won't answer with link messages if it's not required
        }
      }
    }
  }
}

//todo: choose the nodes to use (with nodeAddr) when selecting the mode. Set prev/nextNode in the main
//todo: si un nodo está ocupado en un modo de funcionamiento, no devuelve mensajes de link ni contesta a los nodos que no estén funcionando en conjunto con él (añadirlo al esquema)
//todo: if the node is not the main, but it's selected for the mode, set "mainNode" with the main
//todo: sincronizar parpadeo de los leds durante los modos de funcionamiento (poner parpadeo sinusoidal para saber que está funcionando en modo multinodo)
        //t = 0 en main al enviar el mensaje de modo
        //t = t_now - t_lastMsgRxFromMainNode en el nodo secundario al enviar el mensaje de modo aceptando enlace
//todo: llamar a una función de este módulo desde la máquina de estados para saber si todos los links están enlazados o hay error y tiene que salir del modo multinodo
//todo: when exit from the mode, release the nodes sending a mode message with request=0

//todo: measure the tx time
//todo: utilizar bits reservados del mensaje de link para verificar la versión del programa
