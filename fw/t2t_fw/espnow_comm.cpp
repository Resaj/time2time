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
  uint8_t     nodeAddress;    // Node address. Defined by the jumpers on the board, between 0 and 7
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

uint32_t t_origen = 0;          // Origin time to reference the initial linking and enable it again after leaving a working mode
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
 * Local function declarations
 *********************************************************************/

uint8_t read_moduleNumber(void);
int8_t isMacAddressListed(uint8_t *mac);
void init_node_data(uint8_t nodeAddress);
uint8_t config_node(void);
void OnDataSent(const uint8_t *MAC_Addr, esp_now_send_status_t state);
void OnDataRecv(const uint8_t *MAC_Addr, const uint8_t *receivedData, int32_t len);
void sendESPNowLinkMsg(uint8_t nodeAddress, bool ackExpected);

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
  t2t_node[nodeAddress].nodeAddress = nodeAddress;
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
void OnDataSent(const uint8_t *MAC_Addr, esp_now_send_status_t state)
{
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
 * @param request: Request from main (1), answer to accept from
 *                 secondary (0) or release node from main (0)
 */
void sendESPNowModeMsg(uint8_t nodeAddress, uint8_t work_mode, bool request)
{
  s_espnow_mode_msg msg;
  msg.type = (uint8_t)MODE_MSG;
  msg.work_mode = work_mode;
  msg.request = request;

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
 * @brief Return the linked state of a node
 * 
 * @param nodeAddress: address of the remote node. Range: [0..7]
 * @return: true if the node is linked; false if not
 */
bool isNodeLinked(uint8_t nodeAddress)
{
  return t2t_node[nodeAddress].linked;
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
    if(isNodeLinked(index))
    {
      nodes[linkedNodes] = index;
      linkedNodes++;
    }
  }

  return linkedNodes;
}

/**********************************************************************
 * @brief Check if there is any change in the working mode flag because
 * of a remote mode message and reset the flag
 * 
 * @param nodeAddress: address of the remote node. Range: [0..7]
 * @return: >=0 indicates a mode; -1 is nothing; -2 is ACK to join the
 *          mode; -3 is release node
 */
int8_t isAnyModeRequest(uint8_t nodeAddress)
{
  if(thisNode == &t2t_node[nodeAddress])
    return MODE_REQUEST_UNDEFINED;

  int8_t request = flag_modeReq[nodeAddress];
  flag_modeReq[nodeAddress] = MODE_REQUEST_UNDEFINED; // Reset flag
  return request;
}

/**********************************************************************
 * @brief Check if there is any change in the working mode flag of the
 * main node. This function is normaly used to check a possible release
 * message.
 * 
 * @return: >=0 indicates a mode; -1 is nothing; -2 is ACK to join the
 *          mode; -3 is release node
 */
uint8_t isAnyModeRequestFromMain(void)
{
  int8_t request = flag_modeReq[mainNode->nodeAddress];
  flag_modeReq[mainNode->nodeAddress] = MODE_REQUEST_UNDEFINED; // Reset flag
  return request;
}

/**********************************************************************
 * @brief Clean the mode flags to avoid reading old request in case
 * that there weren't read and cleaned before. This function has to be
 * called in every cycle of the state machine.
 */
void clean_mode_flags(void)
{
  for(uint8_t index=0; index<nNodes; index++)
    flag_modeReq[index] = MODE_REQUEST_UNDEFINED;
}

/**********************************************************************
 * @brief Check if there is any change in the working mode flag of the
 * main node. This function is normaly used to check a possible release
 * message.
 * 
 * @param remoteNodeList: pointer to a list that includes the addresses 
 *                        of the remote nodes to join the mode
 * @param numRemoteNodes: number of remote nodes to join the mode
 */
void configNode4WorkingMode(uint8_t *remoteNodeList, uint8_t numRemoteNodes)
{
  if(numRemoteNodes == 0) // Uni-node mode
  {
    thisNode->nextNode = thisNode;
  }
  else // Multi-node mode
  {
    for(uint8_t index=0; index<numRemoteNodes; index++)
    {
      //todo: añadir cambios de prev y next de este nodo para modo multinodo
    }
  }
}

/**********************************************************************
 * @brief Configure this node as secondary to allow being control by
 * the main one
 * 
 * @param mainNodeAddress: address of the remote node. Range: [0..7]
 */
void configThisNodeAsSecondary(uint8_t mainNodeAddress)
{
  mainNode = &t2t_node[mainNodeAddress];
}

/**********************************************************************
 * @brief Release the node while leaving a working mode. This allows
 * the node to control itself again.
 * 
 * @param nodeAddress: address of the remote node. Range: [0..7]
 */
void releaseNodeAfterWorkingMode(uint8_t nodeAddress)
{
  mainNode = thisNode;
  thisNode->prevNode = NULL;
  thisNode->nextNode = NULL;

  t_origen = get_currentTimeMs(); // Allows link with other nodes after leaving a mode
}

/**********************************************************************
 * @brief Send a message to each node working with this one to release
 * them. This function is only called ont this node y the main one.
 */
void releaseWorkingModeComm(void)
{
  s_t2t_node *p, *p_next;
  p = mainNode->nextNode;
  
  while(p != NULL && p != mainNode)
  {
    sendESPNowModeMsg(p->nodeAddress, 0, 0);
    p_next = p->nextNode;
    p->nextNode = NULL;
    p->prevNode = NULL;
    p = p_next;
  }

  mainNode->nextNode = NULL;
  mainNode->prevNode = NULL;
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
        
        if(msg_link.ackExpected) // If the remote node asks for an ACK (it only occurs between a main and a secondary nodes), send a link msg without asking for another ACK
          sendESPNowLinkMsg(nodeAddress, false);
        break;

      case MODE_MSG:
        s_espnow_mode_msg msg_mode;
        memcpy(&msg_mode, rxBuffer[nextMsgBuff2read].msg, msgLength);

        if(msg_mode.request)
        {
          if(thisNode == mainNode && thisNode->nextNode == NULL)
            flag_modeReq[nodeAddress] = msg_mode.work_mode; // Request from a future main node to join the working mode
        }
        else // msg_mode.request == 0
        {
          if(thisNode == mainNode && thisNode->nextNode == NULL)
            flag_modeReq[nodeAddress] = MODE_REQUEST_ACCEPTED; // ACK from the secondary node to accept the main node request
          else if(thisNode->prevNode != NULL && mainNode == &t2t_node[nodeAddress]) // This is a secondary node
            flag_modeReq[nodeAddress] = MODE_REQUEST_RELEASE_NODE; // Request from main node to release this node from the multi-node working mode
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

        //todo: if this node is controlling the node that sends the battery msg, redirect it to the remote device (PC or mobile App)

        break;

      default:
        break;
    }

    nextMsgBuff2read += 1;
    nextMsgBuff2read = nextMsgBuff2read % (sizeof(rxBuffer)/sizeof(s_rxBuffer));
  }

  // Check the communication with the other nodes is alive in case that this is the main node and it is not working in any single mode
  if(thisNode == mainNode && thisNode->nextNode != thisNode)
  {
    t_now = get_currentTimeMs();
  
    for(uint8_t index=0; index<nNodes; index++)
    {
      if(&t2t_node[index] != thisNode)
      {
        // When this node is waiting orders in the main menu, send link periodical messages
        if(thisNode->nextNode == NULL && (t2t_node[index].linked || t_now - t_origen < 4500))
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

//todo: simnplificar MODE_MSG modificando siempre el flag y decidiendo qué hacer en la máquina de estados
//todo: when exit from the mode, release the nodes sending a mode message with request=0 (no funciona bien)
//todo: programar que salga el nodo secundario tras un tiempo si no recibe mensajes del nodo principal

//todo: al entrar en un modo de funcionamiento simple poner next y prev de main al propio main para que en caso de que sea un modo uni-nodo se pueda detectar que está ocupado
  // llamarlo justo al entrar y salir de RUN_MODE en la máquina de estados


//todo: sincronizar parpadeo de los leds durante los modos de funcionamiento (poner parpadeo sinusoidal para saber que está funcionando en modo multinodo)
        //t = 0 en main al enviar el mensaje de modo
        //t = t_now - t_lastMsgRxFromMainNode en el nodo secundario al enviar el mensaje de modo aceptando enlace
//todo: llamar a una función de este módulo desde la máquina de estados para saber si todos los links están enlazados o hay error y tiene que salir del modo multinodo

//todo: measure the tx time
