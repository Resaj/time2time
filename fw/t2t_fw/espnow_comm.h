/**********************************************************************
 * Project: Time2time
 * 
 * File description: This header file contains the parameters and 
 * functions for the ESP-Now communication between ESP32 modules
 * 
 * Author: Rubén Espino San José
 * Puma Pride Robotics Team
 * 
 * License: Attribution-NonCommercial-ShareAlike 4.0
 * International (CC BY-NC-SA 4.0) 
 *********************************************************************/

#ifndef ESPNOW_COMM_H
#define ESPNOW_COMM_H

/**********************************************************************
 * Includes
 *********************************************************************/


/**********************************************************************
 * Defines & enums
 *********************************************************************/

#define ACCEPTANCE_REJECTED     -1
#define ACCEPTANCE_UNCOMPLETED  0
#define ACCEPTANCE_COMPLETED    1

/**********************************************************************
 * Global configuration parameters
 *********************************************************************/


/**********************************************************************
 * Global variables
 *********************************************************************/


/**********************************************************************
 * Global functions
 *********************************************************************/

void espnow_comm_init(void);

uint8_t getNumberOfNodes(void);
uint8_t getThisNodeAddr(void);
bool getNcheckMACAddr(char* macAddr);
bool isThisTheMainNode(void);
uint8_t getLinkedNodes(uint8_t *nodes);
bool isNodeLinked(uint8_t nodeAddress);
bool isMainNodeLinked(void);
bool isEveryWorkingNodeLinked(void);

void sendESPNowModeMsg(uint8_t nodeAddress, uint8_t work_mode, bool request);
void sendESPNowDetectionMsg(uint8_t nodeAddress);
void sendESPNowLowBattMsg(uint8_t nodeAddress, bool battLow_flag, uint16_t battVoltage);

int8_t isAcceptanceCompleted(void);
void configNodes4WorkingMode(uint8_t t2t_mode, uint8_t *remoteNodeList, uint8_t numRemoteNodes);
void releaseWorkingModeComm(void);

void setThisNodeAsBusy(void);

void espnow_task(void);

#endif /* ESPNOW_COMM_H */
