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
 * Global configuration parameters
 *********************************************************************/


/**********************************************************************
 * Global variables
 *********************************************************************/


/**********************************************************************
 * Global functions
 *********************************************************************/

void espnow_comm_init(void);
bool isThisTheMainNode(void);
void sendESPNowLinkMsg(uint8_t *MACAddr, bool ask4Ack);
void sendESPNowModeMsg(uint8_t *MACAddr, uint8_t func_mode, bool isRxNodeUsed);
void sendESPNowLowBattMsg(uint8_t *MACAddr);
void sendESPNowDetectionMsg(uint8_t *MACAddr);
uint8_t getThisNodeAddr(void);
bool getNcheckMACAddr(char* macAddr);
uint8_t getLinkedNodes(uint8_t *nodes);
void espnow_task(void);

#endif /* ESPNOW_COMM_H */
