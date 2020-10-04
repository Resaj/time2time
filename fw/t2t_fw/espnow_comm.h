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

#include <stdint.h>

/**********************************************************************
 * Structs
 *********************************************************************/

typedef struct {
  uint8_t nodeAddr;
  uint32_t last_time_sensor_detection;  // ms
} s_espnow_msg;

/**********************************************************************
 * Defines & enums
 *********************************************************************/


/**********************************************************************
 * Global configuration parameters
 *********************************************************************/


/**********************************************************************
 * Global variables
 *********************************************************************/

extern uint8_t moduleNumber;
extern s_espnow_msg incoming_msg;
extern uint8_t espnow_error;
extern uint8_t espnow_add_peer_error;

/**********************************************************************
 * Global functions
 *********************************************************************/

extern void espnow_comm_init(void);
extern void sendESPNowData(s_espnow_msg msg);

#endif /* ESPNOW_COMM_H */
