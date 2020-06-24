/**********************************************************************
 * Project: Time2time
 * 
 * File description: This header file contains the parameters and 
 * functions for reading a distance sensor to detect the pass of the 
 * robots
 * 
 * Author: Rubén Espino San José
 * Puma Pride Robotics Team
 * 
 * License: Attribution-NonCommercial-ShareAlike 4.0
 * International (CC BY-NC-SA 4.0) 
 *********************************************************************/

#ifndef PASS_SENSOR_H
#define PASS_SENSOR_H

/**********************************************************************
 * Global variables
 *********************************************************************/

extern bool sensor_interrupt_flag;
extern unsigned long time_detection;

/**********************************************************************
 * Global functions
 *********************************************************************/

void pass_sensor_init(void);
void release_sensor_detection(void);

#endif /* PASS_SENSOR_H */
