/**********************************************************************
 * Project: Time2time
 * 
 * File description: This header file contains the parameters and 
 * functions for controlling the scheduler of the program
 * 
 * Author: Rubén Espino San José
 * Puma Pride Robotics Team
 * 
 * License: Attribution-NonCommercial-ShareAlike 4.0
 * International (CC BY-NC-SA 4.0) 
 *********************************************************************/

#ifndef SCHEDULER_H
#define SCHEDULER_H

/**********************************************************************
 * Includes
 *********************************************************************/

#include <stdint.h>

/**********************************************************************
 * Global variables
 *********************************************************************/

extern uint32_t t_now_ms;

/**********************************************************************
 * Global functions
 *********************************************************************/

void init_setup_task(void);
void scheduler_task(void);

#endif /* SCHEDULER_H */
