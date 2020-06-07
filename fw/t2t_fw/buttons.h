/**********************************************************************
 * Project: Time2time
 * 
 * File description: This header file contains the parameters and 
 * functions for reading the state of the buttons
 * 
 * Author: Rubén Espino San José
 * Puma Pride Robotics Team
 * 
 * License: Attribution-NonCommercial-ShareAlike 4.0
 * International (CC BY-NC-SA 4.0) 
 *********************************************************************/

#ifndef BUTTON_H
#define BUTTON_H

/**********************************************************************
 * Defines
 *********************************************************************/

#define BUTTON_A  0
#define BUTTON_B  1
#define BUTTON_C  2

/**********************************************************************
 * Global functions
 *********************************************************************/

void buttons_init(void);
void read_buttons(void);
bool get_button_state(unsigned int);

#endif /* BUTTON_H */
