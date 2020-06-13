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
 * Defines & enums
 *********************************************************************/

enum button_list {
  BUTTON_A,
  BUTTON_B,
  BUTTON_C
};

/**********************************************************************
 * Global functions
 *********************************************************************/

void buttons_init(void);
void read_buttons(void);
unsigned char get_button_state(button_list);

#endif /* BUTTON_H */