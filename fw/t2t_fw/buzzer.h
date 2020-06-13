/**********************************************************************
 * Project: Time2time
 * 
 * File description: This header file contains the parameters and 
 * functions for controlling a buzzer
 * 
 * Author: Rubén Espino San José
 * Puma Pride Robotics Team
 * 
 * License: Attribution-NonCommercial-ShareAlike 4.0
 * International (CC BY-NC-SA 4.0) 
 *********************************************************************/

#ifndef BUZZER_H
#define BUZZER_H

/**********************************************************************
 * Defines & enums
 *********************************************************************/

enum buzzer_mode_list{
  MUTE,
  SIMPLE_BEEP,
  DOUBLE_BEEP
};

/**********************************************************************
 * Global functions
 *********************************************************************/

void buzzer_init(void);
void set_buzzer_mode(buzzer_mode_list);
void buzzer_task(void);

#endif /* BUZZER_H */
