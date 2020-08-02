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

enum e_buzzer_mode {
  SIMPLE_BEEP,
  DOUBLE_BEEP,
  TRIPLE_BEEP,
  LARGE_BEEP
};

/**********************************************************************
 * Global functions
 *********************************************************************/

void buzzer_init(void);
void set_buzzer_mode(e_buzzer_mode selected_mode);
void buzzer_task(void);

#endif /* BUZZER_H */
