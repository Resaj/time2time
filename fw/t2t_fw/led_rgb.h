/**********************************************************************
 * Project: Time2time
 * 
 * File description: This header file contains the parameters and 
 * functions for control a RGB LED
 * 
 * Author: Rubén Espino San José
 * Puma Pride Robotics Team
 * 
 * License: Attribution-NonCommercial-ShareAlike 4.0
 * International (CC BY-NC-SA 4.0) 
 *********************************************************************/

#ifndef LED_RGB_H
#define LED_RGB_H

/**********************************************************************
 * Defines
 *********************************************************************/

#define LED_ON    HIGH
#define LED_OFF   LOW

/**********************************************************************
 * Global functions
 *********************************************************************/

void rgb_led_init(void);
void turn_off_rgb(void);
void set_rgb_red(bool);
void set_rgb_green(bool);
void set_rgb_blue(bool);

#endif /* LED_RGB_H */
