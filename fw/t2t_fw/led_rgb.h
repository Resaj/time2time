/**********************************************************************
 * Project: Time2time
 * 
 * File description: This header file contains the parameters and 
 * functions for controlling a RGB LED
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
 * Defines & enums
 *********************************************************************/

#define MAX_BRIGHTNESS    256u
#define MIN_BRIGHTNESS    0u

typedef enum
{
  RGB_RED,
  RGB_YELLOW,
  RGB_GREEN,
  RGB_CYAN,
  RGB_BLUE,
  RGB_MAGENTA,
  RGB_WHITE,
  RGB_BLACK // RGB led off
} e_rgb_color;

/**********************************************************************
 * Global functions
 *********************************************************************/

void rgb_led_init(void);
void set_rgb_led_on_mode(e_rgb_color rgb_color, uint16_t brightness);
void set_rgb_led_off_mode(void);
void set_rgb_led_blink_mode(e_rgb_color rgb_color, uint16_t brightness, uint16_t period, uint16_t period_on);
void set_rgb_led_triangle_wave_mode(e_rgb_color rgb_color, uint16_t brightness, uint16_t period, uint16_t period_on);
void set_rgb_led_sinusoidal_wave_mode(e_rgb_color rgb_color, uint16_t brightness, uint16_t period, uint16_t period_on);
void set_led_time_offset(uint32_t msgRxTime, uint16_t ref_time);
uint16_t get_led_period(void);
void clear_led_time_offset(void);
void led_task(void);

#endif /* LED_RGB_H */
