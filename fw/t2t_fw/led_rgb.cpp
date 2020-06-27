/**********************************************************************
 * Project: Time2time
 * 
 * File description: This code file contains the parameters and 
 * functions for controlling a RGB LED
 * 
 * Author: Rubén Espino San José
 * Puma Pride Robotics Team
 * 
 * License: Attribution-NonCommercial-ShareAlike 4.0
 * International (CC BY-NC-SA 4.0) 
 *********************************************************************/

/**********************************************************************
 * Includes
 *********************************************************************/

#include "Arduino.h"
#include "config/PINSEL.h"
#include "led_rgb.h"

/**********************************************************************
 * Configuration parameters
 *********************************************************************/

#define INVERTED_LOGIC true //true: led on at low level; false: led on at high level

/**********************************************************************
 * Global functions
 *********************************************************************/

/**********************************************************************
 * @brief Configures the RGB LED pins as output and initializes them to 
 * off
 */
void rgb_led_init(void)
{
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);

  turn_off_rgb();
}

/**********************************************************************
 * @brief Turns off the RGB LED
 */
void turn_off_rgb(void)
{
  set_rgb_red(LED_OFF);
  set_rgb_green(LED_OFF);
  set_rgb_blue(LED_OFF);
}

/**********************************************************************
 * @brief Sets on/off the red of the RGB LED
 * 
 * @param state: 1 (on) or 0 (off)
 */
void set_rgb_red(unsigned char state)
{
  digitalWrite(PIN_LED_R, INVERTED_LOGIC? !state:state);
}

/**********************************************************************
 * @brief Sets on/off the green of the RGB LED
 * 
 * @param state: 1 (on) or 0 (off)
 */
void set_rgb_green(unsigned char state)
{
  digitalWrite(PIN_LED_G, INVERTED_LOGIC? !state:state);
}

/**********************************************************************
 * @brief Sets on/off the blue of the RGB LED
 * 
 * @param state: 1 (on) or 0 (off)
 */
void set_rgb_blue(unsigned char state)
{
  digitalWrite(PIN_LED_B, INVERTED_LOGIC? !state:state);
}

/**********************************************************************
 * @brief Controls the RGB led state. This function has to be called 
 * periodically through an scheduler
 */
void leds_task(void)
{
  //todo: red blink when battery is low
}

//todo: add defines for the blink mode (no_blink, square, triangle or sinusoidal), period, duty cycle (for square wave) and min and max bright
//todo: control the leds brigth with a PWM, like the buzzer control
//todo: abstract set and turn off functions locally. Create global function to set the desired states and manage them in the task function
