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
#include "scheduler.h"

/**********************************************************************
 * Defines & enums
 *********************************************************************/

typedef enum e_wave_form
{
  NO_WAVE,
  SQUARE_WAVE,
  TRIANGLE_WAVE,
  SINUSOIDAL_WAVE
};

/**********************************************************************
 * Structs
 *********************************************************************/

typedef struct
{
  unsigned int red;    // value between 0 and 256
  unsigned int green;  // value between 0 and 256
  unsigned int blue;   // value between 0 and 256
} s_rgb_color;

typedef struct
{
  e_wave_form wave_form;
  e_rgb_color rgb_color;
  unsigned int period;          // If 0, no wave form is applied
  unsigned int period_up;       // Time with the led on (for the square wave mode) or duration of the rising zone (for the triangle and sinusoidal wave modes)
  unsigned int max_brightness;  // The maximum allowed duty cycle for the PWM of the led
} s_led_mode;

/**********************************************************************
 * Configuration parameters
 *********************************************************************/

#define INVERTED_LOGIC true // true: led on at low level; false: led on at high level

#define PWM_FREQ_HZ         1000u
#define PWM_CHANNEL_LED_R   1u
#define PWM_CHANNEL_LED_G   2u
#define PWM_CHANNEL_LED_B   3u
#define PWM_RESOLUTION      8u

/**********************************************************************
 * Local variables
 *********************************************************************/

const s_rgb_color rgb_red =     { MAX_BRIGHTNESS, MIN_BRIGHTNESS, MIN_BRIGHTNESS };
const s_rgb_color rgb_yellow =  { MAX_BRIGHTNESS, MAX_BRIGHTNESS, MIN_BRIGHTNESS };
const s_rgb_color rgb_green =   { MIN_BRIGHTNESS, MAX_BRIGHTNESS, MIN_BRIGHTNESS };
const s_rgb_color rgb_cyan =    { MIN_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS };
const s_rgb_color rgb_blue =    { MIN_BRIGHTNESS, MIN_BRIGHTNESS, MAX_BRIGHTNESS };
const s_rgb_color rgb_magenta = { MAX_BRIGHTNESS, MIN_BRIGHTNESS, MAX_BRIGHTNESS };
const s_rgb_color rgb_white =   { MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS };
const s_rgb_color rgb_black =   { MIN_BRIGHTNESS, MIN_BRIGHTNESS, MIN_BRIGHTNESS };

s_led_mode led_mode = { NO_WAVE, RGB_WHITE, 0, MIN_BRIGHTNESS };

bool update_led_status = false; // used to update the status when storage status changes

/**********************************************************************
 * Local functions
 *********************************************************************/

/**********************************************************************
 * @brief Sets the led status.
 * 
 * @param pwm_channel_led: PWM_CHANNEL_LED_R, PWM_CHANNEL_LED_G or 
 *    PWM_CHANNEL_LED_B
 * @param brightness: the duty_cycle. It's an integer value between 0 
 *    and 256
 */
void set_led(unsigned char pwm_channel_led, unsigned int brightness)
{
  brightness = (brightness > MAX_BRIGHTNESS) ? MAX_BRIGHTNESS : brightness;
  
  ledcWrite(pwm_channel_led, INVERTED_LOGIC ? (MAX_BRIGHTNESS - brightness) : brightness);
}

/**********************************************************************
 * @brief Sets the RGB led status. Call this function only at
 * led_task().
 * 
 * @param rgb_color: struct which contains the brightness of each color 
 *    of an RGB led
 */
void set_rgb_led(s_rgb_color rgb_color)
{
  set_led(PWM_CHANNEL_LED_R, rgb_color.red);
  set_led(PWM_CHANNEL_LED_G, rgb_color.green);
  set_led(PWM_CHANNEL_LED_B, rgb_color.blue);
}

/**********************************************************************
 * @brief Gets the RGB parameters of the selected basic color. This 
 * functions is called before the function which sets the brightness.
 * 
 * @param selected_color: this is the selected color for the RGB led. 
 *    See e_rgb_color enum to know the available colors.
 * @param rgb_color: pointer to struct which will contain the 
 *    brightness of each color from the basic one.
 */
void get_basic_color(e_rgb_color selected_color, s_rgb_color *rgb_color)
{
  switch(selected_color)
  {
    case RGB_RED:
      *rgb_color = rgb_red;
      break;
      
    case RGB_YELLOW:
      *rgb_color = rgb_yellow;
      break;
      
    case RGB_GREEN:
      *rgb_color = rgb_green;
      break;
      
    case RGB_CYAN:
      *rgb_color = rgb_cyan;
      break;
      
    case RGB_BLUE:
      *rgb_color = rgb_blue;
      break;
      
    case RGB_MAGENTA:
      *rgb_color = rgb_magenta;
      break;
      
    case RGB_WHITE:
      *rgb_color = rgb_white;
      break;
      
    default: // Black color. Turns off the RGB led
      *rgb_color = rgb_black;
      break;
  }
}

/**********************************************************************
 * @brief Sets the brightness of the led over the selected basic color. 
 * This function is used for the on mode of the led.
 * 
 * @param rgb_color: pointer to struct which contains the brightness 
 * of each color from the basic one.
 */
void set_simple_brightness(s_rgb_color *rgb_color)
{
  rgb_color->red = (unsigned int)(led_mode.max_brightness * rgb_color->red / MAX_BRIGHTNESS);
  rgb_color->green = (unsigned int)(led_mode.max_brightness * rgb_color->green / MAX_BRIGHTNESS);
  rgb_color->blue = (unsigned int)(led_mode.max_brightness * rgb_color->blue / MAX_BRIGHTNESS);
}

/**********************************************************************
 * @brief Sets the brightness of the led over the selected basic color. 
 * This function is used for the square ware mode of the led.
 * 
 * @param rgb_color: pointer to struct which contains the brightness 
 * of each color from the basic one.
 * @param t_period: time of period passed in milliseconds
 */
void set_square_wave_brightness(s_rgb_color *rgb_color, unsigned int t_period)
{
  if(t_period <= led_mode.period_up)
    set_simple_brightness(rgb_color);
  else
  {
    *rgb_color = rgb_black;
    set_simple_brightness(rgb_color);
  }
}

/**********************************************************************
 * @brief Sets the brightness of the led over the selected basic color. 
 * This function is used for the triangle ware mode of the led.
 * 
 * @param rgb_color: pointer to struct which contains the brightness 
 * of each color from the basic one.
 * @param t_period: time of period passed in milliseconds
 */
void set_triangle_wave_brightness(s_rgb_color *rgb_color, unsigned int t_period)
{
  unsigned int brightness = 0;
  
  if(t_period <= led_mode.period_up && led_mode.period_up != 0)
    brightness = (unsigned int)(t_period * led_mode.max_brightness/led_mode.period_up);
  else if(led_mode.period != led_mode.period_up)
    brightness = (unsigned int)(led_mode.max_brightness - ((t_period - led_mode.period_up) * (led_mode.max_brightness)/(led_mode.period - led_mode.period_up)));
  
  rgb_color->red = (unsigned int)(brightness * rgb_color->red / MAX_BRIGHTNESS);
  rgb_color->green = (unsigned int)(brightness * rgb_color->green / MAX_BRIGHTNESS);
  rgb_color->blue = (unsigned int)(brightness * rgb_color->blue / MAX_BRIGHTNESS);
}

/**********************************************************************
 * @brief Sets the brightness of the led over the selected basic color. 
 * This function is used for the sinusoidal ware mode of the led.
 * 
 * @param rgb_color: pointer to struct which contains the brightness 
 * of each color from the basic one.
 * @param t_period: time of period passed in milliseconds
 */
void set_sinusoidal_wave_brightness(s_rgb_color *rgb_color, unsigned int t_period)
{
  //todo: modify this function
  unsigned int brightness = 0;
  
  if(t_period <= led_mode.period_up && led_mode.period_up != 0)
    brightness = (unsigned int)(led_mode.max_brightness / 2 * (1.0 - cos(t_period * PI / led_mode.period_up)));
  else if(led_mode.period != led_mode.period_up)
    brightness = (unsigned int)(led_mode.max_brightness / 2 * (1.0 + cos((t_period - led_mode.period_up) * PI / (led_mode.period - led_mode.period_up))));
  
  rgb_color->red = (unsigned int)(brightness * rgb_color->red / MAX_BRIGHTNESS);
  rgb_color->green = (unsigned int)(brightness * rgb_color->green / MAX_BRIGHTNESS);
  rgb_color->blue = (unsigned int)(brightness * rgb_color->blue / MAX_BRIGHTNESS);
}

/**********************************************************************
 * Global functions
 *********************************************************************/

/**********************************************************************
 * @brief Configures the RGB LED pins as PWM outputs and initializes 
 * them to off.
 */
void rgb_led_init(void)
{
  ledcSetup(PWM_CHANNEL_LED_R, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_LED_G, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_LED_B, PWM_FREQ_HZ, PWM_RESOLUTION);

  ledcAttachPin(PIN_LED_R, PWM_CHANNEL_LED_R);
  ledcAttachPin(PIN_LED_G, PWM_CHANNEL_LED_G);
  ledcAttachPin(PIN_LED_B, PWM_CHANNEL_LED_B);

  set_rgb_led_off_mode();
}

/**********************************************************************
 * @brief Storages the RGB led ON state
 * 
 * @param rgb_color: see enum e_rgb_color to know the available colors
 * @param brightness: integer value between 0 (off) and 255 (on)
 */
void set_rgb_led_on_mode(e_rgb_color rgb_color, unsigned int brightness)
{
  led_mode.wave_form = NO_WAVE;
  led_mode.rgb_color = rgb_color;
  led_mode.max_brightness = brightness;

  update_led_status = true;
}

/**********************************************************************
 * @brief Storages the RGB led OFF state
 */
void set_rgb_led_off_mode(void)
{
  led_mode.wave_form = NO_WAVE;
  led_mode.max_brightness = MIN_BRIGHTNESS;

  update_led_status = true;
}

/**********************************************************************
 * @brief Storages the RGB led state. Makes a square wave.
 * 
 * @param rgb_color: see enum e_rgb_color to know the available colors
 * @param brightness: integer value between 0 (off) and 255 (on)
 * @param period: period of the blink in milliseconds
 * @param period_up: time of the period with the led on, in milliseconds
 */
void set_rgb_led_blink_mode(e_rgb_color rgb_color, unsigned int brightness, unsigned int period, unsigned int period_up)
{
  led_mode.wave_form = SQUARE_WAVE;
  led_mode.rgb_color = rgb_color;
  led_mode.period = period;
  led_mode.period_up = period_up;
  led_mode.max_brightness = brightness;

  update_led_status = true;
}

/**********************************************************************
 * @brief Storages the RGB led state. Makes a triangle wave.
 * 
 * @param rgb_color: see enum e_rgb_color to know the available colors
 * @param brightness: integer value between 0 (off) and 255 (on)
 * @param period: period of the blink in milliseconds
 * @param period_up: time of the period with the led on, in milliseconds
 */
void set_rgb_led_triangle_wave_mode(e_rgb_color rgb_color, unsigned int brightness, unsigned int period, unsigned int period_up)
{
  led_mode.wave_form = TRIANGLE_WAVE;
  led_mode.rgb_color = rgb_color;
  led_mode.period = period;
  led_mode.period_up = period_up;
  led_mode.max_brightness = brightness;

  update_led_status = true;
}

/**********************************************************************
 * @brief Storages the RGB led state. Makes a sinusoidal wave.
 * 
 * @param rgb_color: see enum e_rgb_color to know the available colors
 * @param brightness: integer value between 0 (off) and 255 (on)
 * @param period: period of the blink in milliseconds
 * @param period_up: time of the period with the led on, in milliseconds
 */
void set_rgb_led_sinusoidal_wave_mode(e_rgb_color rgb_color, unsigned int brightness, unsigned int period, unsigned int period_up)
{
  led_mode.wave_form = SINUSOIDAL_WAVE;
  led_mode.rgb_color = rgb_color;
  led_mode.period = period;
  led_mode.period_up = period_up;
  led_mode.max_brightness = brightness;

  update_led_status = true;
}

/**********************************************************************
 * @brief Controls the RGB led. This function has to be called 
 * periodically through an scheduler.
 */
void led_task(void)
{
  static unsigned long t = 0;

  if(update_led_status)
  {
    s_rgb_color rgb_color;
    get_basic_color(led_mode.rgb_color, &rgb_color);

    switch(led_mode.wave_form)
    {
      case NO_WAVE:
        set_simple_brightness(&rgb_color);
        update_led_status = false;
        break;
        
      case SQUARE_WAVE:
        set_square_wave_brightness(&rgb_color, (unsigned int)(t_now_ms - t));
        break;
        
      case TRIANGLE_WAVE:
        set_triangle_wave_brightness(&rgb_color, (unsigned int)(t_now_ms - t));
        break;
        
      case SINUSOIDAL_WAVE:
        set_sinusoidal_wave_brightness(&rgb_color, (unsigned int)(t_now_ms - t));
        break;
        
      default:
        update_led_status = false;
        break;
    }

    set_rgb_led(rgb_color);

    if(t_now_ms - t >= led_mode.period)
      t = t_now_ms;
  }
}
