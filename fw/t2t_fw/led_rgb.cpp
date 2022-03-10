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
  uint16_t red;    // value between 0 and 256
  uint16_t green;  // value between 0 and 256
  uint16_t blue;   // value between 0 and 256
} s_rgb_color;

typedef struct
{
  e_wave_form wave_form;
  e_rgb_color rgb_color;
  uint16_t period;          // If 0, no wave form is applied
  uint16_t period_on;       // Time with the led on (for the square wave mode) or duration of the rising zone (for the triangle and sinusoidal wave modes)
  uint16_t max_brightness;  // The maximum allowed duty cycle for the PWM of the led
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

bool update_led_status = false; // Used to update the status when storage status changes
uint16_t time_offset_ms = 0;    // Used to synchronize the leds of the nodes

/**********************************************************************
 * Local functions declarations
 *********************************************************************/

void set_led(uint8_t pwm_channel_led, uint16_t brightness);
void set_rgb_led(s_rgb_color rgb_color);
void get_basic_color(e_rgb_color selected_color, s_rgb_color *rgb_color);
void set_simple_brightness(s_rgb_color *rgb_color);
void set_square_wave_brightness(s_rgb_color *rgb_color, uint16_t t_period);
void set_triangle_wave_brightness(s_rgb_color *rgb_color, uint16_t t_period);
void set_sinusoidal_wave_brightness(s_rgb_color *rgb_color, uint16_t t_period);

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
void set_led(uint8_t pwm_channel_led, uint16_t brightness)
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
  rgb_color->red = (uint16_t)(led_mode.max_brightness * rgb_color->red / MAX_BRIGHTNESS);
  rgb_color->green = (uint16_t)(led_mode.max_brightness * rgb_color->green / MAX_BRIGHTNESS);
  rgb_color->blue = (uint16_t)(led_mode.max_brightness * rgb_color->blue / MAX_BRIGHTNESS);
}

/**********************************************************************
 * @brief Sets the brightness of the led over the selected basic color. 
 * This function is used for the square wave mode of the led.
 * 
 * @param rgb_color: pointer to struct which contains the brightness 
 *                   of each color from the basic one.
 * @param t_period: time of period in milliseconds
 */
void set_square_wave_brightness(s_rgb_color *rgb_color, uint16_t t_period)
{
  if(t_period <= led_mode.period_on)
    set_simple_brightness(rgb_color);
  else
  {
    *rgb_color = rgb_black;
    set_simple_brightness(rgb_color);
  }
}

/**********************************************************************
 * @brief Sets the brightness of the led over the selected basic color. 
 * This function is used for the triangle wave mode of the led.
 * 
 * @param rgb_color: pointer to struct which contains the brightness 
 *                   of each color from the basic one.
 * @param t_period: time of period in milliseconds
 */
void set_triangle_wave_brightness(s_rgb_color *rgb_color, uint16_t t_period)
{
  uint16_t brightness = 0;
  
  if(t_period <= led_mode.period_on/2 && led_mode.period_on != 0)
    brightness = (uint16_t)(t_period * led_mode.max_brightness/(led_mode.period_on/2));
  else if(t_period < led_mode.period_on)
    brightness = (uint16_t)(led_mode.max_brightness - ((t_period - led_mode.period_on/2) * (led_mode.max_brightness)/(led_mode.period_on/2)));
  
  rgb_color->red = (uint16_t)(brightness * rgb_color->red / MAX_BRIGHTNESS);
  rgb_color->green = (uint16_t)(brightness * rgb_color->green / MAX_BRIGHTNESS);
  rgb_color->blue = (uint16_t)(brightness * rgb_color->blue / MAX_BRIGHTNESS);
}

/**********************************************************************
 * @brief Sets the brightness of the led over the selected basic color. 
 * This function is used for the sinusoidal wave mode of the led.
 * 
 * @param rgb_color: pointer to struct which contains the brightness 
 *                   of each color from the basic one.
 * @param t_period: time of period in milliseconds
 */
void set_sinusoidal_wave_brightness(s_rgb_color *rgb_color, uint16_t t_period)
{
  uint16_t brightness = 0;
  
  if(t_period <= led_mode.period_on/2 && led_mode.period_on != 0)
    brightness = (uint16_t)(led_mode.max_brightness / 2 * (1.0 - cos(t_period * PI / (led_mode.period_on/2))));
  else if(t_period < led_mode.period_on)
    brightness = (uint16_t)(led_mode.max_brightness / 2 * (1.0 + cos((t_period - led_mode.period_on/2) * PI / (led_mode.period_on/2))));
  
  rgb_color->red = (uint16_t)(brightness * rgb_color->red / MAX_BRIGHTNESS);
  rgb_color->green = (uint16_t)(brightness * rgb_color->green / MAX_BRIGHTNESS);
  rgb_color->blue = (uint16_t)(brightness * rgb_color->blue / MAX_BRIGHTNESS);
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
void set_rgb_led_on_mode(e_rgb_color rgb_color, uint16_t brightness)
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
 * @param period_on: time of the period with the led on, in milliseconds
 */
void set_rgb_led_blink_mode(e_rgb_color rgb_color, uint16_t brightness, uint16_t period, uint16_t period_on)
{
  led_mode.wave_form = SQUARE_WAVE;
  led_mode.rgb_color = rgb_color;
  led_mode.period = period;
  led_mode.period_on = period > period_on ? period_on : period;
  led_mode.max_brightness = brightness;

  update_led_status = true;
}

/**********************************************************************
 * @brief Storages the RGB led state. Makes a triangle wave.
 * 
 * @param rgb_color: see enum e_rgb_color to know the available colors
 * @param brightness: integer value between 0 (off) and 255 (on)
 * @param period: period of the signal in milliseconds
 * @param period_on: time of the period with the led on, in milliseconds
 */
void set_rgb_led_triangle_wave_mode(e_rgb_color rgb_color, uint16_t brightness, uint16_t period, uint16_t period_on)
{
  led_mode.wave_form = TRIANGLE_WAVE;
  led_mode.rgb_color = rgb_color;
  led_mode.period = period;
  led_mode.period_on = period > period_on ? period_on : period;
  led_mode.max_brightness = brightness;

  update_led_status = true;
}

/**********************************************************************
 * @brief Storages the RGB led state. Makes a sinusoidal wave.
 * 
 * @param rgb_color: see enum e_rgb_color to know the available colors
 * @param brightness: integer value between 0 (off) and 255 (on)
 * @param period: period of the signal in milliseconds
 * @param period_on: time of the period with the led on, in milliseconds
 */
void set_rgb_led_sinusoidal_wave_mode(e_rgb_color rgb_color, uint16_t brightness, uint16_t period, uint16_t period_on)
{
  led_mode.wave_form = SINUSOIDAL_WAVE;
  led_mode.rgb_color = rgb_color;
  led_mode.period = period;
  led_mode.period_on = period > period_on ? period_on : period;
  led_mode.max_brightness = brightness;

  update_led_status = true;
}

/**********************************************************************
 * @brief Set an offset of the time. This is very useful to synchronize
 * the blink of the leds to see that different nodes are working
 * together in the same mode.
 * 
 * @param msgRxTime: time when the request mode message has been
 *                   received in milliseconds
 * @param ref_time: reference time from the main node, in milliseconds.
 *                  Range: [0-1999]
 */
void set_led_time_offset(uint32_t msgRxTime, uint16_t ref_time)
{
  int16_t t_offset_ms = (uint16_t)((ref_time + led_mode.period - (msgRxTime % led_mode.period)) % led_mode.period);
  time_offset_ms = t_offset_ms;
}

/**********************************************************************
 * @brief Return the period of the led blink
 * 
 * @return: period of the led blink in milliseconds
 */
uint16_t get_led_period(void)
{
  return led_mode.period;
}

/**********************************************************************
 * @brief Clear the offset applied to the led blink. This function must
 * be called after getting this node out from a mode where it was
 * working as secondary.
 */
void clear_led_time_offset(void)
{
  time_offset_ms = 0;
}

/**********************************************************************
 * @brief Controls the RGB led. This function has to be called 
 * periodically through an scheduler.
 */
void led_task(void)
{
  if(update_led_status)
  {
    uint16_t t_led = 0;
    s_rgb_color rgb_color;
    get_basic_color(led_mode.rgb_color, &rgb_color);
    
    t_led = (get_currentTimeMs() + time_offset_ms) % led_mode.period;

    switch(led_mode.wave_form)
    {
      case NO_WAVE:
        set_simple_brightness(&rgb_color);
        update_led_status = false;
        break;
        
      case SQUARE_WAVE:
        set_square_wave_brightness(&rgb_color, (uint16_t)t_led);
        break;
        
      case TRIANGLE_WAVE:
        set_triangle_wave_brightness(&rgb_color, (uint16_t)t_led);
        break;
        
      case SINUSOIDAL_WAVE:
        set_sinusoidal_wave_brightness(&rgb_color, (uint16_t)t_led);
        break;
        
      default:
        update_led_status = false;
        break;
    }

    set_rgb_led(rgb_color);
  }
}
