/**********************************************************************
 * Project: Time2time
 * 
 * File description: This header file contains the parameters and 
 * functions for controlling the OLED display
 * 
 * Author: Rubén Espino San José
 * Puma Pride Robotics Team
 * 
 * License: Attribution-NonCommercial-ShareAlike 4.0
 * International (CC BY-NC-SA 4.0) 
 *********************************************************************/

#ifndef DISPLAY_H
#define DISPLAY_H

/**********************************************************************
 * Includes
 *********************************************************************/

#include <stdint.h>

/**********************************************************************
 * Enums
 *********************************************************************/

typedef enum {
  MENU_FONT,
  MAIN_TIME_FONT,
  SECONDARY_TIME_FONT
} text_font;

typedef enum {
  ALIGN_LEFT,
  ALIGN_CENTER,
  ALIGN_RIGHT
} text_aligment;

/**********************************************************************
 * Structs
 *********************************************************************/

typedef struct {
  char text[25];            // Text to show
  uint8_t pos_X;            // Text position X
  uint8_t pos_Y;            // Text position Y
  text_font font;           // Font and letter dimensions
  text_aligment alignment;  // Text alignment
} s_display_text;

/**********************************************************************
 * Global functions
 *********************************************************************/

void display_init(void);
void display_clear(void);
void display_set_data(s_display_text *display_data, uint8_t num_lines);
void display_task(void);

#endif /* DISPLAY_H */
