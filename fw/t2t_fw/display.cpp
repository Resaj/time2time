/**********************************************************************
 * Project: Time2time
 * 
 * File description: This code file contains the parameters and 
 * functions for controlling the OLED display
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
#include "display.h"
#include "config/PINSEL.h"

// Display library
#include "SSD1306Wire.h"

// Fonts libraries
#include "fonts/Crushed_Regular_50.h"
#include "fonts/Dialog_plain_11.h"

/**********************************************************************
 * Configuration parameters
 *********************************************************************/

#define DISPLAY_ADDRESS 0x3C
#define DISPLAY_REFRESH_PERIOD 1000 // us

/**********************************************************************
 * Global variables
 *********************************************************************/

bool refresh_display = true;

/**********************************************************************
 * Global variables
 *********************************************************************/

SSD1306Wire g_display(DISPLAY_ADDRESS, PIN_I2C_SDA_DISP, PIN_I2C_SCL_DISP);

/**********************************************************************
 * Global functions
 *********************************************************************/

/**********************************************************************
 * @brief Initializes the OLED display
 */
void display_init(void)
{
  g_display.init();
  g_display.flipScreenVertically();
}

/**********************************************************************
 * @brief Clears the OLED display
 */
void display_clear(void)
{
  g_display.clear();
}

/**********************************************************************
 * @brief Stores the data to show in the OLED display
 * 
 * @param display_data: contains text lines, positions, fonts and 
 * aligment
 */
void display_set_data(s_display_text *display_data)
{
  char i = 0;
  
  for(i=0; i< sizeof(display_data)/sizeof(s_display_text); i++)
  {
    switch(display_data[i].font)
    {
      case MENU_FONT:
        g_display.setFont(Dialog_plain_11);
        break;

      case MAIN_TIME_FONT:
        g_display.setFont(Crushed_Regular_50);
        break;

      case SECONDARY_TIME_FONT:
        g_display.setFont(Dialog_plain_11);
        break;

      default:
        g_display.setFont(Dialog_plain_11);
        break;
    }

    switch(display_data[i].aligment)
    {
      case ALIGN_CENTER:
        g_display.setTextAlignment(TEXT_ALIGN_CENTER);
        break;
        
      case ALIGN_RIGHT:
        g_display.setTextAlignment(TEXT_ALIGN_RIGHT);
        break;

      default: // Align to the left
        g_display.setTextAlignment(TEXT_ALIGN_LEFT);
        break;
    }

    g_display.drawString(display_data[i].pos_X, display_data[i].pos_Y, display_data[i].text);
  }

  refresh_display = true;
}

/**********************************************************************
 * @brief Clears the OLED display and shows the new data. This function 
 * has to be called periodically through an scheduler
 */
void display_task(void)
{
  if(refresh_display)
  {
    display_clear();
    g_display.display();
    refresh_display = false;
  }
}
