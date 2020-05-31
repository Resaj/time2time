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

// Display library
#include "SSD1306Wire.h"

// Fonts libraries
#include "fonts/Crushed_Regular_50.h"
#include "fonts/Dialog_plain_11.h"

// Other needed libraries
#include "config/PINSEL.h"

/**********************************************************************
 * Configuration parameters
 *********************************************************************/

#define DISPLAY_ADDRESS 0x3C
#define DISPLAY_REFRESH_PERIOD 1000 // us

/**********************************************************************
 * Global variables
 *********************************************************************/

SSD1306Wire g_display(DISPLAY_ADDRESS, I2C_SDA_DISP, I2C_SCL_DISP);
//todo: abstract g_display to local and add global functions to control the display

#endif /* DISPLAY_H */
