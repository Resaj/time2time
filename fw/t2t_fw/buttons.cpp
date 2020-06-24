/**********************************************************************
 * Project: Time2time
 * 
 * File description: This code file contains the parameters and 
 * functions for reading the state of the buttons
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
#include "buttons.h"

/**********************************************************************
 * Configuration parameters
 *********************************************************************/

#define INVERTED      0 // Button logic: actived when output is 0
#define NON_INVERTED  1 // Button logic: actived when output is 1

/**********************************************************************
 * Structs
 *********************************************************************/

typedef struct {
  unsigned char pin;    // uC pin connected to the button
  unsigned char state;  // Button status filtered
  char logic;           // Button logic
  bool flag;            // Filtering flag
} s_button;

/**********************************************************************
 * Local variables
 *********************************************************************/

s_button button_A = {PIN_BUTTON_A, false, INVERTED, false};
s_button button_B = {PIN_BUTTON_B, false, INVERTED, false};
s_button button_C = {PIN_BUTTON_C, false, INVERTED, false};

s_button *buttons[3];

/**********************************************************************
 * Local functions
 *********************************************************************/

/**********************************************************************
 * @brief Reads the status of the buttons
 * 
 * @param button: struct data of button which is going to be read
 */
void read_button(s_button button)
{
  unsigned char state = digitalRead(button.pin);
//todo: add filter with flag
  button.state = (button.logic == INVERTED)? !state: state;
}

/**********************************************************************
 * Global functions
 *********************************************************************/

/**********************************************************************
 * @brief Configures the pins of buttons as inputs
 */
void buttons_init(void)
{
  pinMode(button_A.pin, INPUT);
  pinMode(button_B.pin, INPUT);
  pinMode(button_C.pin, INPUT);

  buttons[0] = &button_A;
  buttons[1] = &button_B;
  buttons[2] = &button_C;
}

/**********************************************************************
 * @brief Reads all the button status and storages them locally. This 
 * function has to be called periodically through an scheduler
 */
void buttons_task(void)
{
  read_button(button_A);
  read_button(button_B);
  read_button(button_C);
}

/**********************************************************************
 * @brief returns the button state. 1 = actived; 0 = unactived
 * 
 * @param button_number: the number of the button to be read
 * @returns battery voltage (volts * 1000)
 */
unsigned char get_button_state(button_list button_number)
{
  return buttons[button_number]->state;
}
