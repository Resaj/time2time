/**********************************************************************
 * Project: Time2time
 * 
 * File description: This code file contains the parameters and 
 * functions for monitoring the battery voltage and control its charge
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
#include "supply.h"

/**********************************************************************
 * Local configuration parameters
 *********************************************************************/

#define MAX_ADC_DIGITS    4095  // ADC (decimal value)
#define MAX_ADC_VOLT      3300  // Maximum volts allowed at the ADC input pin (volts * 1000)
#define BATT_MONITOR_R1   47    // First resistor (to battery) of the divisor to monitor the battery (kOhmios)
#define BATT_MONITOR_R2   120   // Second resistor (to ground) of the divisor to monitor the battery (kOhmios)
#define BATT_VOLT_ALARM   3500  // Minimum battery volts to active low battery warning (volts * 1000)

#define ALPHA_LP_FILTER   4/5   // Quantity of the above battery measure to impact in the low pass filter

/**********************************************************************
 * Defines
 *********************************************************************/

#define POWER_12V_ON  HIGH
#define POWER_12V_OFF LOW

/**********************************************************************
 * Global variables
 *********************************************************************/

unsigned int g_batt_voltage = 0; // volts * 1000

/**********************************************************************
 * Local functions
 *********************************************************************/

/**********************************************************************
 * @brief Reads the battery ADC input and converts it to a voltaje 
 * value
 * 
 * @returns battery voltage (volts * 1000)
 */
unsigned int read_batteryVoltage(void)
{
  unsigned int adc_value, pin_voltage, batt_volts;

  adc_value = analogRead(PIN_BATT_MONITOR);
  pin_voltage = adc_value * MAX_ADC_VOLT / MAX_ADC_DIGITS;  // Convert ADC value to volts*1000 //todo: change the formule to linearize the ADC ESP32 transfer function
  batt_volts = pin_voltage * (BATT_MONITOR_R1 + BATT_MONITOR_R2)/BATT_MONITOR_R2; // Convert pin volts to battery volts

  return batt_volts;
}

/**********************************************************************
 * @brief Reads the battery ADC input, converts it to a voltaje value 
 * and applies low-pass filter
 */
void measure_batteryVoltage(void)
{
  unsigned int actual_batt_voltage;

  actual_batt_voltage = read_batteryVoltage();

  g_batt_voltage = (unsigned int)((float)(actual_batt_voltage)*(1-ALPHA_LP_FILTER) + (float)g_batt_voltage*ALPHA_LP_FILTER); // Apply low-pass filter
}

/**********************************************************************
 * Global functions
 *********************************************************************/

/**********************************************************************
 * @brief Initializes the battery voltage variable to prevent drastic 
 * changes on its value at the first measures
 */
void batt_monitor_init(void)
{
  g_batt_voltage = read_batteryVoltage();
}

/**********************************************************************
 * @brief Supply function for the battery and charging tasks. This 
 * function has to be called periodically through an scheduler
 */
void supply_task(void)
{
  measure_batteryVoltage();
}

/**********************************************************************
 * @brief Initializes the 12V supply, controlled with a enable pin
 */
void power_12v_init(void)
{
  pinMode(PIN_SLEEP_12V, OUTPUT);
  power_12v_enable();
}

/**********************************************************************
 * @brief Enable the 12V supply
 */
void power_12v_enable(void)
{
  digitalWrite(PIN_SLEEP_12V, POWER_12V_ON);
}

/**********************************************************************
 * @brief Disable the 12V supply
 */
void power_12v_disable(void)
{
  digitalWrite(PIN_SLEEP_12V, POWER_12V_OFF);
}
