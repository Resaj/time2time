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

#include "config/PINSEL.h"
#include "supply.h"

/**********************************************************************
 * Local configuration parameters
 *********************************************************************/

#define MAX_ADC_DIGITS    4095u // ADC (decimal value)
#define MAX_ADC_VOLT      3300u // Maximum voltage allowed at the ADC input pin (millivolts)
#define BATT_MONITOR_R1   47u   // First resistor (to battery) of the divisor to monitor the battery (kOhmios)
#define BATT_MONITOR_R2   120u  // Second resistor (to ground) of the divisor to monitor the battery (kOhmios)
#define BATT_VOLT_ALARM   3500u // Minimum battery voltage to active low battery warning (millivolts)

#define ALPHA_LP_FILTER   4.0/5 // Quantity of the battery measures to impact in the low pass filter

/**********************************************************************
 * Defines
 *********************************************************************/

#define POWER_12V_ON  HIGH
#define POWER_12V_OFF LOW

/**********************************************************************
 * Global variables
 *********************************************************************/

uint16_t g_batt_voltage = 0; // millivolts
e_batt_charger_diag batt_charger_diag = NO_INPUT_POWER;

/**********************************************************************
 * Local functions
 *********************************************************************/

/**********************************************************************
 * @brief Reads the battery ADC input and converts it to a voltaje 
 * value
 * 
 * @returns battery voltage (millivolts)
 */
uint16_t read_batteryVoltage(void)
{
  uint16_t adc_value, pin_voltage, batt_voltage;

  adc_value = analogRead(PIN_BATT_MONITOR);
  pin_voltage = (uint16_t)(290 + (adc_value*18.0/22) - pow((adc_value/2500.0),4)*1000.0/11);  // Convert ADC value to millivolts
  batt_voltage = pin_voltage * (BATT_MONITOR_R1 + BATT_MONITOR_R2)/BATT_MONITOR_R2; // Convert pin voltage to battery voltage

  return batt_voltage;
}

/**********************************************************************
 * @brief Reads the battery ADC input, converts it to a voltaje value 
 * and applies low-pass filter
 */
void measure_batteryVoltage(void)
{
  uint16_t actual_batt_voltage;

  actual_batt_voltage = read_batteryVoltage();

  g_batt_voltage = (uint16_t)((actual_batt_voltage)*(1-ALPHA_LP_FILTER) + g_batt_voltage*ALPHA_LP_FILTER); // Apply low-pass filter
}

/**********************************************************************
 * @brief Reads the status of the pins related to the battery charger
 */
void check_battery_charger(void)
{
  uint8_t power_good_status = 0;
  uint8_t stat1_status = 0;
  uint8_t stat2_status = 0;

  power_good_status = digitalRead(PIN_POWER_GOOD);
  stat1_status = digitalRead(PIN_STAT1);
  stat2_status = digitalRead(PIN_STAT2);

  batt_charger_diag = (e_batt_charger_diag)((stat1_status << 2) + (stat2_status << 1) + power_good_status);
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
  pinMode(PIN_POWER_GOOD, INPUT);
  pinMode(PIN_STAT1, INPUT);
  pinMode(PIN_STAT2, INPUT);

  g_batt_voltage = read_batteryVoltage();
}

/**********************************************************************
 * @brief Supply function for the battery and charging tasks. This 
 * function has to be called periodically through an scheduler
 */
void supply_task(void)
{
  measure_batteryVoltage();
  check_battery_charger();
}

/**********************************************************************
 * @brief Returns the voltage of the battery in millivolts
 * 
 * @return: voltage of the battery (millivolts)
 */
uint16_t get_battery_voltage(void)
{
  return g_batt_voltage;
}

/**********************************************************************
 * @brief Compare the battery voltage to determine wether it is
 * undervoltage or not.
 * 
 * @return: true if the battery is undervoltage; false if not
 */
bool is_battery_undervoltage(void)
{
  return (g_batt_voltage < BATT_VOLT_ALARM);
}

/**********************************************************************
 * @brief Initializes the 12V supply, controlled with a enable pin
 */
void power_12v_init(void)
{
  gpio_hold_dis((gpio_num_t)PIN_SLEEP_12V); // Release pin held before a possible deep sleep
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
 * 
 * @param lock: lock the pin after disabling
 */
void power_12v_disable(bool lock)
{
  digitalWrite(PIN_SLEEP_12V, POWER_12V_OFF);

  if(lock)
    gpio_hold_en((gpio_num_t)PIN_SLEEP_12V);
}
