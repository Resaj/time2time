/**********************************************************************
 * Project: Time2time
 * 
 * File description: This header file contains the parameters and 
 * functions for monitoring the battery voltage and control its charge
 * 
 * Author: Rubén Espino San José
 * Puma Pride Robotics Team
 * 
 * License: Attribution-NonCommercial-ShareAlike 4.0
 * International (CC BY-NC-SA 4.0) 
 *********************************************************************/

#ifndef SUPPLY_H
#define SUPPLY_H

//todo: implement battery charge manager interface

/**********************************************************************
 * Global configuration parameters
 *********************************************************************/

#define BATT_VOLT_ALARM   3500  // Minimum battery volts to active low battery warning (volts * 1000)

/**********************************************************************
 * Global variables
 *********************************************************************/

extern unsigned int g_batt_voltage; // volts * 1000

/**********************************************************************
 * Global functions
 *********************************************************************/

void batt_monitor_init(void);
void measure_battery_voltage(void);

#endif /* SUPPLY_H */
