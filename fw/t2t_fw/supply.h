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

//todo: implement battery charge manager interface (POWER_GOOD, STAT1, STAT2)

/**********************************************************************
 * Includes
 *********************************************************************/

#include "Arduino.h"

/**********************************************************************
 * Defines & enums
 *********************************************************************/

typedef enum
{
  // (STAT1 << 2) + (STAT2 << 1) + PG
  TEMP_OR_TIMER_FAULT = 0,  // 0 - Temperature fault or timer fault
  PRECONDITIONING     = 2,  // 2 - Preconditioning, constant current or constant voltage
  LOW_BATTERY_OUTPUT  = 3,  // 3 - Low battery output
  CHARGE_COMPLETE     = 4,  // 4 - Charge complete
  NO_BATTERY          = 6,  // 6 - Shutdown (VDD = VIN) or no battery present
  NO_INPUT_POWER      = 7   // 7 - Shutdown (VDD = VBAT) or no input power present
} e_batt_charger_diag;

/**********************************************************************
 * Global configuration parameters
 *********************************************************************/

#define BATT_VOLT_ALARM   3500u // Minimum battery voltage to active low battery warning (millivolts)

/**********************************************************************
 * Global variables
 *********************************************************************/

extern unsigned int g_batt_voltage; // millivolts
extern e_batt_charger_diag batt_charger_diag;

/**********************************************************************
 * Global functions
 *********************************************************************/

void batt_monitor_init(void);
void supply_task(void);
void power_12v_init(void);
void power_12v_enable(void);
void power_12v_disable(void);

#endif /* SUPPLY_H */
