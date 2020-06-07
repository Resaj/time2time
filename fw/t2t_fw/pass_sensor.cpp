/**********************************************************************
 * Project: Time2time
 * 
 * File description: This code file contains the parameters and 
 * functions for reading a distance sensor to detect the pass of the 
 * robots
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
#include "pass_sensor.h"
#include "supply.h"

/**********************************************************************
 * Configuration parameters
 *********************************************************************/

#define SENSOR_ACTIVE_EDGE FALLING // RISING/FALLING

/**********************************************************************
 * Local variables
 *********************************************************************/

portMUX_TYPE enable_critical_zone = portMUX_INITIALIZER_UNLOCKED; // Needed for the interruption

/**********************************************************************
 * Global functions
 *********************************************************************/

bool sensor_interrupt_flag = false;

/**********************************************************************
 * Global functions
 *********************************************************************/

/**********************************************************************
 * @brief Sensor interruption. The program enters when sensor detects 
 * something
 */
void IRAM_ATTR Sensor_isr(void)
{
  portENTER_CRITICAL_ISR(&enable_critical_zone);
  //todo: capture the instant of time to get a more accurate measurement. Storage the time in a variable before the operations.
  sensor_interrupt_flag = true;
  portEXIT_CRITICAL_ISR(&enable_critical_zone);
}

/**********************************************************************
 * @brief Release sensor detection. It is called once the program has 
 * done the operations related to the detection. Enables the next 
 * detection
 */
void release_sensor_detection(void)
{
  portENTER_CRITICAL(&enable_critical_zone);
  sensor_interrupt_flag = false;
  portEXIT_CRITICAL(&enable_critical_zone);
}

/**********************************************************************
 * @brief Configures the pass sensor as input, actives the sensor 
 * supply and initializes the external interruption with falling edge
 */
void pass_sensor_init(void)
{
  power_12v_init();
  pinMode(PIN_SENSOR, INPUT);
  delay(100); // Delay needed to avoid false interruption detections
  attachInterrupt(digitalPinToInterrupt(PIN_SENSOR), Sensor_isr, SENSOR_ACTIVE_EDGE);
}
