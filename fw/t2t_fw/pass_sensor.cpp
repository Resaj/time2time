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

#include "config/PINSEL.h"
#include "pass_sensor.h"
#include "scheduler.h"
#include "supply.h"

/**********************************************************************
 * Configuration parameters
 *********************************************************************/

#define DEFAULT_SENSOR_ACTIVE_EDGE FALLING // LOW/CHANGE/RISING/FALLING

/**********************************************************************
 * Local variables
 *********************************************************************/

portMUX_TYPE sensor_critical_zone = portMUX_INITIALIZER_UNLOCKED; // Needed for the interruption

uint32_t bufferTimeDetection[20]; // Cyclic buffer to store the times of the detections
uint8_t nextTimeDet2write = 0;    // Next time to write in the storaging buffer
uint8_t nextTimeDet2read = 0;     // Next time to read from the storaging buffer

/**********************************************************************
 * Local functions declarations
 *********************************************************************/

void IRAM_ATTR sensor_isr(void);

/**********************************************************************
 * Local functions
 *********************************************************************/

/**********************************************************************
 * @brief Sensor interruption. The program enters when sensor detects 
 * something
 */
void IRAM_ATTR sensor_isr(void)
{
  portENTER_CRITICAL_ISR(&sensor_critical_zone);
  
  bufferTimeDetection[nextTimeDet2write] = get_currentTimeMs();
  nextTimeDet2write++;
  nextTimeDet2write = nextTimeDet2write % (sizeof(bufferTimeDetection)/sizeof(bufferTimeDetection[0]));
  
  portEXIT_CRITICAL_ISR(&sensor_critical_zone);
}

/**********************************************************************
 * Global functions
 *********************************************************************/

/**********************************************************************
 * @brief Configures the pass sensor as input, actives the sensor 
 * supply and initializes the external interruption with the default
 * active edge, specified with DEFAULT_SENSOR_ACTIVE_EDGE
 */
void pass_sensor_init(void)
{
  power_12v_init();
  pinMode(PIN_SENSOR, INPUT);
  delay(100); // Delay needed to avoid false interruption detections
  set_default_sensor_active_edge();
}

/**********************************************************************
 * @brief Disable the pass sensor to avoid consumption
 *
 * @param lock: lock the pin after disabling
 */
void disable_pass_sensor(bool lock)
{
  power_12v_disable(lock);
}

/**********************************************************************
 * @brief Initializes the external interruption with the default active
 * edge, specified with DEFAULT_SENSOR_ACTIVE_EDGE
 */
void set_default_sensor_active_edge(void)
{
  attachInterrupt(digitalPinToInterrupt(PIN_SENSOR), sensor_isr, DEFAULT_SENSOR_ACTIVE_EDGE);
}

/**********************************************************************
 * @brief Initializes the external interruption with the inverted
 * active edge with respect to the default one, specified with
 * DEFAULT_SENSOR_ACTIVE_EDGE
 */
void invert_sensor_active_edge(void)
{
  uint8_t sensor_active_edge;
  
  switch(DEFAULT_SENSOR_ACTIVE_EDGE)
  {
    case RISING:
      sensor_active_edge = FALLING;
      break;

    case FALLING:
      sensor_active_edge = RISING;
      break;

    default:
      break;
  }

  attachInterrupt(digitalPinToInterrupt(PIN_SENSOR), sensor_isr, sensor_active_edge);
}

/**********************************************************************
 * @brief Check if there are any detections pending of being processed
 *
 * @return: true if there are detections pending; false if not
 */
bool isAnyDetectionPending(void)
{
  return (nextTimeDet2write != nextTimeDet2read);
}

/**********************************************************************
 * @brief Get time of the next detection to be processed
 *
 * @return: 0 if no detections pending; a positive number if yes
 */
uint32_t getNextTimeDetection(void)
{
  uint32_t nextTimeDetection = 0;

  portENTER_CRITICAL(&sensor_critical_zone);

  if(isAnyDetectionPending())
  {
    nextTimeDetection = bufferTimeDetection[nextTimeDet2read];
    nextTimeDet2read++;
    nextTimeDet2read = nextTimeDet2read % (sizeof(bufferTimeDetection)/sizeof(bufferTimeDetection[0]));
  }

  portEXIT_CRITICAL(&sensor_critical_zone);

  return nextTimeDetection;
}

/**********************************************************************
 * @brief Ignores any detection pending of being processed
 */
void ignoreAnyDetectionPending(void)
{
  nextTimeDet2read = nextTimeDet2write;
}
