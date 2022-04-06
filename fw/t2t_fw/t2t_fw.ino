/**********************************************************************
 * Project: Time2time
 * 
 * File description: This program is used as a chronometer to measure 
 * times in robotics competitions.
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

#include "scheduler.h"

/**********************************************************************
 * Functions
 *********************************************************************/

void setup() {
  // Initialize tasks
  init_setup_task();

  // Configure wake up mode
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);
}

void loop() {
  scheduler_task();
}
