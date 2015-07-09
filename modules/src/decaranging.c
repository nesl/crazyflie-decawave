/**
 * PDM - NESL, UCLA
 * Decawave (DW1000) Ranging Module
 */

#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "system.h"
#include "pm.h"
#include "log.h"
#include "ledseq.h"
#include "param.h"
#include "debug.h"

static bool isInit;

static void stabilizerAltHoldUpdate(void);
static void distributePower(const uint16_t thrust, const int16_t roll,
                            const int16_t pitch, const int16_t yaw);
static uint16_t limitThrust(int32_t value);
static void stabilizerTask(void* param);
static float constrain(float value, const float minVal, const float maxVal);
static float deadband(float value, const float threshold);

void decarangingInit(void)
{
  if(isInit)
    return;

  xTaskCreate(decarangingTask, (const signed char * const)DECARANGING_TASK_NAME,
              DECARANGING_TASK_STACKSIZE, NULL, DECARANGING_TASK_PRI, NULL);

  isInit = true;
}


static void decarangingTask(void* param)
{

  vTaskSetApplicationTaskTag(0, (void*)TASK_DECARANGING_ID_NBR);

  //Wait for the system to be fully started before starting ranging
  systemWaitStart();

  lastWakeTime = xTaskGetTickCount ();

  while(1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(IMU_UPDATE_FREQ)); // 500Hz
  }
}

// add log group below?