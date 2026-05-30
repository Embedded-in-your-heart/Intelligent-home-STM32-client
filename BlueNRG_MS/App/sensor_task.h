/**
  ******************************************************************************
  * @file    App/sensor_task.h
  * @brief   FreeRTOS task driving the on-board MEMS sensors
  *          (HTS221 + LSM6DSL via I²C2 / BSP layer).
  *
  *          v1 / Milestone 2a:
  *            - serial-only output on USART1 (VCP)
  *            - no BLE wiring yet
  ******************************************************************************
  */

#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

/**
 * @brief Spawn the SensorTask. Call once from MX_FREERTOS_Init().
 */
void SensorTask_Create(void);

#endif /* SENSOR_TASK_H */
