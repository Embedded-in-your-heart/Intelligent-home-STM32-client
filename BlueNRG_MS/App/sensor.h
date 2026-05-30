/**
  ******************************************************************************
  * @file    App/sensor.h
  * @brief   Header for the BlueNRG-MS GAP / advertising layer.
  *          (File name kept for compatibility with the SensorDemo template;
  *           the code now contains only GAP wiring, not sensor logic.)
  ******************************************************************************
  */

#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>

#define IDB04A1     0
#define IDB05A1     1
#define BDADDR_SIZE 6

void Set_DeviceConnectable(void);
void user_notify(void *pData);

#endif /* SENSOR_H */
