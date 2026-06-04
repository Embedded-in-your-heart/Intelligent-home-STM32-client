/**
  ******************************************************************************
  * @file    App/audio_task.h
  * @brief   DFSDM-based microphone task driving the MP34DT01 PDM mic.
  *
  *            - PDM input via DFSDM1 + DMA1_Channel4 (circular, interrupt-driven)
  *            - RMS over each 200 ms window → MicLevel + LoudAlert over BLE
  *            - one [mic] trace line per second to USART1 VCP
  ******************************************************************************
  */

#ifndef AUDIO_TASK_H
#define AUDIO_TASK_H

/**
 * @brief Spawn the AudioTask. Call once from MX_FREERTOS_Init().
 */
void AudioTask_Create(void);

#endif /* AUDIO_TASK_H */
