/**
  ******************************************************************************
  * @file    App/audio_task.h
  * @brief   DFSDM-based microphone task driving the MP34DT01 PDM mic.
  *
  *          v1 / Milestone 3a:
  *            - PDM input via DFSDM1 + DMA1_Channel4 (circular)
  *            - RMS computed every 200 ms, printed to USART1 VCP
  *            - no BLE wiring yet (deferred to M3b)
  ******************************************************************************
  */

#ifndef AUDIO_TASK_H
#define AUDIO_TASK_H

/**
 * @brief Spawn the AudioTask. Call once from MX_FREERTOS_Init().
 */
void AudioTask_Create(void);

#endif /* AUDIO_TASK_H */
