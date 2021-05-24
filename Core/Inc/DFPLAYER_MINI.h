/*
 * DFPLAYER_MINI.h
 *
 *  Created on: Marh 16, 2021
 */

#ifndef INC_DFPLAYER_MINI_H_
#define INC_DFPLAYER_MINI_H_


void Send_cmdA (uint8_t cmd, uint8_t Parameter1, uint8_t Parameter2);
void DF_PlayFromStart(void);
void DF_Init (uint8_t volume);
void DF_Next (void);
void DF_Pause (void);
void DF_Previous (void);
void DF_Playback (void);
void DF_IncreaseV (void);
void DF_DecreaseV (void);
void DF_SafetyCar(void);
void DF_BestLap(void);
void DF_LastLap(void);
//uint8_t Get_time(uint8_t *buffer);

#endif /* INC_DFPLAYER_MINI_H_ */
