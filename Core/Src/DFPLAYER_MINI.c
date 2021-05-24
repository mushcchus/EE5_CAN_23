/*
 * DFPLAYER_MINI.c
 *
 *  Created on: March 16, 2021
 */


#include "stm32f3xx_hal.h"
#include "stdio.h"
#include "string.h"

extern UART_HandleTypeDef huart2;
#define DF_UART &huart2

#define Source      0x02  // TF CARD

/*#define Previous_Key   GPIO_PIN_1
#define Previous_Port  GPIOA
#define Pause_Key      GPIO_PIN_2
#define Pause_Port     GPIOA
#define Next_Key       GPIO_PIN_3
#define Next_Port      GPIOA*/

#define Increase_Key   GPIO_PIN_1
#define Increase_Port  GPIOA
#define Pause_Key      GPIO_PIN_0
#define Pause_Port     GPIOA
#define Decrease_Key   GPIO_PIN_4
#define Decrease_Port  GPIOA

/*************************************** NO CHANGES AFTER THIS *************************************************/

int ispause =0;
int isplaying=1;


# define Start_Byte 0x7E
# define End_Byte   0xEF
# define Version    0xFF
# define Cmd_Len    0x06
# define Feedback   0x00    //If need for Feedback: 0x01,  No Feedback: 0

unsigned char time_to_be_sent;
float time;

void Send_cmdA (uint8_t cmd, uint8_t Parameter1, uint8_t Parameter2)
{
	uint16_t Checksum = Version + Cmd_Len + cmd + Feedback + Parameter1 + Parameter2;
	Checksum = 0-Checksum;

	uint8_t CmdSequence[10] = { Start_Byte, Version, Cmd_Len, cmd, Feedback, Parameter1, Parameter2, (Checksum>>8)&0x00ff, (Checksum&0x00ff), End_Byte};

	HAL_UART_Transmit(DF_UART, CmdSequence, 10, HAL_MAX_DELAY);
}

void DF_PlayFromStart(void)
{
  Send_cmdA(0x03,0x00,0x01);
  HAL_Delay(200);
}


void DF_Init (uint8_t volume)
{
	Send_cmdA(0x3F, 0x00, Source);
	HAL_Delay(200);
	Send_cmdA(0x06, 0x00, volume);
	HAL_Delay(500);
}

void DF_Next (void)
{
	Send_cmdA(0x01, 0x00, 0x00);
	HAL_Delay(200);
}

void DF_Pause (void)
{
	Send_cmdA(0x0E, 0, 0);
	HAL_Delay(200);
}

void DF_Previous (void)
{
	Send_cmdA(0x02, 0, 0);
	HAL_Delay(200);
}

void DF_Playback (void)
{
	Send_cmdA(0x0D, 0, 0);
	HAL_Delay(200);
}

void DF_IncreaseV (void)
{
	Send_cmdA(0x04, 0, 0);
	HAL_Delay(50);
}

void DF_DecreaseV (void)
{
	Send_cmdA(0x05, 0, 0);
	HAL_Delay(50);
}

void DF_SafetyCar(void){
	Send_cmdA(0x03,0x00,0x01);
}

void DF_BestLap(void){
	Send_cmdA(0x03,0x00,0x02);
}

void DF_LastLap(void){
	Send_cmdA(0x03,0x00,0x03);
}

/*uint8_t Get_time(uint8_t *buffer){
	uint8_t temp[4] = {buffer[1], buffer[2], buffer[3], buffer[4]};
	memcpy(&time, &temp, sizeof(temp));
	time_to_be_sent = (unsigned char)time;
	return time_to_be_sent;
}*/














