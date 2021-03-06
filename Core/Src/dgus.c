//
// Created by 94997 on 2021/4/20.
//
# include "stm32f3xx_hal.h"
#include "stdio.h"
# include "stm32f3xx_hal.h"
#include "string.h"
extern UART_HandleTypeDef huart1;
#define DGUS_UART &huart1

#define header1   0x5A
#define header2   0xA5
#define Button1_KEY GPIO_PIN_6
#define Button1_PORT GPIOC

uint8_t pTime = 0x00;
unsigned char time_to_be_sent;
float time;

void Send_cmd ( uint8_t length, uint8_t instruction, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3){
	uint8_t CmdSequence[8] = {header1, header2, length, instruction, data0, data1, data2, data3};
    HAL_UART_Transmit(DGUS_UART, CmdSequence, 8, HAL_MAX_DELAY);
}

void Send_cmd2 ( uint8_t length, uint8_t instruction, uint8_t add0, uint8_t add1, uint8_t data ){
	uint8_t CmdSequence[8] = {header1, header2, length, instruction, add0, add1, data};
    HAL_UART_Transmit(DGUS_UART, CmdSequence, 8, HAL_MAX_DELAY);
}

void Send_throttle_cmd ( uint8_t X1, uint8_t X2){
	uint8_t CmdSequence[20] = {header1, header2, 0x11, 0x82, 0x18, 0x00, 0x00, 0x04,0x00,0x01,
			0x00,0xFE,0x00,0xD8,X1,X2,0x01,0x06,0xFF,0xFF};
    HAL_UART_Transmit(DGUS_UART, CmdSequence, 22, HAL_MAX_DELAY);
}

void Send_brake_cmd ( uint8_t X2){
	uint8_t CmdSequence[20] = {header1, header2, 0x11, 0x82, 0x15, 0x00, 0x00, 0x04,0x00,0x01,
			0x00,X2,0x00,0xD8,0x00,0xEC,0x01,0x06,0xFF,0xFF};
    HAL_UART_Transmit(DGUS_UART, CmdSequence, 22, HAL_MAX_DELAY);
}

void Send_graph_cmd ( uint8_t length, uint8_t instruction, uint8_t VP1,uint8_t VP2,  uint8_t cmd1, uint8_t cmd2, uint8_t cmd3,
		uint8_t cmd4,uint8_t addr1,uint8_t addr2,uint8_t X1,uint8_t X2,uint8_t Y1,uint8_t Y2,
		uint8_t X3,uint8_t X4,uint8_t Y3,uint8_t Y4,uint8_t X5,uint8_t X6,uint8_t Y5,uint8_t Y6){
	uint8_t CmdSequence[24] = {header1, header2, length, instruction, VP1, VP2, cmd1, cmd2,cmd3,cmd4,addr1,addr2,
			X1,X2,Y1,Y2,X3,X4,Y3,Y4,X5,X6,Y5,Y6};
	HAL_UART_Transmit(DGUS_UART, CmdSequence, 24, HAL_MAX_DELAY);
}

void Send_brake(uint8_t brake){
	uint8_t brakePosition= 208 - 2*brake;
	Send_graph_cmd(0x15, 0x82, 0x00,0x00,0x00,0x06,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
	Send_brake_cmd(brakePosition);
}

void Send_throttle(uint8_t throttle){
	uint8_t *throttlePosition = (uint8_t *)254 + throttle + throttle;
	Send_throttle_cmd(throttlePosition[0], throttlePosition[1]);
}


uint8_t Get_time(uint8_t *buffer){
	uint8_t temp[4] = {buffer[1], buffer[2], buffer[3], buffer[4]};
	memcpy(&time, &temp, sizeof(temp));
	time_to_be_sent = (unsigned char)time;
	return time_to_be_sent;
}

void Send_position(uint8_t position){
	Send_cmd(0x05, 0x82, 0x17,0x30,0x00,position);
}

void Send_speed(uint8_t speedH, uint8_t speedL){
	Send_cmd(0x05, 0x82, 0x16,0x00, speedH, speedL);
	Send_cmd(0x05, 0x82, 0x12,0x00, speedH, speedL);
}

void Send_totalLap(uint8_t tLap){
	Send_cmd(0x05, 0x82, 0x17,0x00,0x00,tLap);
}

void Send_currentLap(uint8_t cLap){
	Send_cmd(0x05, 0x82, 0x17,0x20, 0x00, cLap);
}

void Send_gear(uint8_t gear){
	Send_cmd(0x05, 0x82, 0x20,0x00, 0x00, gear);
}

void Send_cTime(uint8_t *buffer){
	uint8_t cTime = Get_time((uint8_t *)buffer);
	Send_cmd(0x05, 0x82, 0x21,0x00, 0x00, cTime);
}

void Send_bTime(uint8_t *buffer){
	uint8_t bTime = Get_time((uint8_t *)buffer);
	if (pTime != 0x00 & bTime == 0x00){
		bTime = pTime;
	} else if (bTime > pTime ){
		bTime = pTime;
	}
	Send_cmd(0x05, 0x82, 0x23,0x00, 0x00, bTime);
}

void Send_pTime(uint8_t *buffer){
	pTime = Get_time((uint8_t *)buffer);
	Send_cmd(0x05, 0x82, 0x22,0x00, 0x00, pTime);
}

void Send_light(uint8_t light){
	uint8_t temp1;
	uint8_t temp2;
	uint8_t temp3;
	if(light < 34){
		temp1 = (light/33)*4 + (light%23)/22*2 + (light%12)/11;
		temp2 = 0;
		temp3 = 0;
	}else if (light < 67){
		temp1 = 7;
		temp2 = ((light-33)/33)*4 + ((light-33)%23)/22*2 + ((light-33)%12)/11;
		temp3 = 0;
	}else{
		temp1 = 7;
		temp2 = 7;
		temp3 = ((light-66)/33)*4 + ((light-66)%23)/22*2 + ((light-66)%12)/11;
	}
	Send_cmd(0x05, 0x82, 0x12,0x50, 0x00, temp1);
	Send_cmd(0x05, 0x82, 0x13,0x50, 0x00, temp2);
	Send_cmd(0x05, 0x82, 0x14,0x50, 0x00, temp3);
}



/*void Dummy_test1 (void){
    Send_cmd(0x05, 0x82, 0x16,0x00,0x00,0xDF);//223kmh
    Send_cmd(0x05, 0x82, 0x17,0x00,0x00,0x18);
    Send_cmd(0x05, 0x82, 0x17,0x10,0x00,0x38);
    Send_cmd(0x05, 0x82, 0x17,0x20,0x00,0x08);
    Send_cmd(0x05, 0x82, 0x17,0x30,0x00,0x17);
    HAL_Delay(100);
}
void Dummy_test2 (void){
    Send_cmd(0x05, 0x82, 0x16,0x00,0x00,0xC8);//200kmh
    Send_cmd(0x05, 0x82, 0x17,0x20,0x00,0x09);
    Send_cmd(0x05, 0x82, 0x17,0x30,0x00,0x18);
    HAL_Delay(100);
}

void Dummy_button_test(void){
	if (HAL_GPIO_ReadPin(Button1_PORT, Button1_KEY))
	        {
	            while (HAL_GPIO_ReadPin(Button1_PORT, Button1_KEY));
	            Dummy_test2();
	        }
	else
		Dummy_test1();
}
// dummy_test 3 for testing the status of pointer
void Dummy_test3 (void){
    Send_cmd(0x05, 0x82, 0x12,0x00,0x00,0xDC);
    HAL_Delay(100);
}
// dummy_test 4 for testing the status of pointer
void Dummy_test4 (void){
    Send_cmd(0x05, 0x82, 0x12,0x00,0x00,0x00);
    HAL_Delay(100);
}
// dummy_test 5 for testing three green lights on
void Dummy_test5 (void){
    Send_cmd(0x06, 0x82, 0x12,0x50,0x00,0x07);
    Send_graph_cmd(0x15, 0x82, 0x15,0x00,0x00,0x06,0x00,0x02,0x00,0x00,0x00,0xF0,0x00,0xDC,0x01,0x7B,0x01,0x06,0x00,0x51,0x00,0xDF);
    //Send_graph_cmd(0x15, 0x82, 0x18,0x00,0x00,0x06,0x00,0x02,0x00,0x00,0x00,0x81,0x00,0xDF,0x00,0xEB,0x01,0x09,0x00,0xF1,0x00,0xDB);
    //Send_graph_cmd(0x15, 0x82, 0x15,0x00,0x00,0x06,0x00,0x02,0x00,0x00,0x00,0xF0,0x00,0xDC,0x01,0x7B,0x01,0x06,0x00,0x51,0x00,0xDF);
    HAL_Delay(100);
}
// dummy_test 6 for testing three green lights off
void Dummy_test6 (void){
    Send_cmd(0x06, 0x82, 0x12,0x50,0x00,0x00);
	Send_graph_cmd(0x15, 0x82, 0x15,0x00,0x00,0x06,0x00,0x02,0x00,0x00,0x00,0xF0,0x00,0xDC,0x01,0x36,0x01,0x06,0x00,0x51,0x00,0xDF);
	//Send_graph_cmd(0x15, 0x82, 0x18,0x00,0x00,0x06,0x00,0x02,0x00,0x00,0x00,0x81,0x00,0xDF,0x00,0x9E,0x01,0x09,0x00,0xF1,0x00,0xDB);
	HAL_Delay(100);
}
// dummy_test 7 for testing basic graph display
void Dummy_test7 (void){

	//Send_graph_cmd(0x15, 0x82, 0x15,0x00,0x00,0x06,0x00,0x02,0x00,0x00,0x00,0xF0,0x00,0xDC,0x01,0x7B,0x01,0x06,0x00,0x51,0x00,0xDF);
	Send_graph_cmd1(0x11, 0x82, 0x15,0x00,0x00,0x04,0x00,0x01,0x00,0x4f,0x00,0xE0,0x00,0xeb,0x01,0x01,0x00,0x00);
    HAL_Delay(100);
}
// dummy_test 8 for testing basic graph display
void Dummy_test8 (void){
	//Send_graph_cmd(0x15, 0x82, 0x15,0x00,0x00,0x06,0x00,0x02,0x00,0x00,0x00,0xF0,0x00,0xDC,0x01,0x36,0x01,0x06,0x00,0x51,0x00,0xDF);

	Send_graph_cmd1(0x11, 0x82, 0x15,0x00,0x00,0x04,0x00,0x01,0x00,0x4f,0x00,0xE0,0x00,0x9a,0x01,0x01,0x00,0x00);

    HAL_Delay(100);
}


// dummy_test for testing two orange lights on
void Dummy_test_orange_light_on (void){
    Send_cmd(0x05, 0x82, 0x13,0x50,0x00,0x03);
    HAL_Delay(100);
}
// dummy_test for testing two orange lights off
void Dummy_test_orange_light_off (void){
    Send_cmd(0x05, 0x82, 0x13,0x50,0x00,0x00);
    HAL_Delay(100);
}
// dummy_test for testing one red lights on
void Dummy_test_red_light_on (void){
    Send_cmd(0x05, 0x82, 0x14,0x50,0x00,0x01);
    HAL_Delay(100);
}
// dummy_test for testing one red lights off
void Dummy_test_red_light_off (void){
    Send_cmd(0x05, 0x82, 0x14,0x50,0x00,0x00);
    HAL_Delay(100);
}

// dummy_test 7 for testing basic graph display
void Dummy_test_bar1 (void){
	//Send_graph_cmd(0x15, 0x82, 0x18,0x00,0x00,0x06,0x00,0x02,0x00,0x00,0x00,0x81,0x00,0xDF,0x00,0xEB,0x01,0x09,0x00,0xF1,0x00,0xDB);
	Send_graph_cmd1(0x11, 0x82, 0x18,0x00,0x00,0x04,0x00,0x01,0x00,0xF1,0x00,0xDB,0x01,0x8B,0x01,0x05,0x00,0x00);
	HAL_Delay(100);

}
// dummy_test 8 for testing basic graph display
void Dummy_test_bar2 (void){
	//Send_graph_cmd(0x15, 0x82, 0x18,0x00,0x00,0x06,0x00,0x02,0x00,0x00,0x00,0x81,0x00,0xDF,0x00,0x9E,0x01,0x09,0x00,0xF1,0x00,0xDB);
	Send_graph_cmd1(0x11, 0x82, 0x18,0x00,0x00,0x04,0x00,0x01,0x00,0xF1,0x00,0xDB,0x01,0x3B,0x01,0x05,0x00,0x00);
	HAL_Delay(100);
}

// dummy_test for testing one red lights off
void Dummy_test_slider_50 (void){
    Send_cmd(0x05, 0x82, 0x20,0x00,0x00,0x32);
    HAL_Delay(100);
}
// dummy_test for testing one red lights off
void Dummy_test_slider_100 (void){
    Send_cmd(0x05, 0x82, 0x20,0x00,0x00,0x64);
    HAL_Delay(100);
}*/
