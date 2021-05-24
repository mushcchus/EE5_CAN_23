//
// Created by 94997 on 2021/4/20.
//

#ifndef UNTITLED_DGUS_H
#define UNTITLED_DGUS_H

//#define KEY0 GPIO_ReadInputDataBit(GPIOC,GPIO_PIN_6)
void Send_cmd ( uint8_t length, uint8_t instruction, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3);
void Send_graph_cmd ( uint8_t length, uint8_t instruction, uint8_t VP1,uint8_t VP2,  uint8_t cmd1, uint8_t cmd2, uint8_t cmd3,
		uint8_t cmd4,uint8_t addr1,uint8_t addr2,uint8_t X1,uint8_t X2,uint8_t Y1,uint8_t Y2,
		uint8_t X3,uint8_t X4,uint8_t Y3,uint8_t Y4,uint8_t X5,uint8_t X6,uint8_t Y5,uint8_t Y6);
void Send_throttle_cmd ( uint8_t X1, uint8_t X2);
void Send_brake_cmd ( uint8_t X2);
void Dummy_test_slider_50 (void);
void Send_position(uint8_t position);
void Send_speed(uint8_t speedH, uint8_t speedL);
void Send_totalLap(uint8_t tLap);
void Send_currentLap(uint8_t cLap);
void Send_light(uint8_t light);
void Send_gear(uint8_t gear);
void Send_cTime(uint8_t *buffer);
void Send_bTime(uint8_t *buffer);
void Send_pTime(uint8_t *buffer);
void Send_brake(uint8_t brake);
void Send_throttle(uint8_t brake);
uint8_t Get_time(uint8_t *buffer);
/*void Dummy_test1 (void);
void Dummy_test2 (void);
void Dummy_test3 (void);
void Dummy_test4 (void);
void Dummy_test5 (void);
void Dummy_test6 (void);
void Dummy_test7 (void);
void Dummy_test8 (void);
void Send_graph_cmd ( uint8_t length, uint8_t instruction, uint8_t VP1,uint8_t VP2,  uint8_t cmd1, uint8_t cmd2, uint8_t cmd3,
		uint8_t cmd4,uint8_t addr1,uint8_t addr2,uint8_t X1,uint8_t X2,uint8_t Y1,uint8_t Y2,
		uint8_t X3,uint8_t X4,uint8_t Y3,uint8_t Y4,uint8_t X5,uint8_t X6,uint8_t Y5,uint8_t Y6);
void Dummy_button_test(void);
void Dummy_test_orange_light_on (void);
void Dummy_test_orange_light_off (void);
void Dummy_test_red_light_on (void);
void Dummy_test_red_light_off (void);
void Dummy_test_bar1 (void);
void Dummy_test_bar2 (void);
void Send_graph_cmd1 ( uint8_t length, uint8_t instruction, uint8_t VP1,uint8_t VP2,  uint8_t cmd1, uint8_t cmd2, uint8_t cmd3,
		uint8_t cmd4,uint8_t addr1,uint8_t X2,uint8_t Y1,uint8_t Y2,
		uint8_t X3,uint8_t X4,uint8_t Y3,uint8_t Y4,uint8_t X5,uint8_t X6);
void Dummy_test_slider_50 (void);
void Dummy_test_slider_100 (void);*/
#endif //UNTITLED_DGUS_H
