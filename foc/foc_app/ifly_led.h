/**
 * @file    ifly_led.h
 * @brief   LED
 * @author  dyhuo
 * @date    2025-07-14
 * @version 1.0
 */
#ifndef __WK_LED_H
#define __WK_LED_H
#include "foc_api.h"
#include "ifly_fault.h"
#define DELAY_TIME 500

#define SET_COLOR_BIT(reg, bit) ((reg).bit = 1)
#define CLEAR_COLOR_BIT(reg, bit) ((reg).bit = 0)
#define CHECK_COLOR_BIT(reg, bit) ((reg).bit)
#define LED3 3
#define LED2 2
#define LED1 1
typedef struct {
  unsigned int EMERGENCY_FAULT : 1;        //
  unsigned int ARNING_EVENT : 1;           //
  unsigned int COMMUNICATION_ERROR : 1;    //
  unsigned int NORMAL_OPERATION : 1;       //
  unsigned int RESERVED_BITS : 4;          //

} LED_STATUSBits;

void fault_hint_led(void);
void ledLightCode_Set1(uint8_t led_num);
void ledLightCode_Set2(uint8_t led_num);
void ledLightCode_Set3(uint8_t led_num);
void ledLightCode_Set4(uint8_t led_num);
void ledLightCode_Set5(uint8_t led_num);
void running_hint_led(void);
void ledLightCode_Set6(uint8_t led_num);
uint8_t ecat_status_show(void);
uint8_t ecat_fault_judge(void);
void ecat_hint_led(void);
void test_ecat_status(void);
void test_ecat_fault(void);

#endif
