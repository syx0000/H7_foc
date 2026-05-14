/**
 * @file    ifly_fault.h
 * @brief
 * @author  zcliu15
 * @date    2025-07-07
 * @version 1.0
 */

#ifndef _IFLY_FAULT_H_
#define _IFLY_FAULT_H_

#include <stdint.h>
//#include "foc_api.h"

// #define OVER_MOTOR_TEMP_ERR 70//
#define RELEASE_MOTOR_TEMP 65      //
#define HIGH_MOTOR_TEMP_WARN 50    //
#define FILTER_TIME 10             // 100ms

#define DC_BUS_VOL_24 0            // 24V
#if DC_BUS_VOL_24
#define DC_BUS_REF 240             // 24v0.1V
#define OVER_DC_BUS_VOL 276L       // +15%
#define LOW_DC_BUS_VOL 192L        // -20%
#endif

#define DC_BUS_VOL_48 1            // 48V
#if DC_BUS_VOL_48
#define DC_BUS_REF 480             // 48v0.1V
// #define OVER_DC_BUS_VOL 552L// +15%
// #define LOW_DC_BUS_VOL 348L// -20%
#endif
#define MIN_PHASE_VOLTAGE 5L             // 0.5v
#define POSITION_REACH_TIME 100          // 100ms

#define BRAKE_CLOSE_BEFOR_TIME 206848    // 2rpm

/* Servo Error Codes */
#define SERVO_ERR_DRIVER_NFAULT 0x8000
#define SERVO_ERR_ENCODER 0x8010
#define SERVO_ERR_PHASE_CUR_SAMPLE 0x8020
#define SERVO_ERR_ZERO_POINT 0x8030
#define SERVO_ERR_PHASE_ORDER 0x8040
#define SERVO_ERR_EEPROM_FAIL 0x8045
#define SERVO_ERR_STO_ACTIVED 0x804A
#define SERVO_ERR_LOCKED_ROTOR 0x8050
#define SERVO_ERR_HIGH_MOTOR_TEMP 0x8060
#define SERVO_ERR_HIGH_BOARD_TEMP 0x8070
#define SERVO_ERR_OVER_BUS_VOLT 0x8080
#define SERVO_ERR_LOW_BUS_VOLT 0x8090
#define SERVO_ERR_OVER_BUS_CURRENT 0x80A0
#define SERVO_ERR_COMMUNICATE 0x80B0
#define SERVO_ERR_OVER_SPEED 0x80C0
#define SERVO_ERR_OVER_POSITION 0x80D0
#define SERVO_ERR_PHASE_U_VOLT 0x80E0
#define SERVO_ERR_PHASE_V_VOLT 0x80F0
#define SERVO_ERR_PHASE_W_VOLT 0x8100
#define SERVO_ERR_POS_OFFSET 0x8110
#define SERVO_ERR_SPEED_OFFSET 0x8120
#define SERVO_ERR_SPEED_OVER 0x8130
#define SERVO_ERR_CURRENT_OFFSET 0x8140
#define SERVO_ERR_FLASH_READ 0x8150

#define DEFAULT_POS_ERR_VALUE (1024 * 58)       // 单位1/1024°,临时修改，待优化跟谁后确定
#define DEFAULT_POS_CHECKK_TIME 48

#define DEFAULT_SPD_ERR_VALUE (1024 * 80)       // 单位1/1024°
#define DEFAULT_SPD_CHECKK_TIME 32

#define DEFAULT_CURRENT_ERR_VALUE (1024 * 2)    // 单位1/1024°
#define DEFAULT_CURRENT_CHECKK_TIME 32

typedef struct {
  int8_t board_temp;             // 1C
  int8_t motor_temp;             // 1C
  uint32_t UPhaseu;              // u0.1V
  uint32_t VPhaseu;              // v0.1V
  uint32_t WPhaseu;              // w0.1V
  uint32_t Udc;                  // 0.1V
  uint16_t IBusCurrent;          // 0.001A
  uint16_t IBusCurrentFilter;    // 0.001A
} ifly_Err_Pro_Type;

/* Use FlagStatus from STM32 HAL instead of redefining RESET/SET */
#ifndef flag_status
typedef enum { FLAG_RESET = 0, FLAG_SET = 1 } flag_status;
#endif

uint8_t CheckAndHandleAllFaultBits(void);
int8_t dcVoltageProFunc(void);
int8_t boradTempProFunc(void);
uint8_t motorProValueUpdate(void);
void motorSpeedOffsetCheck(void);
void motorSpeedOverCheck(void);
void motorPosOffsetCheck(void);
void busOverCurrentCheck(void);    //
void motorCurrentOffsetCheck(void);
void motorOverPosCheck(void);
uint8_t driverChipFaultCheck(void);
int8_t getBoardTemp(void);
int8_t getMotorTemp(void);
uint32_t getUPhaseu(void);
uint32_t getVPhaseu(void);
uint32_t getWPhaseu(void);
uint32_t getUdc(void);
uint16_t getIBusCurrent(void);
uint8_t LockedRotorProFunc(void);
uint8_t motorFaultCheck(void);
uint8_t ClearFaults(uint8_t Fault_clear);    //
void target_reach_check(void);
/* ADC */
int8_t TemperatureInquiry(uint16_t adc_value);

/* NTC */
#define NTC_TABLE_SIZE 96
static const uint16_t NTCTempTable[] = {
    27683,    //	0
    27487,    //	1
    27292,    //	2
    27096,    //	3
    26900,    //	4
    26705,    //	5
    26488,    //	6
    26271,    //	7
    26055,    //	8
    25838,    //	9
    25622,    //	10
    25385,    //	11
    25149,    //	12
    24913,    //	13
    24677,    //	14
    24440,    //	15
    24187,    //	16
    23935,    //	17
    23682,    //	18
    23429,    //	19
    23176,    //	20
    22910,    //	21
    22644,    //	22
    22378,    //	23
    22111,    //	24
    21845,    //	25
    21569,    //	26
    21292,    //	27
    21016,    //	28
    20740,    //	29
    20463,    //	30
    20182,    //	31
    19900,    //	32
    19618,    //	33
    19337,    //	34
    19055,    //	35
    18773,    //	36
    18491,    //	37
    18209,    //	38
    17927,    //	39
    17645,    //	40
    17366,    //	41
    17086,    //	42
    16806,    //	43
    16527,    //	44
    16247,    //	45
    15974,    //	46
    15702,    //	47
    15429,    //	48
    15156,    //	49
    14883,    //	50
    14621,    //	51
    14359,    //	52
    14096,    //	53
    13834,    //	54
    13572,    //	55
    13322,    //	56
    13073,    //	57
    12823,    //	58
    12573,    //	59
    12324,    //	60
    12093,    //	61
    11862,    //	62
    11632,    //	63
    11401,    //	64
    11170,    //	65
    10956,    //	66
    10742,    //	67
    10528,    //	68
    10315,    //	69
    10101,    //	70
    9902,     //	71
    9704,     //	72
    9506,     //	73
    9307,     //	74
    9109,     //	75
    8927,     //	76
    8746,     //	77
    8564,     //	78
    8382,     //	79
    8201,     //	80
    8035,     //	81
    7870,     //	82
    7705,     //	83
    7540,     //	84
    7374,     //	85
    7225,     //	86
    7076,     //	87
    6927,     //	88
    6778,     //	89
    6629,     //	90
    6494,     //	91
    6359,     //	92
    6223,     //	93
    6088,     //	94
    5953,     //	95
};

/*  */
void brake_open(uint8_t block);
void brake_close(void);
uint8_t check_all_flag(void);
void adc_convert(void);
uint32_t get_brake_open_time(void);
uint16_t brake_open_time_to_now(void);
uint8_t brake_close_limit(void);
uint8_t get_warn_status(void);
uint8_t check_fault_flag(void);
uint8_t get_hardword_status(void);
#endif /* _IFLY_FAULT_H_ */
