/**
 * @file    ifly_fault_api.c
 * @brief
 * @author  dyhuo
 * @date    2025-08-18
 * @version 1.0
 */

#ifndef _IFLY_FAULT_IPA_H_
#define _IFLY_FAULT_IPA_H_

#include <stdio.h>
// #include "foc_api.h"
#include "foc_controller.h"
#include "func_filter.h"
#include "ifly_fault.h"

#define GET_BRAKE_IO_VALUE 1                     // tmr_channel_value_get(TMR1, TMR_SELECT_CHANNEL_1)

#define BOARD_MAX_TEM_WARN 60                    //
#define BOARD_MIN_TEM_WARN 25                    //
#define BOARD_LIMIT_TEM_WARN 50                  //
#define BOARD_MAX_TEM 95                         //
#define BOARD_MIN_TEM 25
#define BOARD_LIMIT_TEM 70
#define MOTOR_MAX_SPEED 1024000                  // 40rpm × 25 × 1024
#define MOTOR_LIMIT_SPEED 768000                 // 30rpm × 25 × 1024
#define MOTOR_MIN_SPEED 640000                   // 25rpm × 25 × 1024
#define MOTOR_MAX_DEV_COE 10                     //
#define MOTOR_MIN_DEV_COE 1                      //
#define MOTOR_LIMIT_DEV_SPEED 2                  //
#define MOTOR_UNDERVOLTAGE_MAX 480               //
#define MOTOR_UNDERVOLTAGE_MIN 200               //
#define MOTOR_UNDERVOLTAGE_LIMIT 348             //
#define MOTOR_OVERVOLTAGE_MAX 600                //
#define MOTOR_OVERVOLTAGE_MIN 481                //
#define MOTOR_OVERVOLTAGE_LIMIT 552              //
#define MOTOR_BLOCK_SPEED 517120                 // 5rpm
#define MOTOR_DEFAULT_BLOCK_SPEED 103424         // 1rpm
#define MOTOR_BLOCK_POSITION 1024 * 10           //
#define MOTOR_DEFAULT_BLOCK_POSITION 1024 * 2    //
#define MOTOR_POSITION_DEV_MAX 51200             // 50
#define MOTOR_POSITION_DEV_MIN 20480             //
#define MOTOR_POSITION_DEV_LIMIT 40960           //
#define MOTOR_IBUS_MAX 9728                      //
#define MOTOR_IBUS_MIN 5120                      //
#define MOTOR_IBUS_LIMIT 6860                    //
#define MOTOR_IBUS_LIMIT_TIME 5                  //
#define MOTOR_IBUS_TIMR_MAX 20                   // 2
#define MOTOR_IBUS_TIMR_MIN 1                    // 0.1
#define MOTOR_BLOCK_TORQUE_MAX 15360             // 15A
#define MOTOR_BLOCK_TORQUE_MIN 1024              // 1A
#define MOTOR_BLOCK_TORQUE_LIMIT 13312           // 13A
#define MOTOR_BLOCK_TIME_MIN 10                  // 1
#define MOTOR_BLOCK_TIME_MAX 50                  // 5
#define MOTOR_BLOCK_TIME_LIMIT 30                // 3
#define MOTOR_POSITION_DEV_TIM_MAX 512           // 1
#define MOTOR_POSITION_DEV_TIM_MIN 51            // 0.1
#define MOTOR_POSITION_DEV_TIM_DEF 256           // 1
#define MOTOR_POSITION_REACH_DEF 256             //
#define MOTOR_POSITION_REACH_MAX 2048            //
#define MOTOR_IUVW_MAX 9728
#define MOTOR_IUVW_MIN 5120
#define MOTOR_IUVW_LIMIT 6860
typedef struct {
  uint32_t SampleUdc;           //
  uint32_t SamplePhaseU;        // U
  uint32_t SamplePhaseV;        // V
  uint32_t SamplePhaseW;        // W
  uint32_t SampleBusCurrent;    //
  uint32_t SampleTempMotor;     //
  uint32_t SampleTempBoard;     //

  uint32_t g_Udc;               //
  uint32_t g_PhaseU;            // U,
  uint32_t g_PhaseV;            // V,
  uint32_t g_PhaseW;            // W,
  uint32_t g_BusCurrent;        //,
  uint32_t g_Mot;               //
  uint32_t g_Temp;              //

  uint16_t Udc;                 // 0.1V
  uint16_t UPhaseu;             // u0.1V
  uint16_t UPhasev;             // v0.1V
  uint16_t UPhasew;             // w0.1V
  uint16_t IBusCurrent;         // 0.001A
  uint16_t TemMortor;           // 1C
  uint16_t TemBorad;            // 1C

} str_MonitADC;

#pragma pack(push, 1)            // 1
typedef struct {
  uint8_t Flag;                  //
  uint16_t LowUdc;               //
  uint16_t OverUdc;              //
  uint16_t LowUdcTime;           //
  uint16_t OverUdcTime;          //
  uint8_t OverCurrentTime;       //
  uint16_t OverCurrent;          //
  uint16_t BlockTorque;          //
  uint16_t BlockTime;            //
  uint32_t BlockSpeed;           //
  uint16_t BlockPosition;        //

  int32_t velocity_Limit;        //
  uint8_t velocity_coe;          //

  uint16_t TemMortor;            //
  uint16_t TemMortorTime;        //
  uint8_t TemBoradWarn;          //
  uint16_t TemBorad;             //
  uint16_t TemBoradTime;         //
  uint16_t PosOffset;            //
  uint8_t PosOffsetnum;          //

  uint16_t PositionErr;          //
  int32_t real_position;         //,1
  int32_t PositionReach;         //
  uint16_t PositionReachTime;    //
  uint16_t UVWCurrentLimit;      // uvw

  uint32_t Crc;                  // crc
  // uint16_t PositionCount;//

} Portection_Value;
#pragma pack(pop)    //
extern int8_t brake_control_flag;

void get_MonitAdcValue(void);
int16_t motor_brake_control(int16_t brake_control_flag);
uint8_t get_motor_brake_state(void);
uint8_t set_borad_temp_warn(int8_t BTempWarn);
uint8_t set_motor_temp_limit(int8_t MTempLimit);
uint8_t set_borad_temp_limit(int8_t BTempLimit);
uint8_t set_motor_temp_limit_time(uint8_t MTemptLimit);
uint32_t get_Over_Velocity_limit(void);
uint32_t set_Over_Velocity_limit(uint32_t OverVelocityLimt);
uint8_t set_Over_Velocity_coefficient(uint8_t OverVelocityCoefficient);
uint32_t set_Under_Udc_limit(uint32_t LowUdcLimit);
uint8_t set_Low_Udc_limit_time(uint8_t LowUdcTimeLimit);
uint32_t set_Over_Udc_limit(uint32_t OverUdcLimit);
uint8_t set_Over_Udc_limit_time(uint8_t OverUdcTimeLimit);
uint16_t set_Block_Torque_limit(uint16_t BlockTorqueLimt);
uint16_t set_Block_Time_limit(uint16_t BlockTimeLimt);
uint32_t set_Block_Speed_limit(uint32_t BlockSpeedLimt);
uint32_t get_Udc_Value(void);
uint32_t get_Encoder_OutValue(void);

uint8_t set_elecoffest_action(int8_t Execute);
uint16_t set_maximal_following_limit(uint16_t ErrPositionLimt);
uint32_t set_Position_window_value(uint32_t Position_window);
uint16_t set_Position_window_time(uint16_t Position_window_time);
uint16_t set_Block_Position_limit(uint16_t BlockPositionLimt);
uint16_t set_bus_over_current_limit(uint16_t IBusCurrent);
uint8_t set_bus_over_current_limit_time(uint8_t IbusCurrentTime);
uint16_t set_Over_Current_limit(uint16_t OverCurrentLimt);    //
uint8_t ClearFaults(uint8_t Fault_clear);
void fault_api_test(void);
void IBusCurrent_Filter(ifly_Err_Pro_Type *Err_Pro);
void CurrentFilterGoing(ControllerStruct *Controller_filt);
void Dual_Encoder_Fault_Detection(ControllerStruct* controller);

#endif
