#ifndef _FOC_DATA_H_
#define _FOC_DATA_H_
/**
 ******************************************************************************
 * File Name          : foc_data.h
 * Description        :
 * author             :
 * data               :
 */

/* Includes ------------------------------------------------------------------*/
#include "foc_controller.h"
#include "ifly_fault_api.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>
/* private includes -------------------------------------------------------------*/

/* exported types -------------------------------------------------------------*/

/* exported constants --------------------------------------------------------*/
#define BREAK_TIME_MIN 0        // 最小抱闸延时时间
#define BREAK_TIME_MAX 2000     // 最大抱闸延时时间
#define BREAK_TIME_LIMIT 500    // 默认抱闸延时时间

/* exported macro ------------------------------------------------------------*/

void ResetControlData(ControllerStruct* controller);
uint8_t InitFlashData(ControllerStruct* controller);
uint8_t PhaseCurrentOffsetEstimate(ControllerStruct* controller);
uint8_t ReadDataFromAddress(ControllerStruct* controller, unsigned int Address);
uint8_t WriteRunDataToFlash(ControllerStruct* controller, unsigned int Address);
void WriteDataToFlash(void);
uint8_t WriteFaultThreshold(Portection_Value* Threshold, uint32_t Address);
uint8_t ReadFaultThreshold(Portection_Value* Threshold_buffer, uint32_t Address);
void InitFaultThreshold(Portection_Value* Threshold_buffer, Portection_Value* Threshold);
uint8_t FlashLimit_Check(Portection_Value* Threshold_buffer);
void get_offest(uint16_t* offset_1, uint16_t* offset_2);
void ElecAngleEstimate(ControllerStruct* controller);
uint8_t DefualtPidValue(FlashSavedData* FlashData);
void ThresholdToObject(Portection_Value* Threshold_buffer);
uint16_t User_Data_Save(uint16_t control);
int8_t Contorl_motor_dir(int8_t dir);
uint16_t set_brake_Time(uint16_t time);
uint8_t MechAngleOffsetEstimata(ControllerStruct* controller, int32_t UserAngle);
uint8_t DefualtArrivedValue(ControllerStruct* controller);
uint8_t DefualtRunDataValue(ControllerStruct* controller);
uint8_t DefualtProteckKeyValue(ControllerStruct* controller);
uint8_t DefualtObjectToFlash(ControllerStruct* controller);
void InitReservedFields(FlashSavedData* FlashData);
int32_t set_velocity_lim(int32_t VelLim);
int32_t get_elecoffest_value(void);
int32_t set_min_pos_lim(int32_t MinPosLim);
int32_t set_max_pos_lim(int32_t MaxPosLim);
int32_t set_current_loop_kp(int32_t Kp);
int32_t get_current_loop_kp(void);
int32_t set_current_loop_ki(int32_t Ki);
int32_t get_current_loop_ki(void);
int32_t set_speed_loop_kp(int32_t Kp);
int32_t get_speed_loop_kp(void);
int32_t set_speed_loop_ki(int32_t Ki);
int32_t get_speed_loop_ki(void);
int32_t set_position_loop_kp(int32_t Kp);
int32_t get_position_loop_kp(void);
int32_t set_position_loop_ki(int32_t Ki);
int32_t get_position_loop_ki(void);
int32_t set_position_loop_kd(int32_t Kd);
int32_t get_position_loop_kd(void);
int16_t set_max_torque(int16_t maxTorruqRef);
int16_t get_max_torque(void);
int16_t get_actual_torque(void);

#endif
