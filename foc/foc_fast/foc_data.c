/**
  **************************************************************************
  * @file     foc_data.c
  * @brief    电机参数及保存相关函数
  * author    cjwang14
  * data      20251230
  *
  **************************************************************************

  **************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "foc_data.h"
// #include "cia402appl.h"  /* EtherCAT removed */
#include "func_subprogram.h"

Portection_Value Threshold_buffer;

extern Portection_Value Threshold;
extern ControllerStruct controller_eyou;
// extern TCiA402Axis LocalAxes[MAX_AXES];

void ResetControlData(ControllerStruct* controller) {
}

uint8_t InitFlashData(ControllerStruct* controller) {
    return 0;
}

uint8_t PhaseCurrentOffsetEstimate(ControllerStruct* controller) {
    return 0;
}

void get_offest(uint16_t* offset_1, uint16_t* offset_2) {
}

uint8_t DefualtPidValue(FlashSavedData* FlashData) {
    return 0;
}

uint8_t ReadDataFromAddress(ControllerStruct* controller, unsigned int Address) {
    return 0;
}

uint8_t WriteRunDataToFlash(ControllerStruct* controller, unsigned int Address) {
    return 0;
}

void WriteDataToFlash(void) {
}

uint8_t WriteFaultThreshold(Portection_Value* Threshold, uint32_t Address) {
    return 0;
}

uint8_t ReadFaultThreshold(Portection_Value* Threshold_buffer, uint32_t Address) {
    return 0;
}

uint8_t FlashLimit_Check(Portection_Value* Threshold_buffer) {
    return 0;
}

void ElecAngleEstimate(ControllerStruct* controller) {
}

uint8_t MechAngleOffsetEstimata(ControllerStruct* controller, int32_t UserAngle) {
    return 0;
}

uint8_t DefualtArrivedValue(ControllerStruct* controller) {
    return 0;
}

uint8_t DefualtRunDataValue(ControllerStruct* controller) {
    return 0;
}

uint8_t DefualtProteckKeyValue(ControllerStruct* controller) {
    return 0;
}

uint8_t DefualtObjectToFlash(ControllerStruct* controller) {
    return 0;
}

void InitReservedFields(FlashSavedData* FlashData) {
}

uint16_t User_Data_Save(uint16_t control) {
    return 0;
}

int8_t Contorl_motor_dir(int8_t dir) {
    return 0;
}

uint16_t set_brake_Time(uint16_t time) {
    return 0;
}

int32_t get_elecoffest_value() {
    return 0;
}

int32_t set_velocity_lim(int32_t VelLim) {
    return 0;
}

int16_t set_max_torque(int16_t maxTorruqRef) {
    return 0;
}

int16_t get_max_torque(void) {
    return 0;
}

int16_t get_actual_torque(void) {
    return 0;
}

int32_t set_min_pos_lim(int32_t MinPosLim) {
    return 0;
}

int32_t set_max_pos_lim(int32_t MaxPosLim) {
    return 0;
}

int32_t set_current_loop_kp(int32_t Kp) {
    return 0;
}

int32_t get_current_loop_kp(void) {
    return 0;
}

int32_t set_current_loop_ki(int32_t Ki) {
    return 0;
}

int32_t get_current_loop_ki(void) {
    return 0;
}

int32_t set_speed_loop_kp(int32_t Kp) {
    return 0;
}

int32_t get_speed_loop_kp(void) {
    return 0;
}

int32_t set_speed_loop_ki(int32_t Ki) {
    return 0;
}

int32_t get_speed_loop_ki(void) {
    return 0;
}

int32_t set_position_loop_kp(int32_t Kp) {
    return 0;
}

int32_t get_position_loop_kp(void) {
    return 0;
}

int32_t set_position_loop_ki(int32_t Ki) {
    return 0;
}

int32_t get_position_loop_ki(void) {
    return 0;
}

int32_t set_position_loop_kd(int32_t Kd) {
    return 0;
}

int32_t get_position_loop_kd(void) {
    return 0;
}
