/**
 * @file    ifly_fault_api.c
 * @brief
 * @author  dyhuo
 * @date    2025-08-18
 * @version 1.0
 */

#include "ifly_fault_api.h"
// #include "cia402appl.h"  /* EtherCAT removed */
#include "encoder.h"
#include "foc_api.h"
#include "foc_data.h"
#include "ifly_fault.h"

// extern TCiA402Axis LocalAxes[MAX_AXES];  /* EtherCAT removed */
extern ifly_Err_Pro_Type motorProValue;
extern str_FILTER1 udcFilter;
Portection_Value Threshold = {
    .LowUdc            = 348,
    .OverUdc           = 552,
    .TemBorad          = 85,
    .TemBoradWarn      = 75,
    .velocity_Limit    = 4136960,
    .velocity_coe      = 1,
    .PositionErr       = 2048 * 20,
    .PositionReachTime = 256,
    .BlockTorque       = 1024 * 13,
    .OverCurrentTime   = 5,
    .OverCurrent       = 28672,
    .BlockTime         = 30,
    .BlockSpeed        = 103424,
    .UVWCurrentLimit   = 6860,
};
int8_t brake_control_flag;

int16_t motor_brake_control(int16_t brake_control_flag) {
    return 0;
}

uint8_t get_motor_brake_state() {
    return 0;
}

uint8_t set_motor_temp_limit(int8_t MTempLimit) {
    return 0;
}

uint8_t set_borad_temp_warn(int8_t BTempWarn) {
    return 0;
}

uint8_t set_borad_temp_limit(int8_t BTempLimit) {
    return 0;
}

uint16_t set_bus_over_current_limit(uint16_t IBusCurrent) {
    return 0;
}

uint8_t set_bus_over_current_limit_time(uint8_t IbusCurrentTime) {
    return 0;
}

uint32_t get_Over_Velocity_limit(void) {
    return 0;
}

uint32_t set_Over_Velocity_limit(uint32_t OverVelocityLimt) {
    return 0;
}

uint8_t set_Over_Velocity_coefficient(uint8_t OverVelocityCoefficient) {
    return 0;
}

uint8_t set_motor_temp_limit_time(uint8_t MTemptLimit) {
    return 0;
}

uint32_t set_Under_Udc_limit(uint32_t LowUdcLimit) {
    return 0;
}

uint8_t set_Low_Udc_limit_time(uint8_t LowUdcTimeLimit) {
    return 0;
}

uint32_t set_Over_Udc_limit(uint32_t OverUdcLimit) {
    return 0;
}

uint8_t set_Over_Udc_limit_time(uint8_t OverUdcTimeLimit) {
    return 0;
}

uint16_t set_Block_Torque_limit(uint16_t BlockTorqueLimt) {
    return 0;
}

uint16_t set_Block_Time_limit(uint16_t BlockTimeLimt) {
    return 0;
}

uint32_t set_Block_Speed_limit(uint32_t BlockSpeedLimt) {
    return 0;
}

uint16_t set_Block_Position_limit(uint16_t BlockPositionLimt) {
    return 0;
}

uint16_t set_Over_Current_limit(uint16_t OverCurrentLimt) {
    return 0;
}

uint32_t get_Encoder_OutValue(void) {
    return 0;
}

uint8_t set_elecoffest_action(int8_t Execute) {
    return 0;
}

uint16_t set_maximal_following_limit(uint16_t ErrPositionLimt) {
    return 0;
}

uint32_t set_Position_window_value(uint32_t Position_window) {
    return 0;
}

uint16_t set_Position_window_time(uint16_t Position_window_time) {
    return 0;
}

void IBusCurrent_Filter(ifly_Err_Pro_Type *Err_Pro) {
}

void CurrentFilterGoing(ControllerStruct *Controller_filt) {
}

void fault_api_test() {
}

void Dual_Encoder_Fault_Detection(ControllerStruct* controller) {
}
