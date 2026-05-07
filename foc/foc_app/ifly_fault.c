/**
 * @file    ifly_fault.c
 * @brief
 * @author  zcliu15
 * @date    2025-07-07
 * @version 1.0
 */

/* Includes ------------------------------------------------------------------*/
#include "ifly_fault.h"
#include "foc_api.h"
#include "ifly_fault_api.h"
#include "ifly_led.h"
#include <stdio.h>

ifly_Err_Pro_Type motorProValue;
extern ControllerStruct controller_eyou;
extern Portection_Value Threshold;
extern LED_STATUSBits LED_STATUS;
extern void CiA402_LocalError(UINT16 ErrorCode);

uint64_t start_timestamp = 0;

uint8_t CheckAndHandleAllFaultBits(void) {
    return 0;
}

int8_t getBoardTemp(void) {
    return 0;
}

int8_t getMotorTemp(void) {
    return 0;
}

uint32_t getUPhaseu(void) {
    return 0;
}

uint32_t getVPhaseu(void) {
    return 0;
}

uint32_t getWPhaseu(void) {
    return 0;
}

uint16_t getIBusCurrent(void) {
    return 0;
}

uint32_t getUdc(void) {
    return 0;
}

int8_t dcVoltageProFunc() {
    return 0;
}

int8_t boradTempProFunc(void) {
    return 0;
}

void motorPhaseVolCheck(void) {
}

uint8_t LockedRotorProFunc(void) {
    return 0;
}

void busOverCurrentCheck(void) {
}

uint8_t driverChipFaultCheck(void) {
    return 0;
}

int8_t TemperatureInquiry(uint16_t adc_value) {
    return 0;
}

void brake_open(uint8_t block) {
}

void brake_close() {
}

uint8_t brake_close_limit(void) {
    return 0;
}

void adc_convert(void) {
}

uint8_t check_fault_flag(void) {
    return 0;
}

uint8_t get_warn_status(void) {
    return 0;
}

uint8_t get_hardword_status(void) {
    return 0;
}

void motorPosOffsetCheck(void) {
}

void motorSpeedOffsetCheck(void) {
}

void motorCurrentOffsetCheck(void) {
}

void motorOverPosCheck(void) {
}
