/**
 * @file    func_errMes.c
 * @brief
 * @author  cjwang14
 * @date    2025-08-29
 * @version 1.0
 */

#include "func_errMes.h"
// #include "FreeRTOS.h"  /* FreeRTOS removed */
#include "func_errMes.h"
#include "ifly_fault.h"
// #include "task.h"  /\* FreeRTOS removed \*/

extern ControllerStruct controller_eyou;
ErrMessgeStruct ErrMessge[ERRMESSGECOUNT] = {0};

void ServoHistoryErrMessgeSolve(void) {
}

uint32_t GetErrMessgeAddress(uint8_t ErrNumber) {
    return 0;
}

uint8_t GetErrMessgeValue(uint32_t Address, ErrMessgeStruct* ErrMessge) {
    return 0;
}

uint8_t UpdataErrMessgeValue(uint8_t ErrNumber, ControllerStruct* controller, ErrMessgeStruct* ErrMessge) {
    return 0;
}

uint8_t WriteErrMessgeValue(uint32_t Address, uint8_t ErrNumber, ErrMessgeStruct* ErrMessge) {
    return 0;
}

uint8_t ErrHistoryDisplay(void) {
    return 0;
}

uint8_t ShowErrData(ErrMessgeStruct* ErrMessge) {
    return 0;
}

ErrMessgeStruct get_error_message_by_index(uint8_t index) {
    ErrMessgeStruct empty_struct = {0};
    return empty_struct;
}

uint8_t InitErrHistoryFromFlash(void) {
    return 0;
}
