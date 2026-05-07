/**
 * @file    ifly_test.h
 * @brief   故障测试代码
 * @author  xlding15
 * @date    2025-11-19
 * @version 1.0
 */
#ifndef _IFLY_TEST_H_
#define _IFLY_TEST_H_

#include <stdint.h>

// void Test_task_run(void);
void Test_log_print(void);
void TestBusOverUdc(void);
void TestBusLowUdc(void);
void TestBoardOverTem(void);
void TestLockedRotorCurrent(void);
void TestLockedRotorSpeed(void);
void TestLockedRotorPosit(void);
void TestSpeedOffset(void);
void TestMotorVelOver(void);
void TestPostionOver(void);
void TestIbusCurrentOver(void);
void TestUVWCurrentOver(void);
void TestCurrentLoopBandwidth(void);
void TestSpeedLoopBandwidth(void);
void TestFluxIdent(void);
void TestInertiaIdent(void);
void TestPositionLoopBandwidth(void);

#endif
