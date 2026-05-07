#ifndef _FUNC_ERRMES_H_
#define _FUNC_ERRMES_H_

// #include "at32f45x.h"
#include "foc_api.h"

typedef struct {             // 54564   4K
  int8_t ErrNumber;          //,1,2,3,4,5......
  int8_t controller_mode;    //

  uint16_t Udc;              // 0.1V
  uint16_t IBusCurrent;      // 0.001A

  uint16_t UPhaseu;          // u0.1V
  uint16_t UPhasev;          // v0.1V
  uint16_t UPhasew;          // w0.1V

  int16_t I_a;               // u0.001A
  int16_t I_b;               // v0.001A
  int16_t I_c;               // w0.001A

  int16_t Iq_ref;            // Q0001A
  int16_t Iq_fbk;            // Q0.001A
  int16_t Id_ref;            // d0.001A
  int16_t Id_fbk;            // d0.001A

  int16_t velocity_ref;      //,0.1rpm
  int16_t dtheta_mech;       //,0.1rpm

  uint16_t TemMortor;        // 1C
  uint16_t TemBorad;         // 1C

  int32_t position_ref;      //,1
  int32_t real_position;     //,1

  uint32_t RunTime;          // 1s
  uint32_t ServoState;       //
  uint32_t ServoErrFlag;     //
} ErrMessgeStruct;

uint32_t GetErrMessgeAddress(uint8_t ErrNumber);
uint8_t GetErrMessgeValue(uint32_t Address, ErrMessgeStruct* ErrMessge);
void ServoHistoryErrMessgeSolve(void);
uint8_t UpdataErrMessgeValue(uint8_t ErrNumber, ControllerStruct* controller, ErrMessgeStruct* ErrMessge);
uint8_t WriteErrMessgeValue(uint32_t Address, uint8_t ErrNumber, ErrMessgeStruct* ErrMessge);
uint8_t ErrHistoryDisplay(void);
uint8_t ShowErrData(ErrMessgeStruct* ErrMessge);
ErrMessgeStruct get_error_message_by_index(uint8_t index);
uint8_t InitErrHistoryFromFlash(void);

#endif
