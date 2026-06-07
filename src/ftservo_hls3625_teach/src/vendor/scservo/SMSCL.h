#ifndef FTSERVO_HLS3625_TEACH__SMSCL_H_
#define FTSERVO_HLS3625_TEACH__SMSCL_H_

#include "SCSerial.h"

#define SMSCL_TORQUE_ENABLE 40
#define SMSCL_ACC 41
#define SMSCL_GOAL_POSITION_L 42
#define SMSCL_GOAL_SPEED_L 46

class SMSCL : public SCSerial
{
public:
  SMSCL();
  explicit SMSCL(u8 end);
  SMSCL(u8 end, u8 level);

  int WritePosEx(u8 id, s16 position, u16 speed, u8 acc = 0);
  int EnableTorque(u8 id, u8 enable);
};

#endif  // FTSERVO_HLS3625_TEACH__SMSCL_H_
