#include "SMSCL.h"

SMSCL::SMSCL()
: SCSerial()
{
  End = 0;
}

SMSCL::SMSCL(u8 end)
: SCSerial(end)
{
}

SMSCL::SMSCL(u8 end, u8 level)
: SCSerial(end, level)
{
}

int SMSCL::WritePosEx(u8 id, s16 position, u16 speed, u8 acc)
{
  if (position < 0) {
    position = static_cast<s16>(-position);
    position = static_cast<s16>(position | (1 << 15));
  }

  u8 buffer[7];
  buffer[0] = acc;
  Host2SCS(buffer + 1, buffer + 2, static_cast<u16>(position));
  Host2SCS(buffer + 3, buffer + 4, 0);
  Host2SCS(buffer + 5, buffer + 6, speed);
  return genWrite(id, SMSCL_ACC, buffer, 7);
}

int SMSCL::EnableTorque(u8 id, u8 enable)
{
  return writeByte(id, SMSCL_TORQUE_ENABLE, enable);
}
