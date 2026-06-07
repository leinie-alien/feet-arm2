#ifndef FTSERVO_HLS3625_TEACH__SCS_H_
#define FTSERVO_HLS3625_TEACH__SCS_H_

#include "INST.h"

class SCS
{
public:
  SCS();
  explicit SCS(u8 end);
  SCS(u8 end, u8 level);

  int genWrite(u8 id, u8 mem_addr, const u8 * data, u8 len);
  int writeByte(u8 id, u8 mem_addr, u8 value);
  int readWord(u8 id, u8 mem_addr);
  int Ping(u8 id);

public:
  u8 Level;
  u8 End;
  u8 Error;

protected:
  virtual int writeSCS(const unsigned char * data, int len) = 0;
  virtual int readSCS(unsigned char * data, int len) = 0;
  virtual int writeSCS(unsigned char byte) = 0;
  virtual void rFlushSCS() = 0;
  virtual void wFlushSCS() = 0;

  void Host2SCS(u8 * data_low, u8 * data_high, u16 data) const;
  u16 SCS2Host(u8 data_low, u8 data_high) const;

private:
  void writeBuf(u8 id, u8 mem_addr, const u8 * data, u8 len, u8 function);
  int Read(u8 id, u8 mem_addr, u8 * data, u8 len);
  int Ack(u8 id);
};

#endif  // FTSERVO_HLS3625_TEACH__SCS_H_
