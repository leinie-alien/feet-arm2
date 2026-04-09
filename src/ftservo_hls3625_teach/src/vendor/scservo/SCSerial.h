#ifndef FTSERVO_HLS3625_TEACH__SCSERIAL_H_
#define FTSERVO_HLS3625_TEACH__SCSERIAL_H_

#include <termios.h>

#include "SCS.h"

class SCSerial : public SCS
{
public:
  SCSerial();
  explicit SCSerial(u8 end);
  SCSerial(u8 end, u8 level);
  virtual ~SCSerial();

  bool begin(int baud_rate, const char * serial_port);
  void end();

public:
  unsigned long IOTimeOut;
  int Err;

protected:
  int writeSCS(const unsigned char * data, int len) override;
  int readSCS(unsigned char * data, int len) override;
  int writeSCS(unsigned char byte) override;
  void rFlushSCS() override;
  void wFlushSCS() override;

private:
  int fd_;
  struct termios original_options_;
  struct termios current_options_;
  unsigned char tx_buffer_[255];
  int tx_buffer_len_;
};

#endif  // FTSERVO_HLS3625_TEACH__SCSERIAL_H_
