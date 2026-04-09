#include "SCS.h"

#include <cstring>

SCS::SCS()
: Level(1), End(0), Error(0)
{
}

SCS::SCS(u8 end)
: Level(1), End(end), Error(0)
{
}

SCS::SCS(u8 end, u8 level)
: Level(level), End(end), Error(0)
{
}

void SCS::Host2SCS(u8 * data_low, u8 * data_high, u16 data) const
{
  if (End) {
    *data_low = static_cast<u8>(data >> 8);
    *data_high = static_cast<u8>(data & 0xff);
    return;
  }

  *data_high = static_cast<u8>(data >> 8);
  *data_low = static_cast<u8>(data & 0xff);
}

u16 SCS::SCS2Host(u8 data_low, u8 data_high) const
{
  u16 data = 0;
  if (End) {
    data = data_low;
    data <<= 8;
    data |= data_high;
    return data;
  }

  data = data_high;
  data <<= 8;
  data |= data_low;
  return data;
}

void SCS::writeBuf(u8 id, u8 mem_addr, const u8 * data, u8 len, u8 function)
{
  u8 message_len = 2;
  u8 header[6];
  header[0] = 0xff;
  header[1] = 0xff;
  header[2] = id;
  header[4] = function;

  u8 checksum = static_cast<u8>(id + function + mem_addr);
  if (data != nullptr) {
    message_len = static_cast<u8>(message_len + len + 1);
    header[3] = message_len;
    header[5] = mem_addr;
    writeSCS(header, 6);
    checksum = static_cast<u8>(checksum + message_len);
    for (u8 i = 0; i < len; ++i) {
      checksum = static_cast<u8>(checksum + data[i]);
    }
    writeSCS(data, len);
  } else {
    header[3] = message_len;
    writeSCS(header, 5);
    checksum = static_cast<u8>(checksum + message_len);
  }
  writeSCS(static_cast<unsigned char>(~checksum));
}

int SCS::genWrite(u8 id, u8 mem_addr, const u8 * data, u8 len)
{
  rFlushSCS();
  writeBuf(id, mem_addr, data, len, INST_WRITE);
  wFlushSCS();
  return Ack(id);
}

int SCS::writeByte(u8 id, u8 mem_addr, u8 value)
{
  return genWrite(id, mem_addr, &value, 1);
}

int SCS::Read(u8 id, u8 mem_addr, u8 * data, u8 len)
{
  rFlushSCS();
  writeBuf(id, mem_addr, &len, 1, INST_READ);
  wFlushSCS();

  unsigned char buffer[255];
  const int expected = static_cast<int>(len) + 6;
  const int size = readSCS(buffer, expected);
  if (size != expected) {
    return 0;
  }
  if (buffer[0] != 0xff || buffer[1] != 0xff) {
    return 0;
  }

  u8 checksum = 0;
  for (int i = 2; i < size - 1; ++i) {
    checksum = static_cast<u8>(checksum + buffer[i]);
  }
  checksum = static_cast<u8>(~checksum);
  if (checksum != buffer[size - 1]) {
    return 0;
  }

  std::memcpy(data, buffer + 5, len);
  Error = buffer[4];
  return len;
}

int SCS::readWord(u8 id, u8 mem_addr)
{
  u8 raw[2];
  if (Read(id, mem_addr, raw, 2) != 2) {
    return -1;
  }
  return static_cast<int>(SCS2Host(raw[0], raw[1]));
}

int SCS::Ping(u8 id)
{
  rFlushSCS();
  writeBuf(id, 0, nullptr, 0, INST_PING);
  wFlushSCS();

  unsigned char buffer[6];
  const int size = readSCS(buffer, 6);
  if (size != 6) {
    return -1;
  }
  if (buffer[0] != 0xff || buffer[1] != 0xff || buffer[2] != id || buffer[3] != 2) {
    return -1;
  }

  u8 checksum = 0;
  for (int i = 2; i < size - 1; ++i) {
    checksum = static_cast<u8>(checksum + buffer[i]);
  }
  checksum = static_cast<u8>(~checksum);
  if (checksum != buffer[size - 1]) {
    return -1;
  }

  Error = buffer[4];
  return 1;
}

int SCS::Ack(u8 id)
{
  Error = 0;
  if (id == 0xfe || Level == 0) {
    return 1;
  }

  unsigned char buffer[6];
  const int size = readSCS(buffer, 6);
  if (size != 6) {
    return 0;
  }
  if (buffer[0] != 0xff || buffer[1] != 0xff || buffer[2] != id || buffer[3] != 2) {
    return 0;
  }

  u8 checksum = 0;
  for (int i = 2; i < size - 1; ++i) {
    checksum = static_cast<u8>(checksum + buffer[i]);
  }
  checksum = static_cast<u8>(~checksum);
  if (checksum != buffer[size - 1]) {
    return 0;
  }

  Error = buffer[4];
  return 1;
}
