#include "SCSerial.h"

#include <fcntl.h>
#include <sys/select.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace
{

speed_t to_speed(int baud_rate)
{
  switch (baud_rate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 500000:
      return B500000;
    case 1000000:
      return B1000000;
    default:
      return B115200;
  }
}

}  // namespace

SCSerial::SCSerial()
: SCS(), IOTimeOut(100), Err(0), fd_(-1), tx_buffer_len_(0)
{
  std::memset(&original_options_, 0, sizeof(original_options_));
  std::memset(&current_options_, 0, sizeof(current_options_));
  std::memset(tx_buffer_, 0, sizeof(tx_buffer_));
}

SCSerial::SCSerial(u8 end)
: SCS(end), IOTimeOut(100), Err(0), fd_(-1), tx_buffer_len_(0)
{
  std::memset(&original_options_, 0, sizeof(original_options_));
  std::memset(&current_options_, 0, sizeof(current_options_));
  std::memset(tx_buffer_, 0, sizeof(tx_buffer_));
}

SCSerial::SCSerial(u8 end, u8 level)
: SCS(end, level), IOTimeOut(100), Err(0), fd_(-1), tx_buffer_len_(0)
{
  std::memset(&original_options_, 0, sizeof(original_options_));
  std::memset(&current_options_, 0, sizeof(current_options_));
  std::memset(tx_buffer_, 0, sizeof(tx_buffer_));
}

SCSerial::~SCSerial()
{
  end();
}

bool SCSerial::begin(int baud_rate, const char * serial_port)
{
  end();
  if (serial_port == nullptr) {
    return false;
  }

  fd_ = open(serial_port, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ == -1) {
    return false;
  }

  if (tcgetattr(fd_, &original_options_) != 0) {
    end();
    return false;
  }
  current_options_ = original_options_;

  const speed_t speed = to_speed(baud_rate);
  cfsetispeed(&current_options_, speed);
  cfsetospeed(&current_options_, speed);

  current_options_.c_cflag &= ~PARENB;
  current_options_.c_cflag &= ~CSTOPB;
  current_options_.c_cflag &= ~CSIZE;
  current_options_.c_cflag |= CS8;
  current_options_.c_cflag |= CREAD;
  current_options_.c_cflag |= CLOCAL;
  cfmakeraw(&current_options_);
  current_options_.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

  if (tcsetattr(fd_, TCSANOW, &current_options_) != 0) {
    end();
    return false;
  }

  tx_buffer_len_ = 0;
  return true;
}

void SCSerial::end()
{
  if (fd_ != -1) {
    tcsetattr(fd_, TCSANOW, &original_options_);
    close(fd_);
    fd_ = -1;
  }
  tx_buffer_len_ = 0;
}

int SCSerial::readSCS(unsigned char * data, int len)
{
  if (fd_ == -1 || len <= 0) {
    return 0;
  }

  int total = 0;
  while (total < len) {
    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(fd_, &read_set);

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = static_cast<suseconds_t>(IOTimeOut * 1000);

    const int ready = select(fd_ + 1, &read_set, nullptr, nullptr, &timeout);
    if (ready <= 0) {
      return total;
    }

    const int count = read(fd_, data + total, len - total);
    if (count <= 0) {
      return total;
    }
    total += count;
  }

  return total;
}

int SCSerial::writeSCS(const unsigned char * data, int len)
{
  if (len <= 0) {
    return tx_buffer_len_;
  }

  const int available = static_cast<int>(sizeof(tx_buffer_)) - tx_buffer_len_;
  if (len > available) {
    throw std::runtime_error("servo tx buffer overflow");
  }

  std::memcpy(tx_buffer_ + tx_buffer_len_, data, len);
  tx_buffer_len_ += len;
  return tx_buffer_len_;
}

int SCSerial::writeSCS(unsigned char byte)
{
  return writeSCS(&byte, 1);
}

void SCSerial::rFlushSCS()
{
  if (fd_ != -1) {
    tcflush(fd_, TCIFLUSH);
  }
}

void SCSerial::wFlushSCS()
{
  if (fd_ == -1 || tx_buffer_len_ == 0) {
    return;
  }

  int written_total = 0;
  while (written_total < tx_buffer_len_) {
    const int count = write(fd_, tx_buffer_ + written_total, tx_buffer_len_ - written_total);
    if (count <= 0) {
      break;
    }
    written_total += count;
  }
  tx_buffer_len_ = 0;
}
