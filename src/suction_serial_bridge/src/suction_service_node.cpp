#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/srv/set_suction.hpp"

namespace
{
using SetSuction = robot_msgs::srv::SetSuction;

speed_t to_speed_t(int baud_rate)
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
    case 230400:
      return B230400;
    default:
      throw std::invalid_argument("unsupported baud rate");
  }
}
}  // namespace

class SuctionServiceNode : public rclcpp::Node
{
public:
  SuctionServiceNode()
  : Node("suction_service_node")
  {
    serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    baud_rate_ = declare_parameter<int>("baud_rate", 115200);
    service_name_ = declare_parameter<std::string>("service_name", "set_suction");
    response_timeout_ms_ = declare_parameter<int>("response_timeout_ms", 500);

    open_serial_or_throw();

    service_ = create_service<SetSuction>(
      service_name_,
      std::bind(&SuctionServiceNode::handle_request, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "Ready: service=%s serial_port=%s baud=%d",
      service_name_.c_str(), serial_port_.c_str(), baud_rate_);
  }

  ~SuctionServiceNode() override
  {
    std::scoped_lock<std::mutex> lock(serial_mutex_);
    close_serial_locked();
  }

private:
  void handle_request(
    const std::shared_ptr<SetSuction::Request> request,
    std::shared_ptr<SetSuction::Response> response)
  {
    std::scoped_lock<std::mutex> lock(serial_mutex_);

    try {
      ensure_serial_open_locked();
      tcflush(serial_fd_, TCIFLUSH);

      const std::string command = request->activate ? "SET 1\n" : "SET 0\n";
      write_all_locked(command);

      const std::string reply = read_line_locked();
      const std::string expected = request->activate ? "OK 1" : "OK 0";
      response->success = (reply == expected);

      if (response->success) {
        RCLCPP_INFO(
          get_logger(),
          "Suction command applied: activate=%s",
          request->activate ? "true" : "false");
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "Unexpected ESP32 reply for activate=%s: '%s'",
          request->activate ? "true" : "false", reply.c_str());
      }
    } catch (const std::exception & e) {
      response->success = false;
      RCLCPP_ERROR(get_logger(), "Suction request failed: %s", e.what());
      close_serial_locked();
    }
  }

  void open_serial_or_throw()
  {
    std::scoped_lock<std::mutex> lock(serial_mutex_);
    open_serial_locked();
  }

  void ensure_serial_open_locked()
  {
    if (serial_fd_ < 0) {
      open_serial_locked();
    }
  }

  void open_serial_locked()
  {
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0) {
      throw std::runtime_error("failed to open serial port " + serial_port_ + ": " + std::strerror(errno));
    }

    termios tty {};
    if (tcgetattr(serial_fd_, &tty) != 0) {
      const std::string error = std::strerror(errno);
      close_serial_locked();
      throw std::runtime_error("tcgetattr failed: " + error);
    }

    cfmakeraw(&tty);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = static_cast<cc_t>((response_timeout_ms_ + 99) / 100);

    const speed_t baud = to_speed_t(baud_rate_);
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      const std::string error = std::strerror(errno);
      close_serial_locked();
      throw std::runtime_error("tcsetattr failed: " + error);
    }

    tcflush(serial_fd_, TCIOFLUSH);
  }

  void close_serial_locked()
  {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
      serial_fd_ = -1;
    }
  }

  void write_all_locked(const std::string & data)
  {
    std::size_t offset = 0;
    while (offset < data.size()) {
      const ssize_t written = write(serial_fd_, data.data() + offset, data.size() - offset);
      if (written < 0) {
        throw std::runtime_error("serial write failed: " + std::string(std::strerror(errno)));
      }
      offset += static_cast<std::size_t>(written);
    }
    tcdrain(serial_fd_);
  }

  std::string read_line_locked()
  {
    std::string line;
    char ch = '\0';

    while (true) {
      const ssize_t n = read(serial_fd_, &ch, 1);
      if (n < 0) {
        throw std::runtime_error("serial read failed: " + std::string(std::strerror(errno)));
      }
      if (n == 0) {
        throw std::runtime_error("serial read timeout");
      }
      if (ch == '\r') {
        continue;
      }
      if (ch == '\n') {
        break;
      }
      line.push_back(ch);
    }

    return line;
  }

  std::string serial_port_;
  int baud_rate_ {115200};
  std::string service_name_;
  int response_timeout_ms_ {500};
  int serial_fd_ {-1};
  std::mutex serial_mutex_;
  rclcpp::Service<SetSuction>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SuctionServiceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
