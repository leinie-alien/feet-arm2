#include <chrono>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

extern "C" {
#include "bsp_can.h"
#include "dm_motor_ctrl.h"
#include "dm_motor_drv.h"
}

class DmMotorMinimalNode : public rclcpp::Node {
public:
  DmMotorMinimalNode() : Node("dm_motor_minimal_node") {
    channel_ = declare_parameter<int>("channel", 1);
    can_baud_ = declare_parameter<int>("can_baud", 1000000);
    canfd_baud_ = declare_parameter<int>("canfd_baud", 5000000);
    loop_hz_ = declare_parameter<double>("loop_hz", 200.0);

    cmd_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "dm_motor/pos_cmd", 10,
        std::bind(&DmMotorMinimalNode::on_pos_cmd, this, std::placeholders::_1));

    fb_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("dm_motor/feedback", 10);

    if (!canx_open(&hcan1, static_cast<uint8_t>(channel_), can_baud_, canfd_baud_)) {
      throw std::runtime_error("canx_open failed. Check USB2CANFD_Dual connection and permissions.");
    }

    dm_motor_init();
    for (int i = 0; i < 3; ++i) {
      dm_motor_enable(&hcan1, &motor[i]);
    }

    const auto period = std::chrono::duration<double>(1.0 / loop_hz_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&DmMotorMinimalNode::on_timer, this));

    RCLCPP_INFO(get_logger(), "started. channel=%d can=%d canfd=%d loop_hz=%.1f",
                channel_, can_baud_, canfd_baud_, loop_hz_);
  }

  ~DmMotorMinimalNode() override {
    for (int i = 0; i < 3; ++i) {
      dm_motor_disable(&hcan1, &motor[i]);
    }
    canx_close(&hcan1);
  }

private:
  void on_pos_cmd(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    const size_t n = std::min<size_t>(msg->data.size(), 3);
    for (size_t i = 0; i < n; ++i) {
      motor[i].ctrl.pos_set = msg->data[i];
    }
  }

  void on_timer() {
    while (canx_pending(&hcan1) > 0U) {
      can1_rx_callback();
    }

    for (int i = 0; i < 3; ++i) {
      dm_motor_ctrl_send(&hcan1, &motor[i]);
    }

    std_msgs::msg::Float32MultiArray msg;
    msg.data.reserve(12);
    for (int i = 0; i < 3; ++i) {
      msg.data.push_back(static_cast<float>(motor[i].id));
      msg.data.push_back(motor[i].para.pos);
      msg.data.push_back(motor[i].para.vel);
      msg.data.push_back(motor[i].para.tor);
    }
    fb_pub_->publish(msg);
  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr fb_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  int channel_ = 1;
  int can_baud_ = 1000000;
  int canfd_baud_ = 5000000;
  double loop_hz_ = 200.0;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<DmMotorMinimalNode>();
    rclcpp::spin(node);
  } catch (const std::exception& ex) {
    fprintf(stderr, "dm_motor_minimal_node error: %s\n", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
