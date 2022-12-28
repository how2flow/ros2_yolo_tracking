#include <functional>
#include <memory>
#include <string>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "servo/servo.hpp"

Servo_::Servo_(const rclcpp::NodeOptions & node_options)
: Node("motor", node_options),
  motor_flag_(0),
  motor_x_(0),
  motor_y_(0)
{
  this->initialize();
}

Servo_::~Servo_()
{
}

void Servo_::initialize()
{
  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = this->get_parameter("qos_depth").get_value<int8_t>();
  this->get_parameter("qos_depth", qos_depth);
  duty.x = 5;
  duty.y = 5;

  auto qos_cpos = rclcpp::QoS(rclcpp::KeepLast(qos_depth));
  sub_cpos = this->create_subscription<geometry_msgs::msg::Point>(
    topic_cpos,
    qos_cpos,
    [this](const geometry_msgs::msg::Point::SharedPtr msg) -> void
    {
      cam_pos.x = msg->x;
      cam_pos.y = msg->y;

      RCLCPP_INFO(this->get_logger(),
        "Subscribed cpos x: %d y: %d", (int)cam_pos.x, (int)cam_pos.y);
    }
  );

  auto get_motor_flag =
    [this](
    const std::shared_ptr<MotorPosition::Request> request,
    std::shared_ptr<MotorPosition::Response> response) -> void
    {
      motor_flag_ = request->flag;
      motor_x_ =
        this->dst_x_position(cam_pos.x, motor_flag_);
      motor_y_ =
        this->dst_y_position(cam_pos.y, motor_flag_);
      response->dst_x = motor_x_;
      response->dst_y = motor_y_;
	};

  motor_service_server_ =
    create_service<MotorPosition>(service_pos, get_motor_flag);
}

int Servo_::dst_x_position(const int & pos, const int8_t flag)
{
  if (tmp.x)
    duty.x = tmp.x;

  if ((pos > 0) && flag) {
    if (pos < 120) {
      duty.x = duty.x + 1;
      if (duty.x >= DUTY_X_MAX)
        duty.x = DUTY_X_MAX;
    }

    if (pos > 440) {
      duty.x = duty.x - 1;
      if (duty.x <= DUTY_X_MIN)
        duty.x = DUTY_X_MIN;
    }
  }
  tmp.x = duty.x;

  return (int)duty.x;
}

int Servo_::dst_y_position(const int & pos, const int8_t flag)
{
  if (tmp.y)
    duty.y = tmp.y;

  if ((pos > 0) && flag) {
    if (pos < 120) {
      duty.y = duty.y - 1;
      if (duty.y <= DUTY_Y_MIN)
        duty.y = DUTY_Y_MIN;
    }

    if (pos > 440) {
      duty.y = duty.y + 1;
      if (duty.y >= DUTY_Y_MAX)
        duty.y = DUTY_Y_MAX;
    }
  }
  tmp.y = duty.y;

  return (int)duty.y;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Servo_>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
