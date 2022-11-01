#include <functional>
#include <memory>
#include <string>
#include <stdlib.h>
#include <wiringPi.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "servo/servo.hpp"

#define MOTOR_X 23
#define MOTOR_Y 7

void Sg90Subscriber_::initialize()
{
  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = this->get_parameter("qos_depth").get_value<int8_t>();
  this->get_parameter("qos_depth", qos_depth);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth));
  auto callback =
    [this](geometry_msgs::msg::Point::SharedPtr dty) {
      servo_write(dty);
    };

  // subscriber set
  sub_pos = create_subscription<geometry_msgs::msg::Point>(topic_pos, qos, callback);
}

void Sg90Subscriber_::pwm_setup()
{
  wiringPiSetup();
  pinMode(MOTOR_X, PWM_OUTPUT);
  pinMode(MOTOR_Y, PWM_OUTPUT);
  pwmSetClock(PWM_SCALE);
  pwmSetRange(PWM_PERIOD);
  pwmWrite(MOTOR_X, X_POS_BASE);
  pwmWrite(MOTOR_Y, Y_POS_BASE);
}

void Sg90Subscriber_::servo_write(geometry_msgs::msg::Point::SharedPtr pos)
{
  pwmWrite(MOTOR_X, (int)pos->x);
  pwmWrite(MOTOR_Y, (int)pos->y);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Sg90Subscriber_>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
