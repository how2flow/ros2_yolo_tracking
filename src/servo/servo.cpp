#include <functional>
#include <memory>
#include <string>
#include <stdlib.h>
#include <wiringPi.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "servo/servo.hpp"

Sg90Subscriber_::Sg90Subscriber_(const rclcpp::NodeOptions & node_options)
  : Node("motor", node_options)
{
  this->pwm_setup();
  this->initialize();
}

Sg90Subscriber_::~Sg90Subscriber_()
{
}

void Sg90Subscriber_::initialize()
{
  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = this->get_parameter("qos_depth").get_value<int8_t>();
  this->get_parameter("qos_depth", qos_depth);

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

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Sg90Subscriber_>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
