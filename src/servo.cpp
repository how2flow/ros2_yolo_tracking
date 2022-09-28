#include <functional>
#include <memory>
#include <string>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "servo/servo.hpp"
#include "servo/pwm.hpp"

#define MOTOR_X 2
#define MOTOR_Y 9

void Sg90Subscriber_::initialize()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  auto pwm = std::make_shared<PwmSignal_>();
  auto callback =
    [this](geometry_msgs::msg::Point::SharedPtr dty) {
      servo_write(dty);
    };

  // pwm config
  pwm->pwm_setup(2);
  pwm->pwm_setup(9);

  // subscriber set
  sub_pos = create_subscription<geometry_msgs::msg::Point>(topic_pos, qos, callback);
}

void Sg90Subscriber_::servo_write(geometry_msgs::msg::Point::SharedPtr pos)
{
  pwm_write(MOTOR_X, (int)pos->x);
  pwm_write(MOTOR_Y, (int)pos->y);
}

void Sg90Subscriber_::pwm_write(int chip, int duty_cycle)
{
  char cmd[100];
  auto pwm = std::make_shared<PwmSignal_>();

  duty_cycle = duty_cycle * 100000;
  sprintf(cmd, "echo %d > %s/%s", duty_cycle,
		  pwm->pwmchip[chip], pwm->pwm_option[DUTY_CYCLE]);
  system(cmd);
  printf("cmd = %s\n", cmd);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Sg90Subscriber_>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
