#include <chrono>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "servo/control.hpp"

#define MIN 5
#define MAX 24

void PosPublisher_::initialize()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  pub_pos = this->create_publisher<geometry_msgs::msg::Point>(
      topic_pos, qos);
  pos.x = 5;
  pos.y = 5;

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(100)),
      std::bind(&PosPublisher_::get_key_and_pub_pos, this));
}

void PosPublisher_::get_key_and_pub_pos()
{
  char input;

  std::cin >> input;

  switch (input) {
    case 'a':
      pos.x -= 1;
      if (pos.x <= 5)
        pos.x = 5;
      printf("x = %f\n", pos.x);
      break;
    case 'd':
      pos.x += 1;
      if (pos.x >= 25)
        pos.x = 25;
      printf("x = %f\n", pos.x);
      break;
    case 's':
      pos.y -= 1;
      if (pos.y <= 5)
        pos.y = 5;
      printf("y = %f\n", pos.y);
      break;
    case 'w':
      pos.y += 1;
      if (pos.y >= 15)
        pos.y = 15;
      printf("y = %f\n", pos.y);
      break;
    default:
      printf("x = %f\n", pos.x);
      printf("y = %f\n", pos.y);
      break;
  }

  pub_pos->publish(pos);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PosPublisher_>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
