#include <chrono>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "servo/control.hpp"

void PosPublisher_::initialize()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  pub_pos = this->create_publisher<geometry_msgs::msg::Point>(
      topic_pos, qos);
  pos.x = X_POS_MIN;
  pos.y = Y_POS_MIN;

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
      pos.x -= POS_INTERVAL;
      if (pos.x <= X_POS_MIN)
        pos.x = X_POS_MIN;
      printf("x = %f\n", pos.x);
      break;
    case 'd':
      pos.x += POS_INTERVAL;
      if (pos.x >= X_POS_MAX)
        pos.x = X_POS_MAX;
      printf("x = %f\n", pos.x);
      break;
    case 's':
      pos.y -= POS_INTERVAL;
      if (pos.y <= Y_POS_MIN)
        pos.y = Y_POS_MIN;
      printf("y = %f\n", pos.y);
      break;
    case 'w':
      pos.y += POS_INTERVAL;
      if (pos.y >= Y_POS_MAX)
        pos.y = Y_POS_MAX;
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
