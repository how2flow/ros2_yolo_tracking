#include <chrono>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "servo/control.hpp"

PosPublisher_::PosPublisher_(const rclcpp::NodeOptions & node_options)
  : Node("control", node_options)
{
  this->initialize();
}

PosPublisher_::~PosPublisher_()
{
}

void PosPublisher_::initialize()
{
  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = this->get_parameter("qos_depth").get_value<int8_t>();
  this->get_parameter("qos_depth", qos_depth);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth));
  pub_pos = this->create_publisher<geometry_msgs::msg::Point>(
      topic_pos, qos);

  pos.x = X_POS_MIN;
  pos.y = Y_POS_MIN;

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(900)),
      std::bind(&PosPublisher_::get_key_and_pub_pos, this));
}

void PosPublisher_::get_key_and_pub_pos()
{
  pos.x += POS_INTERVAL;
  if (pos.x >= X_POS_MAX)
    pos.x = X_POS_MIN;

  pos.y += POS_INTERVAL;
  if (pos.y >= Y_POS_MAX)
    pos.y = Y_POS_MIN;

  pub_pos->publish(pos);

  RCLCPP_INFO(this->get_logger(), "Published x %.2f", pos.x);
  RCLCPP_INFO(this->get_logger(), "Published y %.2f", pos.y);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PosPublisher_>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
