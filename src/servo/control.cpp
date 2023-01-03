#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <wiringPi.h>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "servo/control.hpp"

using namespace std::chrono_literals;

/*
 * camera position.
 * pos_x: pwm_freq(50Hz), 3% right-end.
 * pos_y: pwm_freq(50Hz), 3% look at the top.
 * max_freq: x: 12%, y: 10%
 *
 */
PosContol_::PosContol_(const rclcpp::NodeOptions & node_options)
  : Node("control", node_options)
{
  this->pwm_setup();
  this->initialize();
}

PosContol_::~PosContol_()
{
}

void PosContol_::pwm_setup()
{
  wiringPiSetup();
  pinMode(MOTOR_X, PWM_OUTPUT);
  pinMode(MOTOR_Y, PWM_OUTPUT);
  pwmSetClock(PWM_SCALE);
  pwmSetRange(PWM_PERIOD);
  pwmWrite(MOTOR_X, X_POS_BASE);
  pwmWrite(MOTOR_Y, Y_POS_BASE);
}

void PosContol_::initialize()
{
  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = this->get_parameter("qos_depth").get_value<int8_t>();
  this->get_parameter("qos_depth", qos_depth);
  
  motor_service_client_ = this->create_client<MotorPosition>(service_pos);
  while (!motor_service_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
      return;
	}
	RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
  }
}

void PosContol_::send_request()
{
  auto request = std::make_shared<MotorPosition::Request>();
  request->flag = 1;

  using ServiceResponseFuture = rclcpp::Client<MotorPosition>::SharedFuture;
  auto response_received_callback =
    [this](ServiceResponseFuture future)
	{
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "Result x: %d y: %d", response->dst_x, response->dst_y);
      pwmWrite(MOTOR_X, response->dst_x);
      // pwmWrite(MOTOR_Y, response->dst_y);
      delay(500);
      return;
    };

  auto future_result =
    motor_service_client_->async_send_request(request, response_received_callback);

  request->flag = 0;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PosContol_>();
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->send_request();
    delay(DELAY);
  }
  rclcpp::shutdown();

  return 0;
}

