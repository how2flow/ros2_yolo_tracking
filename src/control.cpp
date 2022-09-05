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
	pub_ = this->create_publisher<geometry_msgs::msg::Point>(
			topic_, qos);
	mo = 0;
	pos.x = 16;
	pos.y = 8;

	timer_ = this->create_wall_timer(
			std::chrono::milliseconds(static_cast<int>(100)),
			std::bind(&PosPublisher_::get_key_and_publish, this));
}

void PosPublisher_::get_key_and_publish()
{
	char input;

	std::cin >> input;

	switch (input) {
		case 'a':
			mo -= 1;
			if (mo <= 0)
				mo = 0;
			pos.x = PosPublisher_::move(mo); // -90
			printf("x = %f\n", pos.x);
			break;
		case 'd':
			mo += 1;
			if (mo >= 20)
				mo = 20;
			pos.x = PosPublisher_::move(mo); // +90
			printf("x = %f\n", pos.x);
			break;
		case 's':
			mo -= 1;
			if (mo <= 0)
				mo = 0;
			pos.y = PosPublisher_::move(mo); // -90
			printf("y = %f\n", pos.y);
			break;
		case 'w':
			mo += 1;
			if (mo >= 20)
				mo = 20;
			pos.y = PosPublisher_::move(mo); // 90
			printf("y = %f\n", pos.y);
			break;
		default:
			pos.x = 5;
			pos.y = 5;
			break;
	}

	pub_->publish(pos);
}

int PosPublisher_::move(int arg)
{
	int ret;

	if (arg <= 0)
		return MIN;
	ret = arg + 4;
	if (ret > MAX)
		return MAX;

	return ret;

}

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<PosPublisher_>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
