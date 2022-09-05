#ifndef SERVO_H_
#define SERVO_H_

class Sg90Subscriber_ : public rclcpp::Node
{
  public:
	Sg90Subscriber_()
	: Node("motor")
	{
		initialize();
	}

	~Sg90Subscriber_()
	{
	}
  private:
	// functions
	void initialize();
	void motor_write(geometry_msgs::msg::Point::SharedPtr pos);

	// vars
	std::string topic_ = "/control/position";
	rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_;
};
#endif
