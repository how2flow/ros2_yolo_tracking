#ifndef CONTROL_H_
#define CONTROL_H_

class PosPublisher_ : public rclcpp::Node
{
  public:
	PosPublisher_()
	: Node("key")
	{
		initialize();
	}

	~PosPublisher_()
	{
	}
  private:
	// functions
	void initialize();
	void get_key_and_publish();
	int move(int);

	// vars
	std::string topic_ = "/control/position";
	rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_;
	rclcpp::TimerBase::SharedPtr timer_;
	geometry_msgs::msg::Point pos;
    int mo;
};
#endif
