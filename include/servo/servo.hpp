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
  void pwm_write(int chip, int duty_cycle);
  void servo_write(geometry_msgs::msg::Point::SharedPtr pos);

  // vars
  std::string topic_pos = "/control/position";
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_pos;
  geometry_msgs::msg::Point pos;

  // pwm
  char *pwmchip[10];
  char *pwm_option[6];
};
#endif
