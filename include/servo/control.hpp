#ifndef CONTROL_H_
#define CONTROL_H_

#define POS_INTERVAL 1
#define X_POS_MIN 3
#define Y_POS_MIN 3
#define X_POS_MAX 12
#define Y_POS_MAX 10


class PosPublisher_ : public rclcpp::Node
{
public:
  explicit PosPublisher_(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~PosPublisher_();

private:
  // functions
  void initialize();
  void get_key_and_pub_pos();

  // vars
  geometry_msgs::msg::Point pos;
  geometry_msgs::msg::Point cam_pos;

  std::string topic_pos = "/control/position";
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_pos;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string topic_cpos = "/camera/pos";
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_cpos;
};
#endif
