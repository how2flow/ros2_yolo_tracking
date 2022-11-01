#ifndef CONTROL_H_
#define CONTROL_H_

#define POS_INTERVAL 20
#define X_POS_MIN 60
#define Y_POS_MIN 60
#define X_POS_MAX 240
#define Y_POS_MAX 200


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
  std::string topic_pos = "/control/position";
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_pos;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Point pos;
};
#endif
