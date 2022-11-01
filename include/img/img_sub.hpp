#ifndef IMG_SUB_H_
#define IMG_SUB_H_
class CamSubscriber_ : public rclcpp::Node
{
public:
  explicit CamSubscriber_(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~CamSubscriber_();

private:
  //functions
  void initialize();
  int encoding2mat(const std::string& encoding);
  void process_image(const sensor_msgs::msg::Image::SharedPtr msg);
  std::string topic_img = "/camera/mat2msg";
  std::string window_name = "Live";
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};
#endif
