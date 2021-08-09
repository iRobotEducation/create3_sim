#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class HazardsVector : public rclcpp::Node
{
public:
  HazardsVector() : Node("hazards_vector"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("hazard_detection", 10);
    float freq = 62.0F; // Hz
    timer_ = this->create_wall_timer(std::chrono::duration<float>(1 / freq), std::bind(&HazardsVector::publish_timer_callback, this));
  }

private:
  void publish_timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HazardsVector>());
  rclcpp::shutdown();
  return 0;
}