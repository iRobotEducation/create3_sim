#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <irobot_create_msgs/msg/hazard_detection.hpp>

class HazardsVector : public rclcpp::Node
{
public:
  HazardsVector() : Node("hazards_vector"), count_(0)
  {
    publisher_ = this->create_publisher<irobot_create_msgs::msg::HazardDetection>("hazard_detection", 10);
    float freq = 62.0F; // Hz
    timer_ = this->create_wall_timer(std::chrono::duration<float>(1 / freq), std::bind(&HazardsVector::publish_timer_callback, this));
  }

private:
  void publish_timer_callback()
  {
    auto message = irobot_create_msgs::msg::HazardDetection();
    RCLCPP_INFO(this->get_logger(), "Publishing now");
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<irobot_create_msgs::msg::HazardDetection>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HazardsVector>());
  rclcpp::shutdown();
  return 0;
}