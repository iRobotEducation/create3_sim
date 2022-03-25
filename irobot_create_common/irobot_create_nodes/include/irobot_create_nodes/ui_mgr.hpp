// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Lola Segura (lsegura@irobot.com)

#ifndef IROBOT_CREATE_NODES__UI_MGR_HPP_
#define IROBOT_CREATE_NODES__UI_MGR_HPP_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "irobot_create_msgs/action/audio_note_sequence.hpp"
#include "irobot_create_msgs/action/led_animation.hpp"
#include "irobot_create_msgs/msg/audio_note_vector.hpp"
#include "irobot_create_msgs/msg/button.hpp"
#include "irobot_create_msgs/msg/interface_buttons.hpp"
#include "irobot_create_msgs/msg/led_color.hpp"
#include "irobot_create_msgs/msg/lightring_leds.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace irobot_create_nodes
{

class UIMgr : public rclcpp::Node
{
public:
  /// \brief Constructor
  explicit UIMgr(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Callback functions
  void lightring_callback(irobot_create_msgs::msg::LightringLeds::SharedPtr msg);
  void audio_callback(irobot_create_msgs::msg::AudioNoteVector::SharedPtr msg);

private:
  rclcpp_action::GoalResponse handle_led_animation_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const irobot_create_msgs::action::LedAnimation::Goal> goal);
  rclcpp_action::CancelResponse handle_led_animation_cancel(
    const std::shared_ptr<
      rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::LedAnimation>> goal_handle);
  void handle_led_animation_accepted(
    const std::shared_ptr<
      rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::LedAnimation>> goal_handle);
  void execute_led_animation(
    const std::shared_ptr<
      rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::LedAnimation>> goal_handle);

  rclcpp_action::GoalResponse handle_audio_note_sequence_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const irobot_create_msgs::action::AudioNoteSequence::Goal> goal);
  rclcpp_action::CancelResponse handle_audio_note_sequence_cancel(
    const std::shared_ptr<
      rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::AudioNoteSequence>> goal_handle);
  void handle_audio_note_sequence_accepted(
    const std::shared_ptr<
      rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::AudioNoteSequence>> goal_handle);
  void execute_audio_note_sequence(
    const std::shared_ptr<
      rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::AudioNoteSequence>> goal_handle);

  // Publish aggregated detections on timer_'s frequency
  rclcpp::TimerBase::SharedPtr buttons_timer_;

  // Publishers
  std::shared_ptr<
    rclcpp::Publisher<irobot_create_msgs::msg::InterfaceButtons>> buttons_publisher_{nullptr};

  // Subscribers
  rclcpp::Subscription<irobot_create_msgs::msg::LightringLeds>::SharedPtr lightring_subscription_;
  rclcpp::Subscription<irobot_create_msgs::msg::AudioNoteVector>::SharedPtr audio_subscription_;

  // Actions
  rclcpp_action::Server<irobot_create_msgs::action::LedAnimation>::SharedPtr
    led_animation_action_server_;
  rclcpp_action::Server<irobot_create_msgs::action::AudioNoteSequence>::SharedPtr
    audio_note_sequence_action_server_;

  // Gazebo simulator being used
  std::string gazebo_;

  // Topic to publish interface buttons to
  std::string buttons_publisher_topic_;

  // Topic to subscribe to light ring vector
  std::string lightring_subscription_topic_;
  // Topic to subscribe to audio note vector
  std::string audio_subscription_topic_;

  // Message to store the interface buttons
  irobot_create_msgs::msg::InterfaceButtons buttons_msg_;

  const std::string base_frame_ {"base_link"};
  std::mutex led_animation_params_mutex_;
  rclcpp::Duration led_animation_end_duration_;
  rclcpp::Time led_animation_start_time_;
  rclcpp::Time last_animation_feedback_time_;
  const rclcpp::Duration report_animation_feedback_interval_ {std::chrono::seconds(3)};
  std::mutex audio_note_sequence_params_mutex_;
  rclcpp::Duration audio_note_sequence_end_duration_;
  rclcpp::Time audio_note_sequence_start_time_;
  rclcpp::Time last_audio_note_feedback_time_;
  int32_t audio_iterations_;
  rclcpp::Duration audio_notes_duration_;
  const rclcpp::Duration report_audio_note_feedback_interval_ {std::chrono::seconds(3)};
};

}  // namespace irobot_create_nodes

#endif  // IROBOT_CREATE_NODES__UI_MGR_HPP_
