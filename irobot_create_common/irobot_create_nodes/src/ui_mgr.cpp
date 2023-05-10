// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Lola Segura (lsegura@irobot.com)

#include "irobot_create_nodes/ui_mgr.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp_components/register_node_macro.hpp"

namespace irobot_create_nodes
{
using namespace std::placeholders;

UIMgr::UIMgr(const rclcpp::NodeOptions & options)
: rclcpp::Node("ui_mgr", options),
  led_animation_end_duration_(rclcpp::Duration::from_nanoseconds(0)),
  audio_note_sequence_end_duration_(rclcpp::Duration::from_nanoseconds(0)),
  audio_notes_duration_(rclcpp::Duration::from_nanoseconds(0))
{
  last_animation_feedback_time_ = this->now();
  last_audio_note_feedback_time_ = this->now();
  // Gazebo simulator being used
  gazebo_ =
    this->declare_parameter("gazebo", "classic");

  // Topic parameter to publish buttons to
  buttons_publisher_topic_ =
    this->declare_parameter("button_topic", "interface_buttons");

  // Subscriber topics
  lightring_subscription_topic_ =
    this->declare_parameter("lightring_topic", "cmd_lightring");
  audio_subscription_topic_ =
    this->declare_parameter("audio_topic", "cmd_audio");

  // Publish rate parameters in Hz
  const double buttons_publish_rate =
    this->declare_parameter("buttons_publish_rate", 1.0);

  // Define buttons publisher
  if (gazebo_ != "ignition") {
    buttons_publisher_ = create_publisher<irobot_create_msgs::msg::InterfaceButtons>(
      buttons_publisher_topic_, rclcpp::QoS(10).reliable());
    RCLCPP_INFO_STREAM(get_logger(), "Advertised button topic: " << buttons_publisher_topic_);
  }

  // Subscription to the lightring leds
  lightring_subscription_ = create_subscription<irobot_create_msgs::msg::LightringLeds>(
    lightring_subscription_topic_, rclcpp::SensorDataQoS(),
    std::bind(&UIMgr::lightring_callback, this, std::placeholders::_1));
  RCLCPP_INFO_STREAM(get_logger(), "Subscription to topic: " << lightring_subscription_topic_);

  audio_subscription_ = create_subscription<irobot_create_msgs::msg::AudioNoteVector>(
    audio_subscription_topic_, rclcpp::SensorDataQoS(),
    std::bind(&UIMgr::audio_callback, this, std::placeholders::_1));
  RCLCPP_INFO_STREAM(get_logger(), "Subscription to topic: " << audio_subscription_topic_);

  led_animation_action_server_ =
    rclcpp_action::create_server<irobot_create_msgs::action::LedAnimation>(
    this,
    "led_animation",
    std::bind(&UIMgr::handle_led_animation_goal, this, _1, _2),
    std::bind(&UIMgr::handle_led_animation_cancel, this, _1),
    std::bind(&UIMgr::handle_led_animation_accepted, this, _1));

  audio_note_sequence_action_server_ =
    rclcpp_action::create_server<irobot_create_msgs::action::AudioNoteSequence>(
    this,
    "audio_note_sequence",
    std::bind(&UIMgr::handle_audio_note_sequence_goal, this, _1, _2),
    std::bind(&UIMgr::handle_audio_note_sequence_cancel, this, _1),
    std::bind(&UIMgr::handle_audio_note_sequence_accepted, this, _1));

  if (gazebo_ != "ignition") {
    buttons_timer_ = rclcpp::create_timer(
      this,
      this->get_clock(),
      rclcpp::Duration(std::chrono::duration<double>(1 / buttons_publish_rate)), [this]() {
        // Set header timestamp.
        this->buttons_msg_.header.stamp = now();

        this->buttons_msg_.button_1.header.stamp = now();
        this->buttons_msg_.button_power.header.stamp = now();
        this->buttons_msg_.button_2.header.stamp = now();

        // Publish topics
        this->buttons_publisher_->publish(this->buttons_msg_);
      });

    // Set buttons header
    buttons_msg_.header.frame_id = base_frame_;
    buttons_msg_.button_1.header.frame_id = "button_1";
    buttons_msg_.button_power.header.frame_id = "button_power";
    buttons_msg_.button_2.header.frame_id = "button_2";
  }
}
rclcpp_action::GoalResponse UIMgr::handle_led_animation_goal(
  const rclcpp_action::GoalUUID & /* uuid*/,
  std::shared_ptr<const irobot_create_msgs::action::LedAnimation::Goal>/*goal*/)
{
  RCLCPP_INFO(get_logger(), "Received new Led Animation goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse UIMgr::handle_led_animation_cancel(
  const std::shared_ptr<
    rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::LedAnimation>>/*goal_handle*/)
{
  RCLCPP_INFO(get_logger(), "Received request to cancel Led Animation goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void UIMgr::handle_led_animation_accepted(
  const std::shared_ptr<
    rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::LedAnimation>> goal_handle)
{
  if (goal_handle) {
    auto goal = goal_handle->get_goal();
    if (goal) {
      {
        RCLCPP_INFO(
          get_logger(), "Starting Led Animation goal with animation_type %s",
          (goal->animation_type ==
          irobot_create_msgs::action::LedAnimation::Goal::BLINK_LIGHTS) ?
          "BLINK_LIGHTS" : "SPIN_LIGHTS");
        const std::lock_guard<std::mutex> lock(led_animation_params_mutex_);
        led_animation_end_duration_ = rclcpp::Duration(goal->max_runtime);
        led_animation_start_time_ = this->now();
      }
      std::thread{std::bind(&UIMgr::execute_led_animation, this, _1), goal_handle}.detach();
    }
    last_animation_feedback_time_ = this->now();
  }
}

void UIMgr::execute_led_animation(
  const std::shared_ptr<
    rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::LedAnimation>> goal_handle)
{
  bool working_on_goal = true;
  rclcpp::Rate loop_rate(10.0f);
  while (working_on_goal) {
    loop_rate.sleep();
    rclcpp::Duration led_animation_runtime(rclcpp::Duration::from_nanoseconds(0));
    {
      const std::lock_guard<std::mutex> lock(led_animation_params_mutex_);
      led_animation_runtime = this->now() - led_animation_start_time_;
    }
    if (led_animation_runtime >= led_animation_end_duration_) {
      RCLCPP_INFO(get_logger(), "Led Animation hit max_runtime, succeeded");
      auto result = std::make_shared<irobot_create_msgs::action::LedAnimation::Result>();
      result->runtime = led_animation_runtime;
      goal_handle->succeed(result);
      working_on_goal = false;
    } else if (goal_handle->is_canceling()) {
      RCLCPP_INFO(get_logger(), "Led Animation canceled");
      auto result = std::make_shared<irobot_create_msgs::action::LedAnimation::Result>();
      result->runtime = led_animation_runtime;
      goal_handle->canceled(result);
      working_on_goal = false;
    } else {
      rclcpp::Time current_time = this->now();
      auto time_since_feedback = current_time - last_animation_feedback_time_;
      if (time_since_feedback > report_animation_feedback_interval_) {
        auto feedback = std::make_shared<irobot_create_msgs::action::LedAnimation::Feedback>();
        feedback->remaining_runtime = led_animation_end_duration_ - led_animation_runtime;
        goal_handle->publish_feedback(feedback);
        last_animation_feedback_time_ = current_time;
      }
    }
  }
}

rclcpp_action::GoalResponse UIMgr::handle_audio_note_sequence_goal(
  const rclcpp_action::GoalUUID & /* uuid*/,
  std::shared_ptr<const irobot_create_msgs::action::AudioNoteSequence::Goal>/*goal*/)
{
  RCLCPP_INFO(get_logger(), "Received new Audio Note Sequence goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse UIMgr::handle_audio_note_sequence_cancel(
  const std::shared_ptr<
    rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::AudioNoteSequence>>/*goal_handle*/)
{
  RCLCPP_INFO(get_logger(), "Received request to cancel Audio Note Sequence goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void UIMgr::handle_audio_note_sequence_accepted(
  const std::shared_ptr<
    rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::AudioNoteSequence>> goal_handle)
{
  if (goal_handle) {
    auto goal = goal_handle->get_goal();
    if (goal) {
      {
        RCLCPP_INFO(
          get_logger(), "Starting Audio Note Sequence goal with iterations %d",
          goal->iterations);
        const std::lock_guard<std::mutex> lock(audio_note_sequence_params_mutex_);
        audio_notes_duration_ = rclcpp::Duration::from_nanoseconds(0);
        for (auto itr{goal->note_sequence.notes.cbegin()}; itr != goal->note_sequence.notes.cend();
          ++itr)
        {
          rclcpp::Duration runtime(itr->max_runtime);
          audio_notes_duration_ = audio_notes_duration_ + runtime;
        }
        audio_iterations_ = goal->iterations;
        audio_note_sequence_end_duration_ = rclcpp::Duration::from_nanoseconds(0);
        for (int32_t iters = 0; iters < audio_iterations_; ++iters) {
          audio_note_sequence_end_duration_ = audio_note_sequence_end_duration_ +
            audio_notes_duration_;
        }
        audio_note_sequence_start_time_ = this->now();
      }
      std::thread{std::bind(&UIMgr::execute_audio_note_sequence, this, _1), goal_handle}.detach();
    }
    last_animation_feedback_time_ = this->now();
  }
}

void UIMgr::execute_audio_note_sequence(
  const std::shared_ptr<
    rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::AudioNoteSequence>> goal_handle)
{
  bool working_on_goal = true;
  rclcpp::Rate loop_rate(10.0f);
  while (working_on_goal) {
    loop_rate.sleep();
    rclcpp::Duration audio_note_sequence_runtime(rclcpp::Duration::from_nanoseconds(0));
    {
      const std::lock_guard<std::mutex> lock(audio_note_sequence_params_mutex_);
      audio_note_sequence_runtime = this->now() - audio_note_sequence_start_time_;
    }
    int32_t iters_played =
      static_cast<int32_t>(audio_note_sequence_runtime.nanoseconds() /
      audio_notes_duration_.nanoseconds());
    if (audio_note_sequence_runtime >= audio_note_sequence_end_duration_) {
      RCLCPP_INFO(get_logger(), "Audio Note Sequence ran for note time, succeeded");
      auto result = std::make_shared<irobot_create_msgs::action::AudioNoteSequence::Result>();
      result->runtime = audio_note_sequence_runtime;
      result->complete = true;
      result->iterations_played = audio_iterations_;
      goal_handle->succeed(result);
      working_on_goal = false;
    } else if (goal_handle->is_canceling()) {
      RCLCPP_INFO(get_logger(), "Audio Note Sequence canceled");
      auto result = std::make_shared<irobot_create_msgs::action::AudioNoteSequence::Result>();
      result->runtime = audio_note_sequence_runtime;
      result->complete = false;
      result->iterations_played = iters_played;
      goal_handle->canceled(result);
      working_on_goal = false;
    } else {
      rclcpp::Time current_time = this->now();
      auto time_since_feedback = current_time - last_audio_note_feedback_time_;
      if (time_since_feedback > report_audio_note_feedback_interval_) {
        auto feedback = std::make_shared<irobot_create_msgs::action::AudioNoteSequence::Feedback>();
        feedback->current_runtime = audio_note_sequence_runtime;
        feedback->iterations_played = iters_played;
        goal_handle->publish_feedback(feedback);
        last_audio_note_feedback_time_ = current_time;
      }
    }
  }
}

void UIMgr::lightring_callback(irobot_create_msgs::msg::LightringLeds::SharedPtr /*msg*/)
{
  RCLCPP_INFO(
    get_logger(),
    "Lightring message received but it is not yet implemented in simulation");
}

void UIMgr::audio_callback(irobot_create_msgs::msg::AudioNoteVector::SharedPtr /*msg*/)
{
  RCLCPP_INFO(
    get_logger(),
    "Audio command message received but it is not yet implemented in simulation");
}

}  // namespace irobot_create_nodes

RCLCPP_COMPONENTS_REGISTER_NODE(irobot_create_nodes::UIMgr)
