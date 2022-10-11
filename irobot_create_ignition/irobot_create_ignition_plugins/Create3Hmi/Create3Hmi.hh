/*
 * Copyright 2021 Clearpath Robotics, Inc.
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#ifndef IROBOT_CREATE_IGNITION__IROBOT_CREATE_IGNITION_PLUGINS__CREATE3HMI__CREATE3HMI_HH_
#define IROBOT_CREATE_IGNITION__IROBOT_CREATE_IGNITION_PLUGINS__CREATE3HMI__CREATE3HMI_HH_

#include <ignition/transport/Node.hh>

#include <ignition/gui/qt.h>
#include <ignition/gui/Plugin.hh>

#include <string>

namespace ignition
{

namespace gui
{

class Create3Hmi : public Plugin
{
  Q_OBJECT

  // \brief Topic
  Q_PROPERTY(
    QString topic
    READ Topic
    WRITE SetTopic
    NOTIFY TopicChanged
  )

public:
  /// \brief Constructor
  Create3Hmi();
  /// \brief Destructor
  virtual ~Create3Hmi();
  /// \brief Called by Ignition GUI when plugin is instantiated.
  /// \param[in] _pluginElem XML configuration for this plugin.
  void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;
  // \brief Get the topic as a string, for example
  /// '/echo'
  /// \return Topic
  Q_INVOKABLE QString Topic() const;

public slots: 
  /// \brief Callback in Qt thread when the topic changes.
  /// \param[in] _topic variable to indicate the topic to
  /// publish the Button commands.
  void SetTopic(const QString &_topic);

signals:
  /// \brief Notify that topic has changed
  void TopicChanged();

protected slots:
  /// \brief Callback trigged when the button is pressed.
  void OnCreate3Button(const int button);
  void OnNamespaceChange(const char* text);

private:
  ignition::transport::Node node_;
  ignition::transport::Node::Publisher create3_button_pub_;
  std::string create3_button_topic_ = "/create3/buttons";
};

}  // namespace gui

}  // namespace ignition

#endif  // IROBOT_CREATE_IGNITION__IROBOT_CREATE_IGNITION_PLUGINS__CREATE3HMI__CREATE3HMI_HH_
