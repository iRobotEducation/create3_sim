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

  // \brief Name
  Q_PROPERTY(
    QString name
    READ Name
    WRITE SetName
    NOTIFY NameChanged
  )

public:
  /// \brief Constructor
  Create3Hmi();
  /// \brief Destructor
  virtual ~Create3Hmi();
  /// \brief Called by Ignition GUI when plugin is instantiated.
  /// \param[in] _pluginElem XML configuration for this plugin.
  void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;
  // \brief Get the robot name as a string, for example
  /// '/echo'
  /// \return Name
  Q_INVOKABLE QString Name() const;

public slots:
  /// \brief Callback in Qt thread when the robot name changes.
  /// \param[in] _name variable to indicate the robot name to
  /// publish the Button commands.
  void SetName(const QString &_name);

signals:
  /// \brief Notify that robot name has changed
  void NameChanged();

protected slots:
  /// \brief Callback trigged when the button is pressed.
  void OnCreate3Button(const int button);

private:
  ignition::transport::Node node_;
  ignition::transport::Node::Publisher create3_button_pub_;
  std::string robot_name_ = "create3";
  std::string create3_button_topic_ = "/model/create3/create3_buttons";
};

}  // namespace gui

}  // namespace ignition

#endif  // IROBOT_CREATE_IGNITION__IROBOT_CREATE_IGNITION_PLUGINS__CREATE3HMI__CREATE3HMI_HH_