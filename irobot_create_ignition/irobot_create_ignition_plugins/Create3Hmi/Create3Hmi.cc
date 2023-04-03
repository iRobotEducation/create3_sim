/*
 * Copyright 2021 Clearpath Robotics, Inc.
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#include "Create3Hmi.hh"

#include <ignition/msgs/int32.pb.h>

#include <iostream>

#include <ignition/plugin/Register.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>


namespace ignition
{

namespace gui
{

Create3Hmi::Create3Hmi()
  : Plugin()
{
  this->create3_button_pub_ = ignition::transport::Node::Publisher();
  this->create3_button_pub_ =
    this->node_.Advertise < ignition::msgs::Int32 > (this->create3_button_topic_);
}

Create3Hmi::~Create3Hmi()
{
}

void Create3Hmi::LoadConfig(const tinyxml2::XMLElement * _pluginElem)
{
  if (this->title.empty()) {
    this->title = "Create3 HMI";
  }

  if (_pluginElem)
  {
    auto namespaceElem = _pluginElem->FirstChildElement("namespace");
    if (nullptr != namespaceElem && nullptr != namespaceElem->GetText())
      this->SetNamespace(namespaceElem->GetText());
  }

  this->connect(
    this, SIGNAL(AddMsg(QString)), this, SLOT(OnAddMsg(QString)),
    Qt::QueuedConnection);
}

void Create3Hmi::OnCreate3Button(const int button)
{
  ignition::msgs::Int32 button_msg;

  button_msg.set_data(button);

  if (!this->create3_button_pub_.Publish(button_msg)) {
    ignerr << "ignition::msgs::Int32 message couldn't be published at topic: " <<
      this->create3_button_topic_ << std::endl;
  }
}

QString Create3Hmi::Namespace() const
{
  return QString::fromStdString(this->namespace_);
}

void Create3Hmi::SetNamespace(const QString &_name)
{
  this->namespace_ = _name.toStdString();
  this->create3_button_topic_ = this->namespace_ + "/create3_buttons";

  ignmsg << "A new robot name has been entered, publishing on topic: '" <<
      this->create3_button_topic_ << " ' " <<std::endl;

  // Update publisher with new topic.
  this->create3_button_pub_ = ignition::transport::Node::Publisher();
  this->create3_button_pub_ =
      this->node_.Advertise< ignition::msgs::Int32 >
      (this->create3_button_topic_);
  if (!this->create3_button_pub_)
  {
    App()->findChild<MainWindow *>()->notifyWithDuration(
      QString::fromStdString("Error when advertising topic: " +
        this->create3_button_topic_), 4000);
    ignerr << "Error when advertising topic: " <<
      this->create3_button_topic_ << std::endl;
  }else {
    App()->findChild<MainWindow *>()->notifyWithDuration(
      QString::fromStdString("Advertising topic: '<b>" +
        this->create3_button_topic_ + "</b>'"), 4000);
  }
  this->NamespaceChanged();
}

}  // namespace gui

}  // namespace ignition

// Register this plugin
IGNITION_ADD_PLUGIN(
  ignition::gui::Create3Hmi,
  ignition::gui::Plugin)
