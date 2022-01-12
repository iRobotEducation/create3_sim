/*
 * Copyright 2021 Clearpath Robotics, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#include <iostream>
#include <ignition/plugin/Register.hh>

#include "Create3Hmi.hh"

#include <ignition/plugin/Register.hh>

#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>

#include <ignition/msgs/int32.pb.h>

using namespace ignition;
using namespace gui;


Create3Hmi::Create3Hmi()
    : Plugin()
{
  this->create3_button_pub_ = ignition::transport::Node::Publisher();
  this->create3_button_pub_ = this->node_.Advertise<ignition::msgs::Int32>(this->create3_button_topic_);
}

Create3Hmi::~Create3Hmi()
{
}

void Create3Hmi::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (!_pluginElem)
    return;

  if (this->title.empty())
  {
    this->title = "Create3 HMI";
  }

  this->connect(this, SIGNAL(AddMsg(QString)), this, SLOT(OnAddMsg(QString)),
                Qt::QueuedConnection);
}

void Create3Hmi::OnCreate3Button(const int button)
{
  ignition::msgs::Int32 button_msg;

  button_msg.set_data(button);

  if (!this->create3_button_pub_.Publish(button_msg))
    ignerr << "ignition::msgs::Int32 message couldn't be published at topic: "
           << this->create3_button_topic_ << std::endl;
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gui::Create3Hmi,
                    ignition::gui::Plugin)