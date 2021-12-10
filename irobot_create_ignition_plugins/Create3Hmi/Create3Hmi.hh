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

#ifndef IGNITION_GUI_CREATE3_HMI_HH_
#define IGNITION_GUI_CREATE3_HMI_HH_

#include <string>

#include <ignition/transport/Node.hh>

#include <ignition/gui/qt.h>
#include <ignition/gui/Plugin.hh>

namespace ignition
{
  namespace gui
  {
    class Create3Hmi : public Plugin
    {
      Q_OBJECT

      /// \brief Constructor
      public: Create3Hmi();

      /// \brief Destructor
      public: virtual ~Create3Hmi();

      /// \brief Called by Ignition GUI when plugin is instantiated.
      /// \param[in] _pluginElem XML configuration for this plugin.
      public: virtual void LoadConfig(const tinyxml2::XMLElement *_pluginElem)
          override;

      /// \brief Callback trigged when the button is pressed.
      protected slots: void OnCreate3Button(const int button);

      private: ignition::transport::Node node_;

      private: ignition::transport::Node::Publisher create3_button_pub_;

      private: std::string create3_button_topic_ = "/create3/buttons";
    };
  }
}

#endif // IGNITION_GUI_CREATE3_HMI_HH_