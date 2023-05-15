^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package irobot_create_nodes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.0 (2023-05-15)
------------------
* Multi robot support (`#207 <https://github.com/iRobotEducation/create3_sim/issues/207>`_)
* Contributors: Roni Kreinin

2.0.0 (2023-01-19)
------------------
* Update to ROS 2 Humble (`#197 <https://github.com/iRobotEducation/create3_sim/issues/197>`_)
  * Update message names to https://github.com/iRobotEducation/irobot_create_msgs/pull/10
  * rename dock topic into dock_status
  * comment ign_ros2_control dependency as it must be built from sources
  Co-authored-by: Francisco Martín Rico <fmrico@gmail.com>
* add missing dependency to irobot-create-common-bringup (`#186 <https://github.com/iRobotEducation/create3_sim/issues/186>`_)
* Contributors: Alberto Soragna, Francisco Martín Rico

1.0.1 (2022-04-12)
------------------
* add boost as explicit dependency and remove unnneded boost usage (`#175 <https://github.com/iRobotEducation/create3_sim/issues/175>`_)
  * add boost as explicit dependency and remove unnneded boost usage
  * do not search for boost component in Boost library

1.0.0 (2022-03-31)
------------------
* Add audio action and move UI elements to ui_mgr node (`#172 <https://github.com/iRobotEducation/create3_sim/issues/172>`_)
  * Add audio notes sequence action and move UI elements from mock to UI mgr node
  * fix linter error
  * add missing yaml file
  * fix flake8 error
* Removed reliable QoS from subscriptions (`#164 <https://github.com/iRobotEducation/create3_sim/issues/164>`_)
* Update dependencies for build from source, throttle action feedback, add stub for audio command (`#161 <https://github.com/iRobotEducation/create3_sim/issues/161>`_)
  * Update dependencies for build from source
  * Add stub for cmd_audio and throttle action feedback
  * fix packages from PR feedback
* Reliable QoS for publishers (`#158 <https://github.com/iRobotEducation/create3_sim/issues/158>`_)
  * reliable QoS for publishers
  * reliable QoS for publishers
  * Fixing tests
* Split `irobot_create_toolbox` (`#153 <https://github.com/iRobotEducation/create3_sim/issues/153>`_)
  * rename irobot_create_toolbox into irobot_create_nodes
  * move common utilities to irobot_create_toolbox
  * register irobot_create_nodes as rclcpp_components
  * update readme
  * use new node names in parameter files
  * remove declare parameter utility and fix linter tests
* Contributors: Alberto Soragna, Justin Kearns, Santti4go, roni-kreinin
