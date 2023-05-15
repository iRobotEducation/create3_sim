^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package irobot_create_gazebo_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.0 (2023-05-15)
------------------
* Multi robot support (`#207 <https://github.com/iRobotEducation/create3_sim/issues/207>`_)
* Contributors: Roni Kreinin

2.0.0 (2023-01-19)
------------------
* make gazebo classic and ignition build time dependency if they are needed by CMakeLists.txt (`#179 <https://github.com/iRobotEducation/create3_sim/issues/179>`_)
* fix dependency tree of create3_sim packages (`#177 <https://github.com/iRobotEducation/create3_sim/issues/177>`_)
* fix setting environment variable GAZEBO_MODEL_PATH for aws models (`#174 <https://github.com/iRobotEducation/create3_sim/issues/174>`_)
* Contributors: Alberto Soragna, crthilakraj

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
* Use package:// to reference meshes (`#168 <https://github.com/iRobotEducation/create3_sim/issues/168>`_)
  * Set GAZEBO_MODEL_URI to empty string to prevent model downloads.
  Added /usr/share/gazebo-11/models/ to GAZEBO_MODEL_PATH to use local ground plane and sun models.
  Using package:// instead of file:// for mesh paths.
  * Append to GAZEBO_MODEL_PATH
* Ignition Gazebo support (`#144 <https://github.com/iRobotEducation/create3_sim/issues/144>`_)
  * Updated URDF to work with both gazebo classic and ignition:
  - Added gazebo arg to create3.urdf.xacro and standard_dock.urdf.xacro. This arg sets which gazebo version to use.
  - Added ray sensor macro which creates the correct sensor given the gazebo arg
  - Added ignition plugins when gazebo=ignition
  - Adjusted front caster position to better align with the create3 model
  - Adjusted wheeldrop spring stiffness to compensate for the front caster position change
  * Launch joint_state_publisher and diffdrive_controller only in classic
  * Use joint_state_publisher for both ignition and classic
  Cleaned up gazebo launch arg
  Adjusted front caster position
  * Simulation -> simulator
  * Added irobot_create_ignition packages
  * Fixed some linter warnings
  * Removed joint state publisher from ros_ign_bridge
  * - Reorganized packages
  - Shifted center of gravity of create3 forwards by 22.8 mm
  - Added min/max velocity and acceleration to diff drive plugin
  * Moved README.md to irobot_create_gazebo
  Created new README.md for irobot_create_ignition
  * Update README.md
  * Added ignition edifice repos for source installation
  * Create README.md
  * Added missing dependency
  * Renamed dock to standard_dock
  Center of gravity offset only applied in ignition for now
  * Fixed Linter errors
  * Updated README to have installation and example instructions for both Ignition and Classic
  Moved .repos files to the root of the repository
  * Ignition and Gazebo packages are now optional and only built if the required dependencies are installed
  * Made ros_ign_interfaces optional in irobot_create_ignition_bringup
  * fix license and minor changes to CMake and README
  * Interface buttons mock publisher not used in Ignition sim
  * Fixed linter errors
  Co-authored-by: Alberto Soragna <alberto.soragna@gmail.com>
* Contributors: Justin Kearns, roni-kreinin
