```bash
04:15:44 borg@borg revisao2020 ±|master ✗|→ rospack list | grep -i turtlebot3
open_manipulator_with_tb3_description /home/borg/catkin_ws/src/turtlebot3_manipulation_tb3/open_manipulator_with_tb3_description
open_manipulator_with_tb3_msgs /home/borg/catkin_ws/src/turtlebot3_manipulation_tb3/open_manipulator_with_tb3_msgs
open_manipulator_with_tb3_tools /home/borg/catkin_ws/src/turtlebot3_manipulation_tb3/open_manipulator_with_tb3_tools
open_manipulator_with_tb3_waffle_moveit /home/borg/catkin_ws/src/turtlebot3_manipulation_tb3/open_manipulator_with_tb3_waffle_moveit
open_manipulator_with_tb3_waffle_pi_moveit /home/borg/catkin_ws/src/turtlebot3_manipulation_tb3/open_manipulator_with_tb3_waffle_pi_moveit
turtlebot3_applications_msgs /opt/ros/melodic/share/turtlebot3_applications_msgs
turtlebot3_automatic_parking /home/borg/catkin_ws/src/turtlebot3_applications/turtlebot3_automatic_parking
turtlebot3_automatic_parking_vision /home/borg/catkin_ws/src/turtlebot3_applications/turtlebot3_automatic_parking_vision
turtlebot3_autorace_camera /opt/ros/melodic/share/turtlebot3_autorace_camera
turtlebot3_autorace_control /opt/ros/melodic/share/turtlebot3_autorace_control
turtlebot3_autorace_core /opt/ros/melodic/share/turtlebot3_autorace_core
turtlebot3_autorace_detect /opt/ros/melodic/share/turtlebot3_autorace_detect
turtlebot3_bringup /home/borg/catkin_ws/src/turtlebot3/turtlebot3_bringup
turtlebot3_description /home/borg/catkin_ws/src/turtlebot3/turtlebot3_description
turtlebot3_example /home/borg/catkin_ws/src/turtlebot3/turtlebot3_example
turtlebot3_fake /opt/ros/melodic/share/turtlebot3_fake
turtlebot3_follow_filter /home/borg/catkin_ws/src/turtlebot3_applications/turtlebot3_follow_filter
turtlebot3_follower /home/borg/catkin_ws/src/turtlebot3_applications/turtlebot3_follower
turtlebot3_gazebo /opt/ros/melodic/share/turtlebot3_gazebo
turtlebot3_manipulation_bringup /home/borg/catkin_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_bringup
turtlebot3_manipulation_description /home/borg/catkin_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_description
turtlebot3_manipulation_gazebo /home/borg/catkin_ws/src/turtlebot3_manipulation_simulations/turtlebot3_manipulation_gazebo
turtlebot3_manipulation_gui /home/borg/catkin_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_gui
turtlebot3_manipulation_moveit_config /home/borg/catkin_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_moveit_config
turtlebot3_manipulation_navigation /home/borg/catkin_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_navigation
turtlebot3_manipulation_slam /home/borg/catkin_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_slam
turtlebot3_msgs /opt/ros/melodic/share/turtlebot3_msgs
turtlebot3_navigation /home/borg/catkin_ws/src/turtlebot3/turtlebot3_navigation
turtlebot3_panorama /opt/ros/melodic/share/turtlebot3_panorama
turtlebot3_slam /home/borg/catkin_ws/src/turtlebot3/turtlebot3_slam
turtlebot3_teleop /home/borg/catkin_ws/src/turtlebot3/turtlebot3_teleop

04:15:55 borg@borg revisao2020 ±|master ✗|→ apt list --installed | grep turtlebot3

WARNING: apt does not have a stable CLI interface. Use with caution in scripts.

ros-melodic-turtlebot3/bionic,now 1.2.2-1bionic.20200406.134636 amd64 [installed]
ros-melodic-turtlebot3-applications-msgs/bionic,now 1.0.0-1bionic.20200320.141228 amd64 [installed]
ros-melodic-turtlebot3-automatic-parking/bionic,now 1.1.0-0bionic.20200320.132517 amd64 [installed]
ros-melodic-turtlebot3-autorace/bionic,now 1.2.0-0bionic.20200320.153552 amd64 [installed]
ros-melodic-turtlebot3-autorace-camera/bionic,now 1.2.0-0bionic.20200320.135005 amd64 [installed]
ros-melodic-turtlebot3-autorace-control/bionic,now 1.2.0-0bionic.20200320.151434 amd64 [installed]
ros-melodic-turtlebot3-autorace-core/bionic,now 1.2.0-0bionic.20200320.111412 amd64 [installed]
ros-melodic-turtlebot3-autorace-detect/bionic,now 1.2.0-0bionic.20200320.150858 amd64 [installed]
ros-melodic-turtlebot3-bringup/bionic,now 1.2.2-1bionic.20200402.222853 amd64 [installed]
ros-melodic-turtlebot3-bringup-dbgsym/bionic,now 1.2.2-1bionic.20200402.222853 amd64 [installed]
ros-melodic-turtlebot3-description/bionic,now 1.2.2-1bionic.20200320.113528 amd64 [installed]
ros-melodic-turtlebot3-example/bionic,now 1.2.2-1bionic.20200406.134126 amd64 [installed]
ros-melodic-turtlebot3-fake/bionic,now 1.2.0-0bionic.20200402.223002 amd64 [installed]
ros-melodic-turtlebot3-fake-dbgsym/bionic,now 1.2.0-0bionic.20200402.223002 amd64 [installed]
ros-melodic-turtlebot3-follow-filter/bionic,now 1.1.0-0bionic.20200408.172513 amd64 [installed]
ros-melodic-turtlebot3-follower/bionic,now 1.1.0-0bionic.20200320.132529 amd64 [installed]
ros-melodic-turtlebot3-gazebo/now 1.2.0-0bionic.20200320.144257 amd64 [installed,upgradable to: 1.2.0-0bionic.20200501.202836]
ros-melodic-turtlebot3-gazebo-dbgsym/now 1.2.0-0bionic.20200320.144257 amd64 [installed,upgradable to: 1.2.0-0bionic.20200501.202836]
ros-melodic-turtlebot3-msgs/bionic,now 1.0.0-0bionic.20200304.005554 amd64 [installed]
ros-melodic-turtlebot3-navigation/bionic,now 1.2.2-1bionic.20200402.223304 amd64 [installed]
ros-melodic-turtlebot3-panorama/bionic,now 1.1.0-0bionic.20200402.223439 amd64 [installed]
ros-melodic-turtlebot3-panorama-dbgsym/bionic,now 1.1.0-0bionic.20200402.223439 amd64 [installed]
ros-melodic-turtlebot3-simulations/now 1.2.0-0bionic.20200402.223654 amd64 [installed,upgradable to: 1.2.0-0bionic.20200501.204018]
ros-melodic-turtlebot3-slam/bionic,now 1.2.2-1bionic.20200402.223305 amd64 [installed]
ros-melodic-turtlebot3-slam-dbgsym/bionic,now 1.2.2-1bionic.20200402.223305 amd64 [installed]
ros-melodic-turtlebot3-teleop/bionic,now 1.2.2-1bionic.20200320.105738 amd64 [installed]

04:19:33 borg@borg revisao2020 ±|master ✗|→ apt list -a ros-melodic-turtlebot3*
Listing... Done
ros-melodic-turtlebot3/bionic,now 1.2.2-1bionic.20200406.134636 amd64 [installed]

ros-melodic-turtlebot3-applications/bionic 1.1.0-0bionic.20200514.235302 amd64

ros-melodic-turtlebot3-applications-msgs/bionic,now 1.0.0-1bionic.20200320.141228 amd64 [installed]

ros-melodic-turtlebot3-automatic-parking/bionic,now 1.1.0-0bionic.20200320.132517 amd64 [installed]

ros-melodic-turtlebot3-automatic-parking-vision/bionic 1.1.0-0bionic.20200514.233738 amd64

ros-melodic-turtlebot3-autorace/bionic,now 1.2.0-0bionic.20200320.153552 amd64 [installed]

ros-melodic-turtlebot3-autorace-camera/bionic,now 1.2.0-0bionic.20200320.135005 amd64 [installed]

ros-melodic-turtlebot3-autorace-control/bionic,now 1.2.0-0bionic.20200320.151434 amd64 [installed]

ros-melodic-turtlebot3-autorace-core/bionic,now 1.2.0-0bionic.20200320.111412 amd64 [installed]

ros-melodic-turtlebot3-autorace-detect/bionic,now 1.2.0-0bionic.20200320.150858 amd64 [installed]

ros-melodic-turtlebot3-bringup/bionic,now 1.2.2-1bionic.20200402.222853 amd64 [installed]

ros-melodic-turtlebot3-bringup-dbgsym/bionic,now 1.2.2-1bionic.20200402.222853 amd64 [installed]

ros-melodic-turtlebot3-description/bionic,now 1.2.2-1bionic.20200320.113528 amd64 [installed]

ros-melodic-turtlebot3-example/bionic,now 1.2.2-1bionic.20200406.134126 amd64 [installed]

ros-melodic-turtlebot3-fake/bionic,now 1.2.0-0bionic.20200402.223002 amd64 [installed]

ros-melodic-turtlebot3-fake-dbgsym/bionic,now 1.2.0-0bionic.20200402.223002 amd64 [installed]

ros-melodic-turtlebot3-follow-filter/bionic,now 1.1.0-0bionic.20200408.172513 amd64 [installed]

ros-melodic-turtlebot3-follower/bionic,now 1.1.0-0bionic.20200320.132529 amd64 [installed]

ros-melodic-turtlebot3-gazebo/bionic 1.2.0-0bionic.20200501.202836 amd64 [upgradable from: 1.2.0-0bionic.20200320.144257]
ros-melodic-turtlebot3-gazebo/now 1.2.0-0bionic.20200320.144257 amd64 [installed,upgradable to: 1.2.0-0bionic.20200501.202836]

ros-melodic-turtlebot3-gazebo-dbgsym/bionic 1.2.0-0bionic.20200501.202836 amd64 [upgradable from: 1.2.0-0bionic.20200320.144257]
ros-melodic-turtlebot3-gazebo-dbgsym/now 1.2.0-0bionic.20200320.144257 amd64 [installed,upgradable to: 1.2.0-0bionic.20200501.202836]

ros-melodic-turtlebot3-msgs/bionic,now 1.0.0-0bionic.20200304.005554 amd64 [installed]

ros-melodic-turtlebot3-navigation/bionic,now 1.2.2-1bionic.20200402.223304 amd64 [installed]

ros-melodic-turtlebot3-panorama/bionic,now 1.1.0-0bionic.20200402.223439 amd64 [installed]

ros-melodic-turtlebot3-panorama-dbgsym/bionic,now 1.1.0-0bionic.20200402.223439 amd64 [installed]

ros-melodic-turtlebot3-simulations/bionic 1.2.0-0bionic.20200501.204018 amd64 [upgradable from: 1.2.0-0bionic.20200402.223654]
ros-melodic-turtlebot3-simulations/now 1.2.0-0bionic.20200402.223654 amd64 [installed,upgradable to: 1.2.0-0bionic.20200501.204018]

ros-melodic-turtlebot3-slam/bionic,now 1.2.2-1bionic.20200402.223305 amd64 [installed]

ros-melodic-turtlebot3-slam-dbgsym/bionic,now 1.2.2-1bionic.20200402.223305 amd64 [installed]

ros-melodic-turtlebot3-teleop/bionic,now 1.2.2-1bionic.20200320.105738 amd64 [installed]
```

