# arm_operation package
## Dependencies
* ROS Melodic
* Qt5
* Moveit
* [Universal_Robot_Client_Library](https://github.com/UniversalRobots/Universal_Robots_Client_Library#plain-cmake)
* [Universal_Robot_ROS_Driver](https://github.com/sean85914/Universal_Robots_ROS_Driver#building)
* [universal_robot](https://github.com/sean85914/universal_robot)

## Building
Clone this repo into your workspace
> $ cd ~/path/to/your/ws/src && git clone -b e_series https://github.com/sean85914/arm_operation.git  
> $ rosdep install --from-paths src --ignore-src -r -y  
> $ cd ~/path/to/your/ws && catkin_make && source devel/setup.bash

## ROS Node:
### urX_control_server (Currently support UR5e)
* Advertise some useful services for controlling the robot, such as set TCP link of the robot arm to user given cartesian/joint pose, etc.
  * Services
    * [~ur_control/goto_pose](https://github.com/sean85914/arm_operation/blob/e_series/srv/target_pose.srv)
    * [~ur_control/linear_move](https://github.com/sean85914/arm_operation/blob/e_series/srv/target_pose.srv)
    * [~ur_control/goto_joint_pose](https://github.com/sean85914/arm_operation/blob/e_series/srv/joint_pose.srv)
    * [~ur_control/velocity_control](https://github.com/sean85914/arm_operation/blob/e_series/srv/velocity_ctrl.srv)  
      **Note** This is not real velocity control (speedl or speedj in URScript), but using position control to simulate velocity control
  * Subscribed Topics
    * [~joint_states](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html)
  * Published Topics
    * [~det](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32.html) Current robot Jacobian matrix determinant
    * [~pose](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html) Current `TCP` pose
  * Parameters
    * ~tool_length (double): length from end effector to TCP, default is 0.0
    * ~tf_prefix (string): joint name prefix, default is ""
    * ~action_server_name (string): action server name for [trajectory control](http://docs.ros.org/en/api/control_msgs/html/action/FollowJointTrajectory.html)
    * ~maximum_speed (dict): allowed joint maximum speed

## Message Type:
* [joint_value.msg](https://github.com/sean85914/arm_operation/blob/e_series/msg/joint_value.msg)
  * float32[6] joint_value

## Service Type:
* [joint_pose.srv](https://github.com/sean85914/arm_operation/blob/e_series/srv/joint_pose.srv)
  * Request:
    * [arm_operation/joint_value](https://github.com/sean85914/arm_operation/blob/e_series/msg/joint_value.msg) target_joint
  * Response:
    * (string) plan_result
* [target_pose.srv](https://github.com/sean85914/arm_operation/blob/e_series/srv/target_pose.srv)
  * Request:
    * [geometry_msgs/Pose](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/Pose.html) target_pose
    * (float32) factor: executation speed factor, the smaller the faster
  * Response:
    * (string) plan_result
* [velocity_ctrl.srv](https://github.com/sean85914/arm_operation/blob/e_series/srv/velocity_ctrl.srv)
  * Request:
    * [geometry_msgs/Twist](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/Twist.html) twist
    * (float32) duration: How long should this twist apply
    * (bool) frame: true for ee_link, false for base_link
  * Response:
    * (string) plan_result

# Use for Real Robot
For real robot, follow the preparation in [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver#prepare-the-robot), conect the Ethernet cable to your PC and run
> $ roslaunch arm_operation urX_real.launch robot_ip:=[ur_robot_ip]

# Use for Robot in Simulation
[TODO]

# TODO
* Fix velocity control link in real robot
* Implement real velocity control
