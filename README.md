# arm_operation package
* Nodes:
  * ur5_control_server
    * Some useful services, such as set TCP link of the robot arm to user given cartesian/joint pose, etc.
    * Services
      * ~ur5_control/goto_pose (arm_operation::target_pose)
      * ~ur5_control/go_straight (arm_operation::target_pose)
      * ~ur5_control/goto_joint_pose (arm_operation::joint_pose)
    * Subscribe topics
      * [~joint_states](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html)
    * Actions
      * [/follow_joint_trajectory](http://docs.ros.org/api/control_msgs/html/action/FollowJointTrajectory.html) if sim is false
      * [/arm_controller/follow_joint_trajectory](http://docs.ros.org/api/control_msgs/html/action/FollowJointTrajectory.html) if sim is true
    * Parameters
      * ~tool_length: tool length, default is 0.18 (robotiq 2-finder gripper)
      * ~sim: true if using simulation, default is false
      * ~prefix: joint name prefix, default is ""
      * ~wrist1_upper_bound: wrist 1 constraint upper bound, default is -30 deg
      * ~wrist1_lower_bound: wrist 1 constraint lower bound, default is -240 deg
      * ~wrist2_upper_bound: wrist 2 constraint upper bound, default is 0
      * ~wrist2_lower_bound: wrist 2 constraint lower bound, default is -3.14
      * ~wrist3_upper_bound: wrist 3 constraint upper bound, default is 5 deg
      * ~wrist3_lower_bound: wrist 3 constraint lower bound, default is -220 deg
  * get_current_pose
    * Record current joint pose and save it in given file, press 'e' to exit and 's' to save file
    * Subscribe topics
      * [/joint_states](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html)
    * Parameters
      * ~file_name: file name without its extension
  * set_pose
    * Subscribe topics:
      * [~index_to_go](http://docs.ros.org/melodic/api/std_msgs/html/msg/Int16.html)
    * Service Client
      * /ur3_control_server/ur_control/goto_joint_pose (joint_pose.srv)
    * Parameter:
      * ~file_name: input file with extension
* Services:
  * joint_pose.srv
    * Request:
      * [float64[6]](http://docs.ros.org/jade/api/std_msgs/html/msg/Float64.html) joint
    * Response:
      * [string](http://docs.ros.org/jade/api/std_msgs/html/msg/String.html) plan_result
  * target_pose.srv
    * Request:
      * [geometry_msgs/Pose](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/Pose.html) target_pose
      * [float32](http://docs.ros.org/jade/api/std_msgs/html/msg/Float32.html) factor
    * Response:
      * [string](http://docs.ros.org/jade/api/std_msgs/html/msg/String.html) plan_result
      
      
## How to use
Clone this repo into your catkin_ws
> $ cd ~/path/to/your/ws/src && git clone https://github.com/sean85914/arm_operation.git  
> $ cd ~/path/to/your/ws && catkin_make && source devel/setup.bash


To simulate UR5 in Gazebo
> $ roslaunch ur_gazebo ur5.launch  
> $ roslaunch arm_operation ur5_sim.launch

For real robot,
> $ roslaunch ur_moder_driver urX_bringup.launch robot_ip=[your_robot_ip]  
> $ roslaunch arm_operation urX_real.launch tool_length:=[tool_length]

