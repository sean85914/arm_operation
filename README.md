# arm_operation package
* Nodes:
  * ur5_control_server
    * Some useful services, such as set TCP link of the robot arm to user given cartesian/joint pose, return to predefined pose, loose robotiq 2-finger gripper, etc.
    * Services
      * /ur5_control/goto_pose (arm_operation::target_pose)
      * [/ur5_control/goto_stby_place](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html)
      * [/ur5_control/goto_stby_pick](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html)
      * /ur5_control/go_straight (arm_operation::target_pose)
      * /ur5_control/goto_joint_pose (arm_operation::joint_pose)
      * [/ur5_control/loose_gripper](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html)
    * Publish topics
      * [/CModelRobotOutput](http://docs.ros.org/hydro/api/robotiq_c_model_control/html/msg/CModel_robot_output.html)
    * Subscribe topics
      * [/joint_states](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html)
    * Actions
      * [/follow_joint_trajectory](http://docs.ros.org/api/control_msgs/html/action/FollowJointTrajectory.html)
    * Parameters
* Services:
  * joint_pose.srv
    * Request:
      * [float64](http://docs.ros.org/jade/api/std_msgs/html/msg/Float64.html) joint1
      * [float64](http://docs.ros.org/jade/api/std_msgs/html/msg/Float64.html) joint2
      * [float64](http://docs.ros.org/jade/api/std_msgs/html/msg/Float64.html) joint3
      * [float64](http://docs.ros.org/jade/api/std_msgs/html/msg/Float64.html) joint4
      * [float64](http://docs.ros.org/jade/api/std_msgs/html/msg/Float64.html) joint5
      * [float64](http://docs.ros.org/jade/api/std_msgs/html/msg/Float64.html) joint6
    * Response:
      * [string](http://docs.ros.org/jade/api/std_msgs/html/msg/String.html) plan_result
  * target_pose.srv
    * Request:
      * [geometry_msgs/Pose](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/Pose.html)
      * [int32](http://docs.ros.org/jade/api/std_msgs/html/msg/Int32.html) finge_dist
      * [float32](http://docs.ros.org/jade/api/std_msgs/html/msg/Float32.html) factor
    * Response:
      * [string](http://docs.ros.org/jade/api/std_msgs/html/msg/String.html) plan_result
