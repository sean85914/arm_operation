/*
 *  CHANGELOG
 *  2021.06.24 Add Moveit function
 *  2021.06.27 Remove subscribe to wrench and stop monitor robot state
 */

/*
 *  Some useful services to control UR robot through follow joint trajectory action
 *  Publish topics:
 *    ~det: current determinant of robot jacobian
 *    ~pose: tool pose w.r.t. base link coordinate
 *  Subscribed topics:
 *    ~joint_states: joint state of universal robot
 *  Action interface:
 *    /follow_joint_trajectory, if `action_server_name` no given
 *  Services:
 *    ~ur5_control/goto_pose: move robot TCP to given target pose in Cartesian space
 *    ~ur5_control/go_straight: move robot TCP from current pose to target pose straightly
 *    ~ur5_control/goto_joint_pose: move robot to given joint space
 *  Parameters:
 *    ~tool_length: length from ee_link to tcp_link
 *    ~prefix: joint name prefix
 *    ~wrist1_upper_bound: wrist 1 upper bound
 *    ~wrist1_lower_bound: wrist 1 lower bound
 *    ~wrist2_upper_bound: wrist 2 upper bound
 *    ~wrist2_lower_bound: wrist 2 lower bound
 *    ~wrist3_upper_bound: wrist 3 upper bound
 *    ~wrist3_lower_bound: wrist 3 lower bound
 *    ~action_server_name: action server name to connect with
 */
#ifndef UR_CONTROL_SERVER_H
#define UR_CONTROL_SERVER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <cmath>
#include <Eigen/Dense>
#include <ur_kin.h>
// MSG
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/JointState.h>
// SRV
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <arm_operation/target_pose.h>
#include <arm_operation/joint_pose.h>
#include <arm_operation/velocity_ctrl.h>
#include <tf/transform_datatypes.h>
// Moveit
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#define deg2rad(x) (x*M_PI/180.0)
#define NUMBEROFPOINTS 10

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

class RobotArm {
 private:
  // Varaibles
  int num_sols;
  const double dt; // Control preiod for velocity control
  double joint[6];
  double tool_length;
  double wrist1_upper_bound, wrist1_lower_bound;
  double wrist2_upper_bound, wrist2_lower_bound;
  double wrist3_upper_bound, wrist3_lower_bound;
  double force_thres;// Higher than this value should cancel the goal DEPRECATED
  bool is_send_goal;
  bool initialized;
  bool wrist1_collision;
  bool wrist2_collision;
  bool wrist3_collision;
  std::string tf_prefix;
  std::string action_server_name;
  std::vector<int> conversion;
  std::vector<std::string> joint_names;
  std_msgs::Float32 det_msg;
  Eigen::Vector3d reference_position;
  geometry_msgs::Pose curr_tcp_pose;
  // ROS
  // Node handle
  ros::NodeHandle nh_, pnh_;
  // Publisher 
  ros::Publisher pub_pose;
  ros::Publisher pub_det;
  // Subscriber
  ros::Subscriber sub_joint_state;
  ros::Subscriber sub_robot_state;
  ros::Subscriber sub_wrench;
  // Services
  ros::ServiceServer goto_pose_srv;
  ros::ServiceServer go_straight_srv;
  ros::ServiceServer goto_joint_pose_srv;
  ros::ServiceServer vel_ctrl_srv;
  TrajClient *traj_client;
  //control_msgs::FollowJointTrajectoryGoal goal; 
  control_msgs::FollowJointTrajectoryGoal path;
  // Moveit
  robot_model_loader::RobotModelLoader robot_model_loader;
  robot_model::RobotModelPtr kinematic_model;
  robot_state::RobotStatePtr kinematic_state;
  robot_state::JointModelGroup* joint_model_group;
  // Private Functions
  /*
   *  Subscriber callback, update joints' value
   */
  void JointStateCallback(const sensor_msgs::JointState &msg);
  /*
   *  Convert pose to transformation matrix
   *  Input:
   *    geometry_msgs::Pose pose: input pose
   *    double *T: output placeholder
   *  Output: None
   */
  /*
   *  Perform inverse kinematic and return the optimized joints solution
   *  Input:
   *    geometry_msgs::Pose target_pose: target pose
   *    double *sol: output placeholder of size-6 double array
   *  Output:
   *    int: number of IK solutions
   */
  int PerformIK(geometry_msgs::Pose target_pose, double *sol);
  /*
   *  Perform inverse kinematic and return the optimized joints solution
   *  Will consider best wrist motion
   *  Input:
   *    geometry_msgs::Pose target_pose: target pose
   *    double *sol: output placeholder of size-6 double array
   *  Output:
   *    int: number of IK solutions
   */
  int PerformIKWristMotion(geometry_msgs::Pose target_pose, double *sol);
  /*
   *  Check if input wrist angle will self-collision
   *  Input:
   *    double &joint: wrist angle
   *    double upper: corresponding wrist upper bound
   *    double lower: corresponding wrist lower bound
   *  Output:
   *    bool: true if collision happened, false otherwise
   */
  inline bool wrist_check_bound(double &joint, double upper, double lower);
  /*
   *  Get current TCP pose
   *  Input: None
   *  Output:
   *    geometry_msgs::Pose: current TCP pose w.r.t. base link
   */
  geometry_msgs::Pose getCurrentTCPPose(void);
  /*
   *  Calculate execution time from start pose to target pose
   *  Input:
   *    const double *now: joint position array now
   *    const double *togo: target pose joint position array
   *    double factor: joint velocity factor, the larger the faster, default as 0.5
   *  Output:
   *    double: time to execute this trajectory
   */
  double calculate_time(const double *now, const double *togo, double factor=0.5);
  /*
   * Get trajectory execution state
   */
  inline actionlib::SimpleClientGoalState getState();
  /*
   *  Start trajectory with given goal
   *  Input:
   *    control_msgs::FollowJointTrajectoryGoal goal: target trajectory goal
   *  Output: None
   */
  inline void StartTrajectory(control_msgs::FollowJointTrajectoryGoal goal);
  /*
   *  Make robot arm go to target pose
   *  Input:
   *    geometry_msgs::Pose pose: target pose
   *    double factor: execution speed factor (the larger the faster)
   *  Output:
   *    control_msgs::FollowJointTrajectoryGoal
   */
  control_msgs::FollowJointTrajectoryGoal ArmToDesiredPoseTrajectory(geometry_msgs::Pose pose, double factor);
 public:
   RobotArm(ros::NodeHandle nh, ros::NodeHandle pnh);
   ~RobotArm();
   // Service server callback
   bool GotoPoseService(arm_operation::target_pose::Request &req, arm_operation::target_pose::Response &res);
   bool GoStraightLineService(arm_operation::target_pose::Request &req, arm_operation::target_pose::Response &res);
   bool GotoJointPoseService(arm_operation::joint_pose::Request &req, arm_operation::joint_pose::Response &res);
   bool VelocityControlService(arm_operation::velocity_ctrl::Request &req, arm_operation::velocity_ctrl::Response &res);
   void PoseToDH(geometry_msgs::Pose pose, double *T);
};

#endif
