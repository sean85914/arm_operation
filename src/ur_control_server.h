/*
 *  Subscribed topics:
 *    ~joint_states: joint state of universal robot
 *  Action interface:
 *    /follow_joint_trajectory
 *  Services:
 *    ~ur5_control/goto_pose
 *    ~ur5_control/go_straight
 *    ~ur5_control/goto_joint_pose
 *  Parameters:
 *    ~tool_length
 *    ~sim
 *    ~prefix
 *    ~wrist1_upper_bound
 *    ~wrist1_lower_bound
 *    ~wrist2_upper_bound
 *    ~wrist2_lower_bound
 *    ~wrist3_upper_bound
 *    ~wrist3_lower_bound
 */
#ifndef UR_CONTROL_SERVER_H
#define UR_CONTROL_SERVER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <arm_operation/target_pose.h>
#include <arm_operation/joint_pose.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <Eigen/Dense>
#include <ur_kin.h>

#define deg2rad(x) (x*M_PI/180.0)
#define NUMBEROFPOINTS 10

typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;

class RobotArm {
 private:
  // Varaibles
  int num_sols;
  double joint[6];
  double tool_length;
  double wrist1_upper_bound, wrist1_lower_bound;
  double wrist2_upper_bound, wrist2_lower_bound;
  double wrist3_upper_bound, wrist3_lower_bound;
  bool sim;
  bool wrist1_collision;
  bool wrist2_collision;
  bool wrist3_collision;
  std::string prefix;
  // ROS
  // Node handle
  ros::NodeHandle nh_, pnh_;
  // Subscriber
  ros::Subscriber sub_joint_state;
  // Services
  ros::ServiceServer goto_pose_srv;
  ros::ServiceServer go_straight_srv;
  ros::ServiceServer goto_joint_pose_srv;
  TrajClient *traj_client;
  control_msgs::FollowJointTrajectoryGoal goal; 
  control_msgs::FollowJointTrajectoryGoal straight;
  // Private Functions
  /* 
   *  Convert input joint angle to branch [-pi, pi]
   *  Input:
   *    double angle: input joint angle
   *  Output:
   *    double: convert angle to branch [-pi, pi]
   */
  double validAngle(double angle);
  /*
   *  Subscriber callback, update joint values
   */
  void JointStateCallback(const sensor_msgs::JointState &msg);
  /*
   *  Convert pose to transformation matrix
   *  Input:
   *    geometry_msgs::Pose pose: input pose
   *    double *T: output placeholder
   *  Output: None
   */
  void PoseToDH(geometry_msgs::Pose pose, double *T);
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
   *  Check if input wrist angle will self-collision
   *  Input:
   *    double &joint: wrist angle
   *    double upper: corresponding wrist upper bound
   *    double lower: corresponding wrist lower bound
   *  Output:
   *    bool: true if collision happened, false otherwise
   */
  bool wrist_check_bound(double &joint, double upper, double lower);
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
   *    int: time to execute this trajectory
   */
  int calculate_time(const double *now, const double *togo, double factor=0.5);
  /*
   * Get traejctory execution state
   */
  actionlib::SimpleClientGoalState getState();
  /*
   *  Start trajectory with given goal
   *  Input:
   *    control_msgs::FollowJointTrajectoryGoal goal: target trajectory goal
   *  Output: None
   */
  void StartTrajectory(control_msgs::FollowJointTrajectoryGoal goal);
  /*
   *  Make robot arm go to target pose
   *  Input:
   *    geometry_msgs::Pose pose: target pose
   *    double factor: execution speed factor (the larget the faster)
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
   bool GotoJointPoseService(arm_operation::joint_pose::Request  &req, arm_operation::joint_pose::Response &res);
};

#endif
