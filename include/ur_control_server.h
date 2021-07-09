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
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#define deg2rad(x) (x*M_PI/180.0)
#define RES 0.01
#define TRANS_TOLER 0.001 // 1 mm
#define ANGLE_TOLER 0.01745 // 1 degree

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

class RobotArm {
 public:
  enum ModeEnum{
    GOTO_JOINT, 
    GOTO_POSE, 
    LINEAR_MOVE, 
    VELOCITY_CTRL
  };
 private:
  // Varaibles
  bool _lock;
  int mode; // last control request mode
  int num_sols;
  const double dt; // Control preiod for control
  double joint[6]; // shoulder-lift/ shoulder-pan/ elbow/ wrist 1/ wrist 2/ wrist 3
  double tool_length;
  double curr_det;
  double _lock_det;
  bool initialized;
  bool self_collision;
  std::string tf_prefix;
  std::string action_server_name;
  std::vector<int> conversion;
  std::vector<std::string> joint_names;
  std::map<std::string, double> max_speed_param;
  std::vector<double> max_speed;
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
  // Services
  ros::ServiceServer goto_pose_srv;
  ros::ServiceServer linear_move_srv;
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
  planning_scene::PlanningScenePtr planning_scene;
  // Private Functions
  /*
   *  Subscriber callback, update joints' value
   */
  /*
   *  Convert pose to transformation matrix
   *  Input:
   *    geometry_msgs::Pose pose: input pose
   *    double *T: output placeholder
   *  Output: None
   */
  void PoseToDH(geometry_msgs::Pose pose, double *T);
  void JointStateCallback(const sensor_msgs::JointState &msg);
  /*
   *  Perform inverse kinematic and return the optimized joints solution
   *  Input:
   *    geometry_msgs::Pose target_pose: target pose
   *    double *sol: output placeholder of size-6 double array
   *    double init_jacob: Jacobian determinant at initial pose, 0 for ignoring sign conversion
   *  Output:
   *    int: number of IK solutions from ur_kinematics::inverse with NaN removed
   */
  int PerformIK(geometry_msgs::Pose target_pose, double *sol, double curr_det);
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
   *    double factor: joint velocity factor, the larger the faster, default is 1.0
   *  Output:
   *    double: time to execute this trajectory
   */
  double calculate_time(const double *now, const double *togo, double factor=1.0);
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
  /*
   *  Check if self-collision will happend in this robot state
   *  Input:
   *    const std::vector<double> *togo: target pose joint position array, in joint order: 1, 2, 3, 4, 5, 6
   *  Output:
   *    bool: true if collision, false otherwise
   */
  bool _checkSelfCollision(const std::vector<double> togo);
  /*
   *  Get Jacobian determinant of this robot state
   *  Input:
   *    const std::vector<double> togo: target joint position to compute
   *  Output:
   *    double: Jacobian matrix determinant
   */
  double _getJacobianDet(const std::vector<double> togo);
  /*
   *  Lock the robot and record current determinant value
   */
  void _lockRobot(void);
  /*
   *  Conpute translation and angle difference between current and target pose
   *  Input:
   *    const geometry_msgs::Pose target: target pose
   *  Outout:
   *    double &trans_diff: translation difference
   *    double &angle_diff: angle difference
   */
  void _compute_pose_diff(const geometry_msgs::Pose target, double &trans_diff, double &angle_diff);
 public:
   RobotArm(ros::NodeHandle nh, ros::NodeHandle pnh);
   ~RobotArm();
   // Service server callback
  bool GotoPoseService(arm_operation::target_pose::Request &req, arm_operation::target_pose::Response &res);
  bool LinearMoveService(arm_operation::target_pose::Request &req, arm_operation::target_pose::Response &res);
  bool GotoJointPoseService(arm_operation::joint_pose::Request &req, arm_operation::joint_pose::Response &res);
  bool VelocityControlService(arm_operation::velocity_ctrl::Request &req, arm_operation::velocity_ctrl::Response &res);
};

#endif
