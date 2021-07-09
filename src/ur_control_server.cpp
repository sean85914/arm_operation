#include "ur_control_server.h"

inline double makeMinorRotate(const double joint_now, const double joint_togo){
  bool sign = (joint_togo>0.0f);
  double dis_1 = std::abs(joint_now-joint_togo);
  double dis_2 = std::abs(joint_now-(joint_togo-(sign?2*M_PI:-2*M_PI)));
  // Choose smaller one
  if(dis_1<dis_2) return joint_togo;
  else return (joint_togo-(sign?2*M_PI:-2*M_PI));
}

/*
 * Check if `inStr` is ends with `key`, same as `str(inStr).endswith(key)` in Python
 */

inline bool endswith(const std::string inStr, const std::string key){
  if(inStr.length()>=key.length())
    return inStr.compare(inStr.length()-key.length(), key.length(), key) == 0;
  else
    return false;
}

/*
 * Check if given quaternion is valid, i.e., if the norm is smaller than a threshold
 */
inline bool isValidQuat(const geometry_msgs::Quaternion quat){
  double qx = quat.x, 
         qy = quat.y, 
         qz = quat.z, 
         qw = quat.w;
  return !(std::sqrt(qx*qx+qy*qy+qz*qz+qw*qw)<1e-3);
}

/* 
 *  Convert input joint angle to branch [-pi, pi]
 *  Input:
 *    double angle: input joint angle
 *  Output:
 *    double: convert angle to branch [-pi, pi]
 */
inline double validAngle(double angle){
  if(abs(angle) > 2*M_PI) angle = fmod(angle, 2*M_PI);
  if(angle > M_PI) angle -= 2*M_PI;
  else if(angle < -M_PI) angle += 2*M_PI;
  return angle;
}

/* Convert geometry_msgs::Pose to printable string */
std::string pose2string(const geometry_msgs::Pose pose){
  std::stringstream ss;
  ss << pose.position.x << ", " 
     << pose.position.y << ", " 
     << pose.position.z << ", " 
     << pose.orientation.x << ", " 
     << pose.orientation.y << ", " 
     << pose.orientation.z << ", " 
     << pose.orientation.w;
  return ss.str();
}

/* Convert joint array to printable string */
std::string joint2string(const double *in){
  std::stringstream ss;
  ss << in[0] << ", "
     << in[1] << ", "
     << in[2] << ", "
     << in[3] << ", "
     << in[4] << ", "
     << in[5];
  return ss.str();
}

// Public functions
RobotArm::RobotArm(ros::NodeHandle nh, ros::NodeHandle pnh): 
  nh_(nh), pnh_(pnh), num_sols(1), dt(0.1), curr_det(0.0), _lock(false), initialized(false), self_collision(false), reference_position(0.0, 0.0, 0.0){
  // Publisher 
  pub_pose = pnh_.advertise<geometry_msgs::PoseStamped>("pose", 1);
  pub_det = pnh_.advertise<std_msgs::Float32>("det", 100);
  // Subscriber
  sub_joint_state = pnh_.subscribe("joint_states", 100, &RobotArm::JointStateCallback, this);
  // Service server
  goto_pose_srv = pnh_.advertiseService("ur_control/goto_pose", &RobotArm::GotoPoseService, this);
  linear_move_srv = pnh_.advertiseService("ur_control/linear_move", &RobotArm::LinearMoveService, this);
  goto_joint_pose_srv = pnh_.advertiseService("ur_control/goto_joint_pose", &RobotArm::GotoJointPoseService, this);
  vel_ctrl_srv = pnh_.advertiseService("ur_control/velocity_control", &RobotArm::VelocityControlService, this);
  // Get parameters
  if(!pnh_.getParam("tf_prefix", tf_prefix)) tf_prefix = "";
  if(!pnh_.getParam("action_server_name", action_server_name))
    action_server_name = "/follow_joint_trajectory";
  pnh_.getParam("maximum_speed", max_speed_param);
  // Moveit
  robot_model_loader = robot_model_loader::RobotModelLoader("robot_description");
  kinematic_model  = robot_model_loader.getModel();
  planning_scene = planning_scene::PlanningScenePtr(new planning_scene::PlanningScene(kinematic_model));
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
  std::vector<std::string> groups = kinematic_model->getJointModelGroupNames();
  int group_idx = -1;
  for(int i=0; i<groups.size(); ++i){ // manipulator/endeffector
    if(kinematic_model->getJointModelGroup(groups[i])->getVariableNames().size()!=0)
      group_idx = i;
  }
  joint_model_group = kinematic_model->getJointModelGroup(groups[group_idx]);
  joint_names = joint_model_group->getVariableNames();
  ROS_INFO("[%s] Joint names inside [%s]", ros::this_node::getName().c_str(), groups[group_idx].c_str());
  for(int i=0; i<joint_names.size(); ++i)
    std::cout << "\t" << joint_model_group->getLinkModelNames()[i] << " <-> " << joint_names[i]  << "\n";
  // Show parameter information
  ROS_INFO("*********************************************************************************");
  ROS_INFO("[%s] tf_prefix: %s", ros::this_node::getName().c_str(), tf_prefix.c_str());
  ROS_INFO("[%s] Tool length: %f", ros::this_node::getName().c_str(), tool_length);
  ROS_INFO("[%s] Action server name: %s", ros::this_node::getName().c_str(), action_server_name.c_str());
  if(!max_speed_param.empty()){
    ROS_INFO("[%s] Joint maximum speed: ", ros::this_node::getName().c_str());
    for(auto it=max_speed_param.begin(); it!=max_speed_param.end(); ++it)
      printf("\t %s: %f\n", it->first.c_str(), it->second);
  }
  ROS_INFO("*********************************************************************************");
  // Tell the action client that we want to spin a thread by default
  traj_client = new TrajClient(action_server_name, true);
  // Wait for action server to come up
  while (!traj_client->waitForServer(ros::Duration(5.0)))
    ROS_INFO("[%s] Waiting for the %s server", ros::this_node::getName().c_str(), action_server_name.c_str());
  ROS_INFO("[%s] Action server connected!", ros::this_node::getName().c_str());
}

RobotArm::~RobotArm(){
  delete traj_client;
  printf("[%s] Node shutdown\n", ros::this_node::getName().c_str());
}

bool RobotArm::GotoPoseService(arm_operation::target_pose::Request &req, arm_operation::target_pose::Response &res){
  mode = GOTO_POSE;
  if(!initialized){
    res.plan_result = "robot not ready yet";
    ROS_WARN("[%s] %s", ros::this_node::getName().c_str(), res.plan_result.c_str());
    return true;
  }
  if(!isValidQuat(req.target_pose.orientation)){
    ROS_WARN("[%s] Receive invalid quaternion, abort request...", ros::this_node::getName().c_str());
    res.plan_result = "invalid target orientation";
    return true;
  }
  ROS_INFO("[%s] Receive new pose goal: %f %f %f %f %f %f %f", ros::this_node::getName().c_str(),
                                                               req.target_pose.position.x,
                                                               req.target_pose.position.y,
                                                               req.target_pose.position.z,
                                                               req.target_pose.orientation.x,
                                                               req.target_pose.orientation.y, 
                                                               req.target_pose.orientation.z, 
                                                               req.target_pose.orientation.w);
  double trans_diff, angle_diff;
  _compute_pose_diff(req.target_pose, trans_diff, angle_diff);
  if(trans_diff<TRANS_TOLER and angle_diff<ANGLE_TOLER){
    // Already in target pose
    res.plan_result = "done";
    return true;
  }
  ROS_INFO("[%s] Joint state now: %f %f %f %f %f %f", ros::this_node::getName().c_str(),
                                                      joint[0], joint[1], joint[2], joint[3], joint[4], joint[5]);
  auto traj = ArmToDesiredPoseTrajectory(req.target_pose, req.factor);
  if(num_sols == 0 and self_collision){
    res.plan_result = "target pose will self-collision"; 
  } else if(num_sols == 0 and !self_collision){
    res.plan_result = "target pose not reachable";
  } else{
    StartTrajectory(traj);
    res.plan_result = "done";
  }
  return true;
}

// TODO: check self collision
bool RobotArm::LinearMoveService(arm_operation::target_pose::Request &req, arm_operation::target_pose::Response &res){
  mode = LINEAR_MOVE;
  if(!initialized){
    res.plan_result = "robot not ready yet";
    ROS_WARN("[%s] %s", ros::this_node::getName().c_str(), res.plan_result.c_str());
    return true;
  }
  if(_lock){
    res.plan_result = "robot is lock since near the singular point, use goto_joint or goto_pose server to unlock robot";
    ROS_WARN("[%s] %s", ros::this_node::getName().c_str(), res.plan_result.c_str());
    return true;
  }
  if(!isValidQuat(req.target_pose.orientation)){
    ROS_WARN("[%s] Receive invalid quaternion, abort request...", ros::this_node::getName().c_str());
    res.plan_result = "invalid target pose";
    return true;
  }
  ROS_INFO("[%s] Receive new straight line goal: %f %f %f %f %f %f %f", ros::this_node::getName().c_str(),
                                                                        req.target_pose.position.x, 
                                                                        req.target_pose.position.y,
                                                                        req.target_pose.position.z,
                                                                        req.target_pose.orientation.x,
                                                                        req.target_pose.orientation.y,
                                                                        req.target_pose.orientation.z,
                                                                        req.target_pose.orientation.w);
  trajectory_msgs::JointTrajectory &l = path.trajectory;
  if(l.joint_names.size()==0){
    l.joint_names.resize(joint_names.size());
    for(int i=0; i<6; ++i)
      l.joint_names[i] = joint_names[conversion[i]];
  }
  double trans_diff, angle_diff;
  _compute_pose_diff(req.target_pose, trans_diff, angle_diff);
  if(trans_diff<TRANS_TOLER and angle_diff<ANGLE_TOLER){
    // Already in target pose
    res.plan_result = "done";
    return true;
  }
  int NUMBEROFPOINTS = (trans_diff>RES?ceil(trans_diff/RES):ceil(angle_diff/RES));
  printf("%d temporary waypoints\n", NUMBEROFPOINTS);
  l.points.resize(NUMBEROFPOINTS+1);
  for(int i=0; i<NUMBEROFPOINTS+1; ++i){
    l.points[i].positions.resize(joint_names.size());
    l.points[i].velocities.resize(joint_names.size());
  }
  geometry_msgs::Pose pose_now = curr_tcp_pose;
  double waypoint_sol_[NUMBEROFPOINTS * 6] = {0}, temp[6] = {0};
  tf::Quaternion q(pose_now.orientation.x, 
                   pose_now.orientation.y,
                   pose_now.orientation.z,
                   pose_now.orientation.w), 
                 q_inter_,
                 q_goal_(req.target_pose.orientation.x, 
                         req.target_pose.orientation.y,
                         req.target_pose.orientation.z,
                         req.target_pose.orientation.w);
  double _curr_det = curr_det; // save current Jacobian determinant as local variable, prevent from value changing
  for(int i=0; i<NUMBEROFPOINTS; ++i) {
    geometry_msgs::Pose waypoint;
    // Interpolated points
    waypoint.position.x = pose_now.position.x + (req.target_pose.position.x - pose_now.position.x) * (i+1) / double(NUMBEROFPOINTS);
    waypoint.position.y = pose_now.position.y + (req.target_pose.position.y - pose_now.position.y) * (i+1) / double(NUMBEROFPOINTS);
    waypoint.position.z = pose_now.position.z + (req.target_pose.position.z - pose_now.position.z) * (i+1) / double(NUMBEROFPOINTS);
    q_inter_ = q.slerp(q_goal_, (i+1) / double(NUMBEROFPOINTS));
    waypoint.orientation.x = q_inter_.getX();
    waypoint.orientation.y = q_inter_.getY();
    waypoint.orientation.z = q_inter_.getZ();
    waypoint.orientation.w = q_inter_.getW();
    PerformIK(waypoint, temp, _curr_det);
    if(num_sols == 0) {
      std::string err_msg = "[" + ros::this_node::getName() + "] Temporary waypoint (" + pose2string(waypoint)+ ") ";
      if(self_collision){
        err_msg += "will self-collision\n";
        ROS_ERROR("%s", err_msg.c_str());
        res.plan_result = "temporary waypoint self-collision";
      } else{
        err_msg += "can't find IK solution\n";
        ROS_ERROR("%s", err_msg.c_str());
        res.plan_result = "temporary waypoint not reachable";
      }
      return true;
    }/*else{
      std::vector<double> tmp(temp, temp+6);
      std::string info = "[ " + pose2string(waypoint) + "] [" + joint2string(temp) + "] " + std::to_string(_getJacobianDet(tmp));
      std::cout << info << "\n";
    }*/
    for(int j=0; j<6; ++j) {waypoint_sol_[i*6 + j] = temp[j];}
  }
  double total_time = calculate_time(joint, temp, req.factor); // Total time cost from initial to goal
  ROS_INFO("[%s] Execution time: %.1f seconds", ros::this_node::getName().c_str(), total_time);
  // Least Square to solve joints angular velocities
  // Assume: q(t) = a*t^2 + b*t + c
  // Then:   w(t) = 2a*t + b
  Eigen::MatrixXd A(NUMBEROFPOINTS+1, 3);
  Eigen::MatrixXd b(NUMBEROFPOINTS+1, 6);
  // Build matrix A and b
  for (int i = 0; i < NUMBEROFPOINTS; ++i) {
    double t = total_time * (i+1) / double(NUMBEROFPOINTS);
    A(i+1, 0) = t*t; A(i+1, 1) = t; A(i+1, 2) = 1;
    for (int j=0; j<6; ++j) {
      b(i+1, j) = waypoint_sol_[i*6 + j];
    }
  }
  A(0, 0) = 0; A(0, 1) = 0; A(0, 2) = 1;
  for(int j = 0; j<6; ++j) {b(0, j)=joint[j];}
  Eigen::MatrixXd x(3, 6); x = (A.transpose()*A).inverse()*A.transpose()*b;
  for (int i=0; i<NUMBEROFPOINTS; ++i) {
    double temp[6]={0};
    for (int j=0; j<6; ++j){
      l.points[i+1].positions[j] = waypoint_sol_[i*6 + conversion[j]];
      l.points[i+1].velocities[j] = 2* x(0, j) * total_time * (i+1) / double(NUMBEROFPOINTS) + x(1, conversion[j]);
      l.points[i+1].time_from_start = ros::Duration(total_time * (i+1) / double(NUMBEROFPOINTS));
    }
  }
  for (int i=0; i<6; ++i) {
    l.points[0].positions[i] = joint[conversion[i]];
    l.points[0].velocities[i] = 0;
    l.points[NUMBEROFPOINTS].velocities[i] = 0; 
    l.points[0].time_from_start = ros::Duration(0);
  }
  StartTrajectory(path);
  if(!_lock)
    res.plan_result = "done";
  else
    res.plan_result = "lock robot since close to singular point";
  return true;
}

bool RobotArm::GotoJointPoseService(arm_operation::joint_pose::Request &req, arm_operation::joint_pose::Response &res){
  mode = GOTO_JOINT;
  std::stringstream ss;
  /*
  for(int i=0; i<req.joints.size(); ++i){
    for(int j=0; j<6; ++j) 
      ss << std::fixed << std::setprecision(6) << req.joints[i].joint_value[j] << " ";
    ss << "\n";
  }*/
  for(int j=0; j<6; ++j) 
    ss << std::fixed << std::setprecision(6) << req.target_joint.joint_value[j] << " ";
  ss << "\n";
  ROS_INFO("[%s] Receive new joint pose request: %s", 
           ros::this_node::getName().c_str(), ss.str().c_str());
  if(!initialized){
    res.plan_result = "robot not ready yet";
    ROS_WARN("[%s] %s", ros::this_node::getName().c_str(), res.plan_result.c_str());
    return true;
  }
  control_msgs::FollowJointTrajectoryGoal tmp;
  trajectory_msgs::JointTrajectory &t = tmp.trajectory;
  t.joint_names.resize(6);
  for(int i=0; i<6; ++i)
    t.joint_names[i] = joint_names[conversion[i]];
  // Fifth-order polynomial
  double togo[6] = {0.0};
  for(int i=0; i<6; ++i)
    togo[i] = req.target_joint.joint_value[i]; 
  std::vector<double> _tmp(togo, togo+6);
  self_collision = _checkSelfCollision(_tmp);
  if(self_collision){
    res.plan_result = "target joint will self-collision";
    ROS_WARN("[%s] %s", ros::this_node::getName().c_str(), res.plan_result.c_str());
    return true;
  } 
  std::vector<double> joint_diff;
  for(int i=0; i<6; ++i)
    joint_diff.push_back(std::abs(togo[i]-joint[i]));
  if(/*check if all less than ANGLE_TOLER*/std::all_of(std::begin(joint_diff), std::end(joint_diff), [](double i){return (i<ANGLE_TOLER);})){
    // Already in target position
    res.plan_result = "done";
    return true;
  }
  double tt = calculate_time(joint, togo); // total time
  ROS_INFO("[%s] Execution time: %.1f seconds", ros::this_node::getName().c_str(), tt);
  int NUMBEROFPOINTS = tt/dt+1;
  t.points.resize(NUMBEROFPOINTS);
  for(int wp=0; wp<NUMBEROFPOINTS; ++wp){
    for(int idx=0; idx<6; ++idx){
      if(wp==0){ // first point
        t.points[0].positions.push_back(joint[conversion[idx]]);
        t.points[0].velocities.push_back(0.0);
      }else{
        double K = togo[conversion[idx]]-joint[conversion[idx]], time = dt*wp, _t = time/tt;
        t.points[wp].positions.push_back(6*K*pow(_t, 5)-15*K*pow(_t, 4)+10*K*pow(_t, 3)+joint[conversion[idx]]);
        t.points[wp].velocities.push_back(30*K/pow(tt, 5)*pow(time, 2)*pow(time-tt, 2));
      }
      t.points[wp].time_from_start = ros::Duration(dt*wp);
    }
  }
  StartTrajectory(tmp);
  res.plan_result = "done";
  return true;
}

// FIXME: sometimes the robot will vibrate back in simulator?
// FIXME: if the twist is too big, the final position may far from ideal one
bool RobotArm::VelocityControlService(arm_operation::velocity_ctrl::Request &req, arm_operation::velocity_ctrl::Response &res){
  mode = VELOCITY_CTRL;
  double vx = req.twist.linear.x, 
         vy = req.twist.linear.y, 
         vz = req.twist.linear.z, 
         wx = req.twist.angular.x, 
         wy = req.twist.angular.y, 
         wz = req.twist.angular.z, 
         t = req.duration;
  std::string frame = (req.frame?"ee_link":"base_link");
  ROS_INFO("[%s] Received velocity control request: (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f) in %.2f seconds w.r.t. %s", ros::this_node::getName().c_str(), vx, vy, vz, wx, wy, wz, t, frame.c_str());
  if(t<dt){
    ROS_WARN("[%s] Received duration less than control period (%f), ignore...", ros::this_node::getName().c_str(), dt);
    res.plan_result = "request duration less than control period";
    return true;
  }
  if(_lock){
    res.plan_result = "robot is lock since near the singular point, use goto_joint or goto_pose server to unlock robot";
    ROS_WARN("[%s] %s", ros::this_node::getName().c_str(), res.plan_result.c_str());
    return true;
  }
  if((vx==0.0) and (vy==0.0) and (vz==0.0) and (wx==0.0) and (wy==0.0) and (wz==0.0)){ // all zero
    res.plan_result = "done";
    return true;
  }
  const int STEPS = t/dt;
  control_msgs::FollowJointTrajectoryGoal goal;
  trajectory_msgs::JointTrajectory &traj = goal.trajectory;
  for(int i=0; i<6; ++i)
    traj.joint_names.push_back(joint_names[conversion[i]]);
  traj.points.resize(STEPS+1);
  for(int i=0; i<6; ++i){
    traj.points[0].positions.push_back(joint[i]);
    traj.points[0].velocities.push_back(0.0);
  }
  tf::Quaternion quat(curr_tcp_pose.orientation.x, 
                      curr_tcp_pose.orientation.y, 
                      curr_tcp_pose.orientation.z, 
                      curr_tcp_pose.orientation.w);
  tf::Matrix3x3 mat(quat);
  geometry_msgs::Pose final_pose = curr_tcp_pose;
  tf::Matrix3x3 del(1., -wz*dt, wy*dt, 
                    wz*dt, 1., -wx*dt, 
                    -wy*dt, wx*dt, 1.), 
                T, 
                T_final;
  T.setIdentity();
  for(int i=0; i<STEPS; ++i)
    T *= del;
  if(!req.frame){ // base, multiply ahead
    final_pose.position.x += vx*t;
    final_pose.position.y += vy*t;
    final_pose.position.z += vz*t;
    T_final = T*mat;
  }
  else{ // ee, multiply behind
    tf::Vector3 trans_base(vx*t, vy*t, vz*t), trans_ee = mat*trans_base;
    final_pose.position.x += trans_ee.getX();
    final_pose.position.y += trans_ee.getY();
    final_pose.position.z += trans_ee.getZ();
    T_final = mat*T;
  }
  tf::Quaternion _quat; 
  T_final.getRotation(_quat);
  _quat.normalize();
  final_pose.orientation.x = _quat.getX();
  final_pose.orientation.y = _quat.getY();
  final_pose.orientation.z = _quat.getZ();
  final_pose.orientation.w = _quat.getW();
  double tmp[8*6];
  if(!PerformIK(final_pose, tmp, curr_det)){
    res.plan_result = "robot can't reach final pose";
    ROS_WARN("[%s] %s", ros::this_node::getName().c_str(), res.plan_result.c_str());
    return true;
  }
  traj.points[0].time_from_start = ros::Duration(0.0);
  Eigen::VectorXd twist(6);
  // Convert back to base_link coordinate if frame == 0 
  if(!req.frame){
    tf::Vector3 vel_vec = mat.inverse() * tf::Vector3(vx, vy, vz), 
                rot_vec = mat.inverse() * tf::Vector3(wx, wy, wz);
    twist << vel_vec.getX(), vel_vec.getY(), vel_vec.getZ(), rot_vec.getX(), rot_vec.getY(), rot_vec.getZ();
  } else
    twist << vx, vy, vz, wx, wy, wz;
  std::vector<std::vector<double>> joints;
  joints.push_back(std::vector<double>(std::begin(joint), std::end(joint))); // put current joints as the first one
  // Copy another one moveit object
  robot_state::RobotStatePtr kinematic_state_ = kinematic_state; 
  robot_state::JointModelGroup* joint_model_group_ = joint_model_group;
  for(int i=0; i<STEPS; ++i){
    kinematic_state_->setFromDiffIK(joint_model_group_, twist, joint_model_group->getLinkModelNames().back() /* ee_link*/, dt);
    std::vector<double> _joint;
    kinematic_state_->copyJointGroupPositions(joint_model_group_, _joint);
    for(int j=0; j<6; ++j){
      traj.points[i+1].positions.push_back(_joint[conversion[j]]);
      const double *joint_vel = kinematic_state->getVariableVelocities();
      traj.points[i+1].velocities.push_back(joint_vel[conversion[j]]);
    }
    traj.points[i+1].time_from_start = ros::Duration((i+1)*dt);
  }
  StartTrajectory(goal);
  if(!_lock)
    res.plan_result = "done";
  else
    res.plan_result = "lock robot since close to singular point";
  return true;
}

// Private functions
void RobotArm::JointStateCallback(const sensor_msgs::JointState &msg){
  // Check incoming joint order
  if(!initialized){
    conversion.resize(joint_names.size());
    max_speed.resize(joint_names.size());
    printf("Conversion: ");
    for(int i=0; i<joint_names.size(); ++i){
      int j=0;
      for(; j<joint_names.size(); ++j){
        if(endswith(msg.name[i], joint_names[j]))
          break;
      }
      conversion[i] = j;
      auto it = max_speed_param.find(joint_names[j]);
      max_speed[i] = (it!=max_speed_param.end()?it->second:M_PI/2);
      printf("%d ", j);
    }
    printf("\nMaximum joint speed: ");
    for(int i=0; i<6; ++i)
      printf("%f ", max_speed[i]);
    printf("\n");
    initialized = true;
  }
  
  for(int i=0; i<6; ++i)
    joint[i] = msg.position[conversion[i]];
  // Compute determinant
  kinematic_state->setJointGroupPositions(joint_model_group, joint);
  Eigen::MatrixXd jacobian;
  kinematic_state->getJacobian(joint_model_group,
                               kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()) /*ee_link*/,
                               reference_position, jacobian);
  curr_det = jacobian.determinant();
  std_msgs::Float32 out_msg;
  out_msg.data = curr_det;
  pub_det.publish(out_msg);
  // Singularity watchdog
  if((mode==LINEAR_MOVE or mode==VELOCITY_CTRL) and getState().toString()=="ACTIVE" and !_lock and std::abs(curr_det)<0.035/*TODO*/){
    _lockRobot();
  }
  if((mode==GOTO_JOINT or mode==GOTO_POSE) and _lock){
    if(std::abs(curr_det)>_lock_det){
      _lock = false;
      ROS_INFO("[%s] robot unlocked", ros::this_node::getName().c_str());
    }
  }
  // Forward kinematics
  double T[16];
  curr_tcp_pose = getCurrentTCPPose();
  geometry_msgs::PoseStamped ps;
  ps.header.stamp = ros::Time::now();
  ps.header.frame_id = tf_prefix + "base_link";
  ps.pose = curr_tcp_pose;
  pub_pose.publish(ps);
}

void RobotArm::PoseToDH(geometry_msgs::Pose pose, double *T){
  double qx = pose.orientation.x,
         qy = pose.orientation.y, 
         qz = pose.orientation.z, 
         qw = pose.orientation.w;
  tf::Quaternion quat(qx, qy, qz, qw);
  quat.normalize();
  tf::Matrix3x3 mat(quat);
  for(int row=0; row<3; ++row){
    T[row*4] = mat[row].getX();
    T[row*4+1] = mat[row].getY();
    T[row*4+2] = mat[row].getZ();
  }
  T[3]  = pose.position.x; 
  T[7]  = pose.position.y; 
  T[11] = pose.position.z; 
  T[15] = 1.0;
}

int RobotArm::PerformIK(geometry_msgs::Pose target_pose, double *sol, double curr_det){
  double T[16] = {0};
  PoseToDH(target_pose, T);
  // tcp_link to ee_link since what we interested is tcp_link
  for (int i = 0; i < 3; ++i)
    T[i*4+3] -= tool_length*T[i*4];
  double q_sols[8*6], min = 1e6, dist = 0;
  int sols = ur_kinematics::inverse(T, q_sols), index = -1, valid_sols = sols;
  std::vector<bool> _self_collision(sols, false);
  for (int i = 0; i < sols; ++i) {
    // Check if NAN solution
    if(std::isnan(q_sols[i*6])){
      valid_sols-=1;
      _self_collision[i] = true; // invalid solution, set self_collision to true
      continue;
    }
    // Preprocess joint angle to -pi ~ pi
    for (int j = 0; j < 6; ++j) q_sols[i*6 + j] = validAngle(q_sols[i*6 + j]); 
    //for (int j = 0; j < 6; ++j) printf("%f ", q_sols[i*6+j]);
    //printf("\n");
    q_sols[i*6] = makeMinorRotate(joint[0], q_sols[i*6]);
    // Check if this robot state will self-collision
    std::vector<double> tmp;
    for(int j=0; j<6; ++j)
      tmp.push_back(q_sols[i*6+j]);
    _self_collision[i] = _checkSelfCollision(tmp);
    // Get determinant of Jacobian of this robot state
    double det = _getJacobianDet(tmp);
    // Check if corresponding angle for wrist
    for(int j=3; j<6; ++j) q_sols[i*6+j] = makeMinorRotate(joint[j], q_sols[i*6+j]);
    for (int j = 0; j < 6; ++j) {
      if(j<3) dist += pow((q_sols[i*6 + j] - joint[j])*1.5, 2); // For joint 1, 2, and 3, multiply by 1.5
      else dist += pow((q_sols[i*6 + j] - joint[j])*0.5, 2); // For joint 4, 5 and 6, multiply by 0.5
    }
    // Find solution with minimum joint angle difference and not self-collsion
    // For linear move, it will also check if singularity occurred (i.e., the sign of Jacobian determinant should not change)
    if(min>dist && !_self_collision[i] and (curr_det==0.0 or (curr_det!=0.0 and curr_det*det > 0.0))){
      min = dist;
      index = i;
    } dist = 0; 
  } 
  if(sols!=0)
    self_collision = /*check if all true*/std::all_of(std::begin(_self_collision), std::end(_self_collision), [](bool i){return i;}); 
  else
    self_collision = false; // pose not reachable, set self_collision to false
  if(index == -1 and valid_sols!=0) {
    return 0; // All valid solutions will self-collision
  }
  for(int i=0; i<6; ++i) sol[i] = q_sols[index*6+i];
  return (num_sols = valid_sols);
}

geometry_msgs::Pose RobotArm::getCurrentTCPPose(void){
  geometry_msgs::Pose pose_now;
  double T[16] = {0};
  ur_kinematics::forward(joint, T);
  tf::Quaternion q;
  tf::Matrix3x3 mat;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      mat[i][j] = T[i*4 + j];
    }
  }
  mat.getRotation(q);
  // ee_link to tcp_link
  pose_now.position.x = T[3] + T[0] * tool_length;
  pose_now.position.y = T[7] + T[4] * tool_length;
  pose_now.position.z = T[11] + T[8] * tool_length;
  pose_now.orientation.x = q.getX()*(q.getW()>=0?1.0:-1.0);
  pose_now.orientation.y = q.getY()*(q.getW()>=0?1.0:-1.0);
  pose_now.orientation.z = q.getZ()*(q.getW()>=0?1.0:-1.0);
  pose_now.orientation.w = q.getW()*(q.getW()>=0?1.0:-1.0); // Convert to positive qw
  return pose_now;
}

// TODO: Combine with Jacobian matrix to compute reasonable time for LINEAR_MOVE/GOTO_POSE?
double RobotArm::calculate_time(const double *now, const double *togo, double factor){
  double time;
  if(factor == 0.0) {ROS_WARN("Invalid factor, set to 1.0"); factor = 1.0;}
  std::vector<double> diff(joint_names.size(), 0.0);
  for(int i=0; i<joint_names.size(); ++i){
    diff[i] = std::abs(togo[i]-now[i])/max_speed[i];
  }
  auto it = std::max_element(diff.begin(), diff.end());
  time = *it*15./8.*factor;
  // round to first decimal
  time = round(time*10.)/10.;
  return time;
}

inline actionlib::SimpleClientGoalState RobotArm::getState() {
  return traj_client->getState();
}

inline void RobotArm::StartTrajectory(control_msgs::FollowJointTrajectoryGoal goal){
  goal.trajectory.header.stamp = ros::Time::now();
  ROS_INFO("[%s] Trajectory start!", ros::this_node::getName().c_str());
  traj_client->sendGoal(goal);
  // Wait for ur to finish joint trajectory
  while(!getState().isDone() && ros::ok()) {usleep(100000); ros::spinOnce();}
  ROS_INFO("[%s] Trajectory completed!", ros::this_node::getName().c_str());
}

control_msgs::FollowJointTrajectoryGoal RobotArm::ArmToDesiredPoseTrajectory(geometry_msgs::Pose pose, double factor){
  control_msgs::FollowJointTrajectoryGoal goal;
  trajectory_msgs::JointTrajectory &t = goal.trajectory;
  for(int i=0; i<6; ++i)
    t.joint_names.push_back(joint_names[conversion[i]]);
  t.points.resize(2);
  // Get closest joint space solution
  double sol[6] = {0};
  if(!PerformIK(pose, sol, 0.0)) {
    ROS_WARN("[%s] Cannot find IK solution!", ros::this_node::getName().c_str());
  }else{
    for (int i = 0; i < 6; ++i) {
      t.points[0].positions.push_back(joint[conversion[i]]);
      t.points[1].positions.push_back(sol[conversion[i]]); 
      t.points[0].velocities.push_back(0.0);
      t.points[1].velocities.push_back(0.0);
    }
    t.points[0].time_from_start = ros::Duration(0.01);
    double exeute_time = calculate_time(joint, sol, factor);
    t.points[1].time_from_start = ros::Duration(exeute_time);
    ROS_INFO("[%s] Execution time: %.1f seconds", ros::this_node::getName().c_str(), exeute_time);
  }
  
  return goal;
}

bool RobotArm::_checkSelfCollision(const std::vector<double> togo){
  robot_state::RobotState _state(kinematic_model); // copy an RobotState object
  for(int idx=0; idx<togo.size(); ++idx)
    _state.setVariablePosition(idx, togo[idx]);
  planning_scene->setCurrentState(_state);
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  planning_scene->checkSelfCollision(collision_request, collision_result);
  return collision_result.collision;
}

double RobotArm::_getJacobianDet(const std::vector<double> togo){
  robot_state::RobotState _state(kinematic_model); // copy an RobotState object
  for(int idx=0; idx<togo.size(); ++idx)
    _state.setVariablePosition(idx, togo[idx]);
  Eigen::MatrixXd jacobian;
  _state.getJacobian(joint_model_group,
                     kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()) /*ee_link*/,
                     reference_position, jacobian);
  return jacobian.determinant();
}

void RobotArm::_lockRobot(void){
  // cancel goal
  ROS_ERROR("[%s] Current Jacobian determinant to small (%f), cancel goal and lock the robot", ros::this_node::getName().c_str(), curr_det);
  traj_client->cancelGoal();
  _lock = true;
  _lock_det = curr_det;
}

void RobotArm::_compute_pose_diff(const geometry_msgs::Pose target, double &trans_diff, double &angle_diff){
  tf::Vector3 curr_position(curr_tcp_pose.position.x, curr_tcp_pose.position.y, curr_tcp_pose.position.z), 
              tgt_position(target.position.x, target.position.y, target.position.z),
              diff_vec = curr_position - tgt_position;
  tf::Quaternion curr_quat(curr_tcp_pose.orientation.x, curr_tcp_pose.orientation.y, curr_tcp_pose.orientation.z, curr_tcp_pose.orientation.w), 
                 tgt_quat(target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w),
                 diff_quat = tgt_quat*curr_quat.inverse();
  trans_diff = diff_vec.length();
  angle_diff = diff_quat.getAngle();
}
