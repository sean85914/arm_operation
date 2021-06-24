#include "ur_control_server.h"

inline double makeMinorRotate(const double joint_now, const double joint_togo){
  bool sign = (joint_togo>0.0f);
  double dis_1 = std::abs(joint_now-joint_togo);
  double dis_2 = std::abs(joint_now-(joint_togo-(sign?2*M_PI:-2*M_PI)));
  // Choose small one
  if(dis_1<dis_2) return joint_togo;
  else return (joint_togo-(sign?2*M_PI:-2*M_PI));
}

inline bool endswith(const std::string inStr, const std::string key){
  if(inStr.length()>=key.length())
    return inStr.compare(inStr.length()-key.length(), key.length(), key) == 0;
  else
    return false;
}

// Public functions
RobotArm::RobotArm(ros::NodeHandle nh, ros::NodeHandle pnh): 
  nh_(nh), pnh_(pnh), num_sols(1), is_robot_enable(true), is_send_goal(false), initialized(false), new_topic(false), reference_position(0.0, 0.0, 0.0){
  // Publisher 
  pub_pose = pnh_.advertise<geometry_msgs::PoseStamped>("pose", 1);
  pub_det = pnh_.advertise<std_msgs::Float32>("det", 100);
  // Subscriber
  sub_joint_state = pnh_.subscribe("joint_states", 100, &RobotArm::JointStateCallback, this);
  sub_robot_state = pnh_.subscribe("/ur_driver/robot_mode_state", 1, &RobotArm::RobotModeStateCallback, this);
  sub_wrench      = pnh_.subscribe("/wrench", 1, &RobotArm::RobotWrenchCallback, this);
  // Service server
  goto_pose_srv = pnh_.advertiseService("ur_control/goto_pose", &RobotArm::GotoPoseService, this);
  go_straight_srv = pnh_.advertiseService("ur_control/go_straight", &RobotArm::GoStraightLineService, this);
  goto_joint_pose_srv = pnh_.advertiseService("ur_control/goto_joint_pose", &RobotArm::GotoJointPoseService, this);
  /*
  robot_state_srv = pnh_.advertiseService("ur_control/get_robot_state", &RobotArm::GetRobotModeStateService, this);
  unlock_protective_stop_srv = pnh_.advertiseService("ur_control/unlock_protective", &RobotArm::UnlockProtectiveStopService, this);
  stop_program_srv = pnh_.advertiseService("ur_control/stop_program", &RobotArm::StopProgramService, this);
  */
  // Parameters
  if(!nh_.getParam("tf_prefix", tf_prefix)) tf_prefix = "";
  if(!pnh_.getParam("tool_length", tool_length)) tool_length = 0.0;
  // Wrist1 default bound [-240, -30]
  if(!pnh_.getParam("wrist1_upper_bound", wrist1_upper_bound)) wrist1_upper_bound = deg2rad(-30);
  if(!pnh_.getParam("wrist1_lower_bound", wrist1_lower_bound)) wrist1_lower_bound = deg2rad(-240);
  // Wrist2 default bound [-pi, 0]
  if(!pnh_.getParam("wrist2_upper_bound", wrist2_upper_bound)) wrist2_upper_bound = 0;
  if(!pnh_.getParam("wrist2_lower_bound", wrist2_lower_bound)) wrist2_lower_bound = -M_PI;
  // Wrist3 default bound [-220, 5]
  if(!pnh_.getParam("wrist3_upper_bound", wrist3_upper_bound)) wrist3_upper_bound = deg2rad(5);
  if(!pnh_.getParam("wrist3_lower_bound", wrist3_lower_bound)) wrist3_lower_bound = deg2rad(-220);
  if(!pnh_.getParam("action_server_name", action_server_name))
    action_server_name = "/follow_joint_trajectory";
  // Moveit
  robot_model_loader = robot_model_loader::RobotModelLoader("robot_description");
  kinematic_model  = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
  std::vector<std::string> groups = kinematic_model->getJointModelGroupNames();
  int group_idx = -1;
  for(int i=0; i<groups.size(); ++i){
    if(kinematic_model->getJointModelGroup(groups[i])->getVariableNames().size()!=0)
      group_idx = i;
  }
  joint_model_group = kinematic_model->getJointModelGroup(groups[group_idx]);
  joint_names = joint_model_group->getVariableNames();
  ROS_INFO("[%s] Joint name inside [%s]", ros::this_node::getName().c_str(), groups[group_idx].c_str());
  for(int i=0; i<joint_names.size(); ++i)
    std::cout << "\t" << joint_names[i] << "\n";
  
  /*
  if(!pnh_.getParam("force_thres", force_thres)){
    force_thres = 100.0f;
    pnh_.setParam("force_thres", force_thres);
    ROS_WARN("[%s] Set force_thres with default value: %f", ros::this_node::getName().c_str(), force_thres);
  }
  */
  // Show parameter information
  ROS_INFO("*********************************************************************************");
  ROS_INFO("[%s] Tool length: %f", ros::this_node::getName().c_str(), tool_length);
  ROS_INFO("[%s] Action server name: %s", ros::this_node::getName().c_str(), action_server_name.c_str());
  ROS_INFO("[%s] Wrist 1 bound: [%f, %f]", ros::this_node::getName().c_str(), wrist1_lower_bound, wrist1_upper_bound);
  ROS_INFO("[%s] Wrist 2 bound: [%f, %f]", ros::this_node::getName().c_str(), wrist2_lower_bound, wrist2_upper_bound);
  ROS_INFO("[%s] Wrist 3 bound: [%f, %f]", ros::this_node::getName().c_str(), wrist3_lower_bound, wrist3_upper_bound);
  //ROS_INFO("[%s] Force thres: %f", ros::this_node::getName().c_str(), force_thres);
  ROS_INFO("*********************************************************************************");
  // Tell the action client that we want to spin a thread by default
  traj_client = new TrajClient(action_server_name, true);
  // Wait for action server to come up
  while (!traj_client->waitForServer(ros::Duration(5.0)))
    ROS_INFO("[%s] Waiting for the %s server", ros::this_node::getName().c_str(), action_server_name.c_str());
  ROS_INFO("[%s] Action server connected!", ros::this_node::getName().c_str());
  // Timer
  checkParameterTimer = pnh_.createTimer(ros::Duration(1.0f), &RobotArm::TimerCallback, this);
  
  trajectory_msgs::JointTrajectory &t = goal.trajectory;
  trajectory_msgs::JointTrajectory &l = path.trajectory;
  t.points.resize(2); l.points.resize(NUMBEROFPOINTS+1);
  for(int i=0; i<2; ++i){
    t.points[i].positions.resize(6);
    t.points[i].velocities.resize(6);
  } 
  for(int i=0; i<NUMBEROFPOINTS+1; ++i) {
    l.points[i].positions.resize(6);
    l.points[i].velocities.resize(6);
  }
}

RobotArm::~RobotArm(){
  delete traj_client;
}

bool RobotArm::GotoPoseService(arm_operation::target_pose::Request &req, arm_operation::target_pose::Response &res){
  double qx = req.target_pose.orientation.x, 
         qy = req.target_pose.orientation.y, 
         qz = req.target_pose.orientation.z, 
         qw = req.target_pose.orientation.w;
  if(std::sqrt(qx*qx+qy*qy+qz*qz+qw*qw) < 1e-3){
    ROS_WARN("[%s] Receive invalid quaternion, abort request...", ros::this_node::getName().c_str());
    res.plan_result = "Invalid target pose";
    return true;
  }
  ROS_INFO("[%s] Receive new pose goal: %f %f %f %f %f %f %f", ros::this_node::getName().c_str(),
                                                               req.target_pose.position.x,
                                                               req.target_pose.position.y,
                                                               req.target_pose.position.z,
                                                               qx, qy, qz, qw);
  ROS_INFO("[%s] Joint state now: %f %f %f %f %f %f", ros::this_node::getName().c_str(),
                                                      joint[0], joint[1], joint[2], joint[3], joint[4], joint[5]);
  if(!initialized){
    ROS_WARN("[%s] Robot not ready yet", ros::this_node::getName().c_str());
    res.plan_result = "robot_not_ready";
    return true;
  }
  if(!is_robot_enable){
    ROS_WARN("Robot is emergency/protective stop, abort request...");
    res.plan_result = "robot_disable"; return true;
  }
  auto traj = ArmToDesiredPoseTrajectory(req.target_pose, req.factor);
  if(num_sols == 0) {res.plan_result = "fail_to_find_solution"; return true;}
  StartTrajectory(traj);
  res.plan_result = "find_one_feasible_solution";
  return true;
}

// TODO
bool RobotArm::GoStraightLineService(arm_operation::target_pose::Request &req, arm_operation::target_pose::Response &res){
  ROS_INFO("[%s] Receive new straight line goal: %f %f %f %f %f %f %f", ros::this_node::getName().c_str(),
                                                                        req.target_pose.position.x, 
                                                                        req.target_pose.position.y,
                                                                        req.target_pose.position.z,
                                                                        req.target_pose.orientation.x,
                                                                        req.target_pose.orientation.y,
                                                                        req.target_pose.orientation.z,
                                                                        req.target_pose.orientation.w);
  if(!is_robot_enable){
    ROS_WARN("Robot is emergency/protective stop, abort request...");
    res.plan_result = "robot_disable"; return true;
  }
  trajectory_msgs::JointTrajectory &l = path.trajectory;
  if(l.joint_names.size()==0){
    l.joint_names.resize(6);
    for(int i=0; i<6; ++i)
      l.joint_names[i] = joint_names[conversion[i]];
  }
  geometry_msgs::Pose pose_now = getCurrentTCPPose();
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
    PerformIK(waypoint, temp);
    if(num_sols == 0) {
      ROS_ERROR("waypoint index: %d fail to find IK solution", i);
      res.plan_result = "fail_to_find_solution";
      return true;
    }
    for(int j=0; j<6; ++j) {waypoint_sol_[i*6 + j] = temp[j];}
  }
  double total_time = calculate_time(joint, temp, req.factor); // Total time cost from initial to goal
  ROS_INFO("[%s] Execution time: %.f seconds", ros::this_node::getName().c_str(), total_time);
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
      l.points[i+1].positions[j] = waypoint_sol_[i*6 + j];
      l.points[i+1].velocities[j] = 2* x(0, j) * total_time * (i+1) / double(NUMBEROFPOINTS) + x(1, j);
      l.points[i+1].time_from_start = ros::Duration(total_time * (i+1) / double(NUMBEROFPOINTS));
    }
  }
  for (int i=0; i<6; ++i) {
    l.points[0].positions[i] = joint[i];
    l.points[0].velocities[i] = 0;
    l.points[NUMBEROFPOINTS].velocities[i] = 0; 
    l.points[0].time_from_start = ros::Duration(0);
  }
  StartTrajectory(path);
  res.plan_result = "find_one_feasible_solution";
  return true;
}

bool RobotArm::GotoJointPoseService(arm_operation::joint_pose::Request  &req, arm_operation::joint_pose::Response &res){
  std::stringstream ss;
  for(int i=0; i<req.joints.size(); ++i){
    for(int j=0; j<6; ++j) 
      ss << std::fixed << std::setprecision(6) << req.joints[i].joint_value[j] << " ";
    ss << "\n";
  }
  ROS_INFO("[%s] Receive new joint pose request:\n\
Totally %d waypoints\n%s", ros::this_node::getName().c_str(), (int)req.joints.size(), ss.str().c_str());
  if(!initialized){
    ROS_WARN("[%s] Robot not ready yet", ros::this_node::getName().c_str());
    res.plan_result = "robot_not_ready";
    return true;
  }
  if(!is_robot_enable){
    ROS_WARN("Robot is emergency/protective stop, abort request...");
    res.plan_result = "robot_disable"; return true;
  }
  control_msgs::FollowJointTrajectoryGoal tmp;
  trajectory_msgs::JointTrajectory &t = tmp.trajectory;
  t.joint_names.resize(6);
  for(int i=0; i<6; ++i)
    t.joint_names[i] = joint_names[conversion[i]];
  t.points.resize(req.joints.size()+1);
  for(int i=0; i<=req.joints.size(); ++i){
    t.points[i].positions.resize(6);
    t.points[i].velocities.resize(6);
    for(int j=0; j<6; ++j){
      if(i==0){
        t.points[i].positions[j] = joint[conversion[j]];
        t.points[i].time_from_start = ros::Duration(0);
      } else{
        t.points[i].positions[j] = req.joints[i-1].joint_value[conversion[j]];
        double tmp_arr[6];
        std::copy(req.joints[i-1].joint_value.begin(),
                  req.joints[i-1].joint_value.end(), tmp_arr);
        t.points[i].time_from_start = ros::Duration(calculate_time(joint, tmp_arr));
      }
    }
  }
  StartTrajectory(tmp);
  return true;
}

bool RobotArm::GetRobotModeStateService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
  res.success = is_robot_enable;
  res.message = is_robot_enable?"robot enable":"robot disable";
  ROS_INFO("%s", res.message.c_str());
  return true;
}

bool RobotArm::UnlockProtectiveStopService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  ur_control.unlock_protective_stop();
  return true;
}

bool RobotArm::StopProgramService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  ur_control.stop_program();
  return true;
}


// DEPRECATED
/*
void RobotArm::pubPoseCallback(const ros::TimerEvent &event){
  if(!new_topic) return;
  double T[16];
  ur_kinematics::forward(joint, T);
  geometry_msgs::PoseStamped ps;
  ps.header.stamp = ros::Time::now();
  ps.header.frame_id = "base_link";
  tf::Matrix3x3 mat(T[0], T[1], T[2], T[4], T[5], T[6], T[8], T[9], T[10]);
  tf::Quaternion quat;
  mat.getRotation(quat);
  ps.pose.position.x = T[3];
  ps.pose.position.y = T[7];
  ps.pose.position.z = T[11];
  ps.pose.orientation.x = quat.getX();
  ps.pose.orientation.y = quat.getY();
  ps.pose.orientation.z = quat.getZ();
  ps.pose.orientation.w = quat.getW();
  pub_pose.publish(ps);
  new_topic = false;
}
*/

// Private functions

inline double RobotArm::validAngle(double angle){
  if(abs(angle) > 2*M_PI) angle = fmod(angle, 2*M_PI);
  if(angle > M_PI) angle -= 2*M_PI;
  else if(angle < -M_PI) angle += 2*M_PI;
  return angle;
}

void RobotArm::JointStateCallback(const sensor_msgs::JointState &msg){
  // Check incoming joint order
  if(!initialized){
    conversion.resize(6);
    printf("Conversion: ");
    for(int i=0; i<6; ++i){
      int j=0;
      for(; j<6; ++j){
        if(endswith(msg.name[i], joint_names[j]))
          break;
      }
      conversion[i] = j;
      printf("%d ", j);
    }
    printf("\n");
    initialized = true;
  }
  for(int i=0; i<6; ++i){
    joint[i] = msg.position[conversion[i]];
    new_topic = true;
  }
  // Compute determinant
  kinematic_state->setJointGroupPositions(joint_model_group, joint);
  Eigen::MatrixXd jacobian;
  kinematic_state->getJacobian(joint_model_group,
                               kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                               reference_position, jacobian);
  std_msgs::Float32 out_msg;
  out_msg.data = jacobian.determinant();
  pub_det.publish(out_msg);
  // Forward kinematics
  double T[16];
  ur_kinematics::forward(joint, T);
  geometry_msgs::PoseStamped ps;
  ps.header.stamp = ros::Time::now();
  ps.header.frame_id = tf_prefix + "base_link";
  tf::Matrix3x3 mat(T[0], T[1], T[2], T[4], T[5], T[6], T[8], T[9], T[10]);
  tf::Quaternion quat;
  mat.getRotation(quat);
  ps.pose.position.x = T[3];
  ps.pose.position.y = T[7];
  ps.pose.position.z = T[11];
  ps.pose.orientation.x = quat.getX();
  ps.pose.orientation.y = quat.getY();
  ps.pose.orientation.z = quat.getZ();
  ps.pose.orientation.w = quat.getW();
  pub_pose.publish(ps);
}

void RobotArm::RobotModeStateCallback(const ur_msgs::RobotModeDataMsg &msg){
  is_robot_enable = (!msg.is_emergency_stopped and !msg.is_protective_stopped);
  if(!is_robot_enable){
    ROS_WARN("%f | Robot is now disable.", force);
  }
}

void RobotArm::RobotWrenchCallback(const geometry_msgs::WrenchStamped &msg){
  bool should_shot; // Should not block in following `if` statement if already higher than force thres
  double f_x = msg.wrench.force.x,
         f_y = msg.wrench.force.y,
         f_z = msg.wrench.force.z;
  double past_force = force;
  force = sqrt(f_x*f_x+f_y*f_y+f_z*f_z);
  if(past_force<force_thres && force>=force_thres) should_shot = true;
  else should_shot = false;
  if(should_shot){
    if(is_send_goal){
      ROS_WARN("[%s] | %f | High flange force detected, stop program", ros::this_node::getName().c_str(), force);
      traj_client->cancelGoal();
      ur_control.stop_program();
      is_send_goal = false;
    }
  }
}

void RobotArm::TimerCallback(const ros::TimerEvent &event){
  double tmp;
  pnh_.getParam("force_thres", tmp);
  if(tmp!=force_thres){
    ROS_INFO("[%s] force_thres set from %f to %f", ros::this_node::getName().c_str(), force_thres, tmp);
    force_thres = tmp;
  }
}

void RobotArm::PoseToDH(geometry_msgs::Pose pose, double *T){
  double roll = 0, pitch = 0, yaw = 0;
  geometry_msgs::Point &p = pose.position;
  tf::Quaternion q;
  tf::quaternionMsgToTF(pose.orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  double sinr = sin(roll), cosr = cos(roll);
  double sinp = sin(pitch), cosp = cos(pitch);
  double siny = sin(yaw), cosy = cos(yaw);
  // DH matrix, ZYX convention, see: https://en.wikipedia.org/wiki/Euler_angles#Rotation_matrix (Tait-Bryan, Z1Y2X3)
  T[0] = cosy*cosp;
  T[1] = cosy*sinp*sinr - cosr*siny;
  T[2] = siny*sinr + cosy*cosr*sinp;
  T[3] = p.x;
  T[4] = cosp*siny;
  T[5] = cosy*cosr + siny*sinp*sinr;
  T[6] = cosr*siny*sinp - cosy*sinr;
  T[7] = p.y;
  T[8] = -sinp;
  T[9] = cosp*sinr;
  T[10] = cosp*cosr;
  T[11] = p.z;
  T[15] = 1;
}

int RobotArm::PerformIK(geometry_msgs::Pose target_pose, double *sol){
  double T[16] = {0};
  PoseToDH(target_pose, T);
  // tcp_link to ee_link since what we interested is tcp_link
  for (int i = 0; i < 3; ++i)
    T[i*4+3] -= tool_length*T[i*4];
  double q_sols[8*6], min = 1e6, dist = 0;
  int sols = ur_kinematics::inverse(T, q_sols), index = -1;
  int valid_sols = sols;
  for (int i = 0; i < sols; ++i) {
    // Check if NAN solution
    if(std::isnan(q_sols[i*6])){
      valid_sols-=1;
      continue;
    }
    // Preprocess joint angle to -pi ~ pi
    for (int j = 0; j < 6; ++j) q_sols[i*6 + j] = validAngle(q_sols[i*6 + j]); 
    //for (int j = 0; j < 6; ++j) printf("%f ", q_sols[i*6+j]);
    //printf("\n");
    q_sols[i*6] = makeMinorRotate(joint[0], q_sols[i*6]);
    // Convert wrist joints to available range, or set wristX_collision to true if collision happen
    wrist1_collision = wrist_check_bound(q_sols[i*6+3], wrist1_upper_bound, wrist1_lower_bound); 
    wrist2_collision = wrist_check_bound(q_sols[i*6+4], wrist2_upper_bound, wrist2_lower_bound); 
    wrist3_collision = wrist_check_bound(q_sols[i*6+5], wrist3_upper_bound, wrist3_lower_bound);
    // Check if corresponding angle for wrist
    for(int j=3; j<6; ++j) q_sols[i*6+j] = makeMinorRotate(joint[j], q_sols[i*6+j]);
    for (int j = 0; j < 6; ++j) {
      if(j<3) dist += pow((q_sols[i*6 + j] - joint[j])*1.5, 2); // For joint 1, 2, and 3, multiply by 1.5
      else dist += pow((q_sols[i*6 + j] - joint[j])*0.5, 2); // For joint 4, 5 and 6, multiply by 0.5
    }
    // Find solution with minimum joint angle difference
    if(min>dist && !wrist1_collision && !wrist2_collision && !wrist3_collision){
      min = dist;
      index = i;
    } dist = 0; 
  } 
  if(index == -1 and valid_sols!=0) {
    printf("PerformIK wrist collsion\n"); 
    return 0; // All valid solutions will self-collision
  }
  for(int i=0; i<6; ++i) sol[i] = q_sols[index*6+i];
  return (num_sols = valid_sols);
}

int RobotArm::PerformIKWristMotion(geometry_msgs::Pose target_pose, double *sol){
  double T[16] = {0};
  PoseToDH(target_pose, T);
  // tcp_link to ee_link since what we interested is tcp_link
  for(int i=0; i<3; ++i)
    T[i*4+3] -= tool_length*T[i*4];
  double q_sols[8*6], min = 1e6, dist = 0;
  int sols = ur_kinematics::inverse(T, q_sols), index = -1;
  for(int i=0; i<sols; ++i){
    // Preprocess joint anglr to -pi ~ pi
    for(int j=0; j<6; ++j){
      q_sols[i*6+j] = validAngle(q_sols[i*6+j]);
    }
    // Consider only the same sign of joint 1
    if(joint[0]*q_sols[i*6]<0.0f) continue;
    // See if Co-located angle for wrist
    for(int j=3; j<6; ++j) {
      q_sols[i*6+j] = makeMinorRotate(joint[j], q_sols[i*6+j]);
      dist += pow(q_sols[i*6+j] - joint[j], 2);
    }if(min>dist){
      min = dist; index = i; 
    } dist = 0;
  } if(index == -1) return 0;
  for(int i=0; i<6; ++i) sol[i] = q_sols[index*6+i];
  return (num_sols = sols);
}

inline bool RobotArm::wrist_check_bound(double &joint, double upper, double lower){
  if(joint>upper) joint-=2*M_PI;
  else if(joint<lower) joint+=2*M_PI;
  if(joint>lower and joint<upper) return false;
  return true;
}

geometry_msgs::Pose RobotArm::getCurrentTCPPose(void){
  geometry_msgs::Pose pose_now;
  ros::spinOnce(); // Update joint state
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
  pose_now.orientation.x = q.getX();
  pose_now.orientation.y = q.getY();
  pose_now.orientation.z = q.getZ();
  pose_now.orientation.w = q.getW();
  return pose_now;
}

// TODO: Combine with Jacobian to compute reasonable time?
double RobotArm::calculate_time(const double *now, const double *togo, double factor){
  double time;
  // The less the factor, the faster the move
  if(factor == 0.0) {ROS_WARN("Invalid factor, set to 0.5"); factor = 0.5;}
  double dist = 0;
  for(int i=0; i<6; ++i){
    dist += pow(now[i] - togo[i], 2);
  }
  if(sqrt(dist)/factor<=0.5) time = 0.5;
  else time = ceil(sqrt(dist)/factor);
  return (time);
}

inline actionlib::SimpleClientGoalState RobotArm::getState() {
  return traj_client->getState();
}

inline void RobotArm::StartTrajectory(control_msgs::FollowJointTrajectoryGoal goal){
  goal.trajectory.header.stamp = ros::Time::now();
  is_send_goal = true;
  traj_client->sendGoal(goal);
  // Wait for ur to finish joint trajectory
  while(!traj_client->getState().isDone() && ros::ok()) {usleep(100000); ros::spinOnce();}
  is_send_goal = false;
}

control_msgs::FollowJointTrajectoryGoal RobotArm::ArmToDesiredPoseTrajectory(geometry_msgs::Pose pose, double factor){
  // Check if given quaternion is invalid
  trajectory_msgs::JointTrajectory &t = goal.trajectory;
  if(t.joint_names.size()==0){
    t.joint_names.resize(6);
    for(int i=0; i<6; ++i)
      t.joint_names[i] = joint_names[conversion[i]];
  }
  // Get closest joint space solution
  double sol[6] = {0};
  if(!PerformIK(pose, sol)) {
    ROS_WARN("[%s] Cannot find IK solution!", ros::this_node::getName().c_str());
  }else{
    for (int i = 0; i < 6; ++i) {
      t.points[0].positions[i] = joint[conversion[i]];
      t.points[1].positions[i] = sol[conversion[i]]; 
      t.points[0].velocities[i] = 
      t.points[1].velocities[i] = 0.0;
    }
    t.points[0].time_from_start = ros::Duration(0.01);
    double exeute_time = calculate_time(joint, sol, factor);
    t.points[1].time_from_start = ros::Duration(exeute_time);
    ROS_INFO("[%s] Execution time: %f seconds", ros::this_node::getName().c_str(), exeute_time);
    /*
    for(int i=0; i<6; ++i)
      printf("%s Start: %f Target: %f\n", t.joint_names[i].c_str(), t.points[0].positions[i], t.points[1].positions[i]);
    */
  }
  
  return goal;
}
