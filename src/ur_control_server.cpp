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

// Public functions
RobotArm::RobotArm(ros::NodeHandle nh, ros::NodeHandle pnh): 
  nh_(nh), pnh_(pnh), num_sols(1), dt(0.1), is_send_goal(false), initialized(false), reference_position(0.0, 0.0, 0.0){
  // Publisher 
  pub_pose = pnh_.advertise<geometry_msgs::PoseStamped>("pose", 1);
  pub_det = pnh_.advertise<std_msgs::Float32>("det", 100);
  // Subscriber
  sub_joint_state = pnh_.subscribe("joint_states", 100, &RobotArm::JointStateCallback, this);
  // Service server
  goto_pose_srv = pnh_.advertiseService("ur_control/goto_pose", &RobotArm::GotoPoseService, this);
  go_straight_srv = pnh_.advertiseService("ur_control/go_straight", &RobotArm::GoStraightLineService, this);
  goto_joint_pose_srv = pnh_.advertiseService("ur_control/goto_joint_pose", &RobotArm::GotoJointPoseService, this);
  vel_ctrl_srv = pnh_.advertiseService("ur_control/velocity_control", &RobotArm::VelocityControlService, this);
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
  for(int i=0; i<groups.size(); ++i){ // manipulator/endeffector
    if(kinematic_model->getJointModelGroup(groups[i])->getVariableNames().size()!=0)
      group_idx = i;
  }
  joint_model_group = kinematic_model->getJointModelGroup(groups[group_idx]);
  joint_names = joint_model_group->getVariableNames();
  ROS_INFO("[%s] Joint names inside [%s]", ros::this_node::getName().c_str(), groups[group_idx].c_str());
  for(int i=0; i<joint_names.size(); ++i)
    std::cout << "\t" << joint_names[i] << "\n";
  // Show parameter information
  ROS_INFO("*********************************************************************************");
  ROS_INFO("[%s] Tool length: %f", ros::this_node::getName().c_str(), tool_length);
  ROS_INFO("[%s] Action server name: %s", ros::this_node::getName().c_str(), action_server_name.c_str());
  ROS_INFO("[%s] Wrist 1 bound: [%f, %f]", ros::this_node::getName().c_str(), wrist1_lower_bound, wrist1_upper_bound);
  ROS_INFO("[%s] Wrist 2 bound: [%f, %f]", ros::this_node::getName().c_str(), wrist2_lower_bound, wrist2_upper_bound);
  ROS_INFO("[%s] Wrist 3 bound: [%f, %f]", ros::this_node::getName().c_str(), wrist3_lower_bound, wrist3_upper_bound);
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
  ROS_WARN("[%s] Node shutdown", ros::this_node::getName().c_str());
}

bool RobotArm::GotoPoseService(arm_operation::target_pose::Request &req, arm_operation::target_pose::Response &res){
  if(!initialized){
    ROS_WARN("[%s] Robot not ready yet", ros::this_node::getName().c_str());
    res.plan_result = "robot_not_ready";
    return true;
  }
  if(!isValidQuat(req.target_pose.orientation)){
    ROS_WARN("[%s] Receive invalid quaternion, abort request...", ros::this_node::getName().c_str());
    res.plan_result = "Invalid target pose";
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
  ROS_INFO("[%s] Joint state now: %f %f %f %f %f %f", ros::this_node::getName().c_str(),
                                                      joint[0], joint[1], joint[2], joint[3], joint[4], joint[5]);
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
  trajectory_msgs::JointTrajectory &l = path.trajectory;
  if(l.joint_names.size()==0){
    l.joint_names.resize(6);
    for(int i=0; i<6; ++i)
      l.joint_names[i] = joint_names[conversion[i]];
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
  control_msgs::FollowJointTrajectoryGoal tmp;
  trajectory_msgs::JointTrajectory &t = tmp.trajectory;
  t.joint_names.resize(6);
  for(int i=0; i<6; ++i)
    t.joint_names[i] = joint_names[conversion[i]];
  t.points.resize(req.joints.size()+1);
  for(int i=0; i<=req.joints.size(); ++i){
    //t.points[i].positions.resize(6);
    //t.points[i].velocities.resize(6);
    for(int j=0; j<6; ++j){
      t.points[i].velocities.push_back(0.0);
      if(i==0){
        t.points[i].positions.push_back(joint[conversion[j]]);
        t.points[i].time_from_start = ros::Duration(0.01);
      } else{
        t.points[i].positions.push_back(req.joints[i-1].joint_value[conversion[j]]);
        double tmp_arr[6];
        std::copy(req.joints[i-1].joint_value.begin(),
                  req.joints[i-1].joint_value.end(), tmp_arr);
        t.points[i].time_from_start = ros::Duration(calculate_time(joint, tmp_arr));
      }
    }
  }
  StartTrajectory(tmp);
  res.plan_result = "done";
  return true;
}

bool RobotArm::VelocityControlService(arm_operation::velocity_ctrl::Request &req, arm_operation::velocity_ctrl::Response &res){
  double vx = req.twist.linear.x, 
         vy = req.twist.linear.y, 
         vz = req.twist.linear.z, 
         wx = req.twist.angular.x, 
         wy = req.twist.angular.y, 
         wz = req.twist.angular.z, 
         t = req.duration;
  std::string frame = (req.frame?"ee_link":"base_link");
  ROS_INFO("[%s] Received velocity control request: (%f, %f, %f, %f, %f, %f) in %f seconds w.r.t. %s", ros::this_node::getName().c_str(), vx, vy, vz, wx, wy, wz, t, frame.c_str());
  if(t<dt){
    ROS_WARN("[%s] Received duration less than control period (%f), ignore...", ros::this_node::getName().c_str(), dt);
    res.plan_result = "request duration less than control period";
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
  traj.points[0].time_from_start = ros::Duration(0.0);
  Eigen::VectorXd twist(6);
  // Convert back to base_link coordinate if frame == 0 
  if(!req.frame){
    tf::Quaternion quat(curr_tcp_pose.orientation.x, 
                        curr_tcp_pose.orientation.y, 
                        curr_tcp_pose.orientation.z, 
                        curr_tcp_pose.orientation.w);
    tf::Matrix3x3 mat(quat);
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
    kinematic_state_->setFromDiffIK(joint_model_group_, twist, joint_model_group->getLinkModelNames().back(), dt);
    std::vector<double> _joint;
    kinematic_state_->copyJointGroupPositions(joint_model_group_, _joint);
    for(int j=0; j<6; ++j){
      traj.points[i+1].positions.push_back(_joint[conversion[j]]);
      const double *joint_vel = kinematic_state->getVariableVelocities();
      traj.points[i+1].velocities.push_back(joint_vel[conversion[j]]);
    }
    traj.points[i+1].time_from_start = ros::Duration((i+1)*dt);
  }
  // TODO: what if robot arm out-of-workspace?
  // FIXME: sometimes the robot will vibrate back?
  StartTrajectory(goal);
  res.plan_result = "done";
  return true;
}

// Private functions
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
  for(int i=0; i<6; ++i)
    joint[i] = msg.position[conversion[i]];
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
  std::cout << "\n";
}

int RobotArm::PerformIK(geometry_msgs::Pose target_pose, double *sol){
  double T[16] = {0};
  PoseToDH(target_pose, T);
  // tcp_link to ee_link since what we interested is tcp_link
  for (int i = 0; i < 3; ++i)
    T[i*4+3] -= tool_length*T[i*4];
  double q_sols[8*6], min = 1e6, dist = 0;
  int sols = ur_kinematics::inverse(T, q_sols), index = -1, valid_sols = sols;
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

// TODO
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
  control_msgs::FollowJointTrajectoryGoal goal;
  trajectory_msgs::JointTrajectory &t = goal.trajectory;
  for(int i=0; i<6; ++i)
    t.joint_names.push_back(joint_names[conversion[i]]);
  t.points.resize(2);
  // Get closest joint space solution
  double sol[6] = {0};
  if(!PerformIK(pose, sol)) {
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
    ROS_INFO("[%s] Execution time: %f seconds", ros::this_node::getName().c_str(), exeute_time);
  }
  
  return goal;
}
