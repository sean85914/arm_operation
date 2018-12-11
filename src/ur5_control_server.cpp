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
#include <robotiq_c_model_control/CModel_robot_output.h>

#include <cmath>
#include <Eigen/Dense>

#include <ur_kin.h>


#define deg2rad(x) (x*M_PI/180.0)

// Pick standby pose, in joint space
const double stby_pick[6]  = {-1.76529, -1.34114, 1.11831, \
		              -1.54085, -1.51315, -1.72227};
// Place standby pose, in joint space
const double stby_place[6] = {-0.16464, -2.13780, 1.82646, \
		              -2.92213, -1.49629, -1.56415};

bool in_range(double input, double lower, double upper)
{
  if(input > lower and input < upper) return true;
  else return false;
}

typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;

class RobotArm {
 private:
  TrajClient *traj_client_;
  control_msgs::FollowJointTrajectoryGoal goal_, straight_;
  ros::NodeHandle nh_;
  // gripper cmd publisher
  ros::Publisher pub_gripper_cmd;
  ros::Subscriber sub_joint_state_;
  ros::ServiceServer goto_pose_srv_;
  ros::ServiceServer goto_stby_pick_srv_;
  ros::ServiceServer goto_stby_place_srv_;
  ros::ServiceServer go_straight_srv_;
  ros::ServiceServer goto_joint_pose_srv_;
  ros::ServiceServer loose_gripper_srv_;
  // gripper cmd
  robotiq_c_model_control::CModel_robot_output cmd;  
  int num_sols_, last_gripper_dist_;
  const int numberOfPoints = 10.0;
  double joint_[6], tool_length_, finger_dist;
  bool wrist1_collision_ = false, wrist2_collision_ = false, wrist3_collision_ = false;
  /*
     Change input joint to principle branch [-pi, pi]
     Input: 
       double angle: input joint angle from IK
     Output:
       double angle: joint angle converted to principle branch 
  */

  double validAngle(double angle) {
    if (abs(angle) > 2*M_PI) angle = fmod(angle, 2*M_PI);
    if (angle > M_PI) angle -= 2*M_PI;
    else if (angle < -M_PI) angle += 2*M_PI;
    return angle;
  }
  void JointStateCallback(const sensor_msgs::JointState &msg) {
    joint_[0] = msg.position[0];
    joint_[1] = msg.position[1];
    joint_[2] = msg.position[2];
    joint_[3] = msg.position[3];
    joint_[4] = msg.position[4];
    joint_[5] = msg.position[5];
  }
  /*
      Convert pose to transformation matrix
      Input:
        geometry_msgs::Pose pose: target pose, including position and orientation
        double *T: output placeholder of size-16 double array
      Output:
        None
  */
  void PoseToDH(geometry_msgs::Pose pose, double *T) {
    double roll = 0, pitch = 0, yaw = 0;
    geometry_msgs::Point &p = pose.position;
    tf::Quaternion q;
    tf::quaternionMsgToTF(pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    double sinr = sin(roll), cosr = cos(roll);
    double sinp = sin(pitch), cosp = cos(pitch);
    double siny = sin(yaw), cosy = cos(yaw);
    
    // DH matrix, ZYX convention
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
  /*
      Perform IK and return the optimized joints
      Input: 
        geometry_msgs::Pose target_pose: target pose
        double *sol: output placeholder of size-6 double array
      Output:
        int: number of IK solutions
  */
  int PerformIK(geometry_msgs::Pose target_pose, double *sol) {
    double T[16] = {0};
    PoseToDH(target_pose, T);
    // tcp_link to ee_link since what we interested is tcp_link
    for (int i = 0; i < 3; ++i)
      T[i*4+3] -= tool_length_*T[i*4];

    double q_sols[8*6], min = 1e6, dist = 0;
    int sols = ur_kinematics::inverse(T, q_sols), index = 0;
    for (int i = 0; i < sols; ++i) {
      // Preprocess joint angle to -pi ~ pi.      
      for (int j = 0; j < 6; ++j) {
        q_sols[i*6 + j] = validAngle(q_sols[i*6 + j]); 
      }
      // Convert wrist joints to available range
      wrist1_check_bound(q_sols[i*6+3]); wrist2_check_bound(q_sols[i*6+4]); wrist3_check_bound(q_sols[i*6+5]);
      for (int j = 0; j < 6; ++j) {dist += pow(q_sols[i*6 + j] - joint_[j], 2);}
      // Find solution with minimun joint angle difference
      if (min > dist) {
        min = dist;
        index = i; 
      } dist = 0;
    }
    for (int i = 0; i < 6; ++i) 
      sol[i] = q_sols[index*6 + i];
    std::cout << "Joint solved from IK: ";
    for (int i = 0; i <6; ++i)
      std::cout << sol[i] << " ";
    std::cout << std::endl;
    return (num_sols_ = sols);
  }
  // Initial gripper, see package://robotiq_c_model_control/nodes/CModelSimpleController.py
  void initial_gripper(void)
  {
    // rPR = 0: open
    // rPR = 255: closed
    // Reset
    pub_gripper_cmd.publish(cmd);
    ros::Duration(0.5).sleep();
    cmd.rACT = 1; // active 
    cmd.rGTO = 1; // go to
    cmd.rATR = 0; // automatic release
    cmd.rPR  = 0; // position request
    cmd.rSP  = 100; // speed
    cmd.rFR  = 30; // force
    pub_gripper_cmd.publish(cmd);
    // sleep 0.5s to make sure gripper receive the command
    ros::Duration(0.5).sleep();
    last_gripper_dist_ = 0;
  }
  /* 
     Wrist 1 constraint
     Input:
       double& joint: wrist 1 joint value
     Output:
       bool: true if wrist 1 will not collision itself, false otherwise
  */
  // joint angle limits: [-240, -30] (deg)
  bool wrist1_check_bound(double& joint){
    if(joint > 0) joint -= 2*M_PI;
    if(!in_range(joint, deg2rad(-240), deg2rad(-30))) {wrist1_collision_ = true; return false;}
    wrist1_collision_ = false;
    return true;
  }
  /* 
     Wrist 2 constraint
     Input:
       double& joint: wrist 2 joint value
     Output:
       bool: true if wrist 2 will not collision itself, false otherwise
  */
  // joint angle limits: [-pi, 0] (rad)
  bool wrist2_check_bound(double& joint){
    if(joint > 0) joint -= 2*M_PI;
    if(!in_range(joint, -M_PI, 0)) {wrist2_collision_ = true; return false;}
    wrist2_collision_ = false;
    return true;
  }
  /* 
     Wrist 3 constraint
     Input:
       double& joint: wrist 3 joint value
     Output:
       bool: true if wrist 3 will not collision itself, false otherwise
  */
  // joint angle limits: [-220, 5]         (deg)
  bool wrist3_check_bound(double& joint){
    if(joint > deg2rad(5)) joint -= 2*M_PI;
    else if(joint < deg2rad(-220)) joint += 2*M_PI;
    if(!in_range(joint, deg2rad(-220), deg2rad(5))) {wrist3_collision_ = true; return false;}
    wrist3_collision_ = false; 
    return true;
  }

 public:
  // Constructor
  RobotArm() : joint_(), tool_length_(0.18), finger_dist(0),num_sols_(1) {
    // tell the action client that we want to spin a thread by default
    traj_client_ = new TrajClient("/follow_joint_trajectory", true);
    // wait for action server to come up
    while (!traj_client_->waitForServer(ros::Duration(5.0))) 
      ROS_INFO("Waiting for the joint_trajectory_action server");
    // subscriber, publisher and service server
    sub_joint_state_ = nh_.subscribe("/joint_states", 1, &RobotArm::JointStateCallback, this);
    pub_gripper_cmd = nh_.advertise<robotiq_c_model_control::CModel_robot_output>("/CModelRobotOutput", 1);
    goto_pose_srv_ = nh_.advertiseService("/ur5_control/goto_pose", &RobotArm::GotoPoseService, this);
    goto_stby_pick_srv_ = nh_.advertiseService("/ur5_control/goto_stby_pick", &RobotArm::GotoSTBYPickService, this);
    goto_stby_place_srv_ = nh_.advertiseService("/ur5_control/goto_stby_place", &RobotArm::GotoSTBYPlaceService, this);
    go_straight_srv_ = nh_.advertiseService("/ur5_control/go_straight", &RobotArm::GoStraightLineService, this);
    goto_joint_pose_srv_ = nh_.advertiseService("/ur5_control/goto_joint_pose", &RobotArm::GotoJointPoseService, this);
    loose_gripper_srv_ = nh_.advertiseService("/ur5_control/loose_gripper", &RobotArm::LooseGripperService, this);
    // initialize the gripper
    initial_gripper();
    trajectory_msgs::JointTrajectory &t = goal_.trajectory;
    trajectory_msgs::JointTrajectory &l = straight_.trajectory;
    t.joint_names.resize(6);                   l.joint_names.resize(6);
    t.joint_names[0] = "shoulder_pan_joint";   l.joint_names[0] = "shoulder_pan_joint";
    t.joint_names[1] = "shoulder_lift_joint";  l.joint_names[1] = "shoulder_lift_joint";
    t.joint_names[2] = "elbow_joint";          l.joint_names[2] = "elbow_joint";
    t.joint_names[3] = "wrist_1_joint";        l.joint_names[3] = "wrist_1_joint";
    t.joint_names[4] = "wrist_2_joint";        l.joint_names[4] = "wrist_2_joint";
    t.joint_names[5] = "wrist_3_joint";        l.joint_names[5] = "wrist_3_joint";
    
    l.points.resize(numberOfPoints+1);
    for (int i = 0; i < numberOfPoints+1; ++i) {
      l.points[i].positions.resize(6);
      l.points[i].velocities.resize(6);
    }

    // Go to pose "stby_pick"    
    t.points.resize(2); 
    t.points[0].positions.resize(6);
    t.points[0].velocities.resize(6);
    t.points[1].positions.resize(6);
    t.points[1].velocities.resize(6);
    
    ros::spinOnce();
    for (int i = 0; i < 6; ++i) {
      t.points[0].positions[i] = joint_[i];
      t.points[1].positions[i] = stby_pick[i];
      t.points[0].velocities[i] = 
      t.points[1].velocities[i] = 0;
    }
    
    t.points[0].time_from_start = ros::Duration(0);
    t.points[1].time_from_start = ros::Duration(calculate_time(joint_, stby_pick));
    StartTrajectory(goal_);
  }
  // Destructor
  ~RobotArm() {
    delete traj_client_;
  }
  bool LooseGripperService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    cmd.rPR = 0;
    pub_gripper_cmd.publish(cmd);
    return true;
  }
  bool GotoPoseService(arm_operation::target_pose::Request &req, arm_operation::target_pose::Response &res) {
    ROS_INFO("Receive new pose goal: %f %f %f %f %f %f %f", req.target_pose.position.x,
                                                            req.target_pose.position.y,
                                                            req.target_pose.position.z,
                                                            req.target_pose.orientation.x,
                                                            req.target_pose.orientation.y, 
                                                            req.target_pose.orientation.z, 
                                                            req.target_pose.orientation.w);
    tf::Quaternion q_req(req.target_pose.orientation.x,
                         req.target_pose.orientation.y, 
                         req.target_pose.orientation.z, 
                         req.target_pose.orientation.w);
    std::cout << "Joint now: ";
    for (int i=0; i<6; ++i){std::cout << joint_[i] << " ";} std::cout << std::endl;
    StartTrajectory(ArmToDesiredPoseTrajectory(req.target_pose, req.factor));
    if(last_gripper_dist_ != req.finger_dist) ros::Duration(1.0).sleep();
    cmd.rPR = req.finger_dist / 100.0 * 255;
    pub_gripper_cmd.publish(cmd);
    if(last_gripper_dist_ != req.finger_dist) ros::Duration(1.0).sleep();
    if(num_sols_ == 0) res.plan_result = "fail_to_find_solution";
    else {
      if(!wrist1_collision_ and !wrist2_collision_ and !wrist3_collision_) res.plan_result = "find_one_feasible_solution";
      else res.plan_result = "wrist_collision_itself";
    }
    last_gripper_dist_ = req.finger_dist;
    return true;
  }

  bool GotoSTBYPickService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    trajectory_msgs::JointTrajectory &t = goal_.trajectory;
    for (int i = 0; i < 6; ++i) {
      t.points[0].positions[i] = joint_[i];
      t.points[1].positions[i] = stby_pick[i];
      t.points[0].velocities[i] = 
      t.points[1].velocities[i] = 0;
    }
    t.points[0].time_from_start = ros::Duration(0);
    t.points[1].time_from_start = ros::Duration(calculate_time(joint_, stby_pick));
    StartTrajectory(goal_);
  }

  bool GotoSTBYPlaceService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    trajectory_msgs::JointTrajectory &t = goal_.trajectory;
    for (int i = 0; i < 6; ++i) {
      t.points[0].positions[i] = joint_[i];
      t.points[1].positions[i] = stby_place[i];
      t.points[0].velocities[i] = 
      t.points[1].velocities[i] = 0;
    }
    t.points[0].time_from_start = ros::Duration(0);
    t.points[1].time_from_start = ros::Duration(calculate_time(joint_, stby_place));
    StartTrajectory(goal_);
  }

  bool GoStraightLineService(arm_operation::target_pose::Request  &req,
                             arm_operation::target_pose::Response &res) {
    ROS_INFO("Receive new straight line goal: %f %f %f %f %f %f %f", req.target_pose.position.x, 
                                                                     req.target_pose.position.y,
                                                                     req.target_pose.position.z,
                                                                     req.target_pose.orientation.x,
                                                                     req.target_pose.orientation.y,
                                                                     req.target_pose.orientation.z,
                                                                     req.target_pose.orientation.w);
    tf::Quaternion q_req(req.target_pose.orientation.x,
                         req.target_pose.orientation.y, 
                         req.target_pose.orientation.z, 
                         req.target_pose.orientation.w);
    /*if(q_req.getW() < 0 or abs(q_req.length()-1) > 1e-1) {
      ROS_ERROR("Non-normalized quaternion, ignore the request"); 
      res.plan_result = "fail_to_find_solution"; 
      return true;
    }*/
    std::cout << "Joint now: ";
    for (int i=0; i<6; ++i){std::cout << joint_[i] << " ";} std::cout << std::endl;
    trajectory_msgs::JointTrajectory &l = straight_.trajectory;
    geometry_msgs::Pose pose_now = getCurrentTCPPose();
    double waypoint_sol_[numberOfPoints * 6] = {0}, temp[6] = {0};
    tf::Quaternion q(pose_now.orientation.x, 
                     pose_now.orientation.y,
                     pose_now.orientation.z,
                     pose_now.orientation.w), 
                   q_inter_,
                   q_goal_(req.target_pose.orientation.x, 
                           req.target_pose.orientation.y,
                           req.target_pose.orientation.z,
                           req.target_pose.orientation.w);
    for (int i = 0; i < numberOfPoints ; ++i) {
      geometry_msgs::Pose waypoint;
      waypoint.position.x = pose_now.position.x + (req.target_pose.position.x - pose_now.position.x) * (i+1) / double(numberOfPoints);
      waypoint.position.y = pose_now.position.y + (req.target_pose.position.y - pose_now.position.y) * (i+1) / double(numberOfPoints);
      waypoint.position.z = pose_now.position.z + (req.target_pose.position.z - pose_now.position.z) * (i+1) / double(numberOfPoints);
      q_inter_ = q.slerp(q_goal_, (i+1) / double(numberOfPoints));
      waypoint.orientation.x = q_inter_.getX();
      waypoint.orientation.y = q_inter_.getY();
      waypoint.orientation.z = q_inter_.getZ();
      waypoint.orientation.w = q_inter_.getW();
      std::cout << "waypoint: " << i+1 <<" : " << waypoint.position.x << " " << waypoint.position.y <<
                   " " << waypoint.position.z << " " << waypoint.orientation.x << " " <<
                   waypoint.orientation.y << " " << waypoint.orientation.z << " " <<
                   waypoint.orientation.w << std::endl;
      PerformIK(waypoint, temp);
      if(num_sols_ == 0) {
        ROS_ERROR("waypoint index: %d fail to find IK solution", i);
        res.plan_result = "fail_to_find_solution";
        return true;
      }
      for (int j = 0; j < 6; ++j) {waypoint_sol_[i*6 + j] = temp[j];}
   }
   double total_time = calculate_time(joint_, temp, req.factor); // Total time cost from initial to goal
   // Least Square to solve joints angular velocities
   Eigen::MatrixXd A(numberOfPoints+1, 3);
   Eigen::MatrixXd x(3, 6);
   Eigen::MatrixXd b(numberOfPoints+1, 6);
   for (int i = 0; i < numberOfPoints; ++i) {
     double t = total_time * (i+1) / double(numberOfPoints);
     A(i+1, 0) = t*t; A(i+1, 1) = t; A(i+1, 2) = 1;
     for (int j=0; j<6; ++j) {
       b(i+1, j) = waypoint_sol_[i*6 + j];
     }
   }
   A(0, 0) = 0; A(0, 1) = 0; A(0, 2) = 1;
   for(int j = 0; j<6; ++j) {b(0, j)=joint_[j];}
   x = (A.transpose()*A).inverse()*A.transpose()*b;
   ROS_INFO_STREAM("matrix x: " << x);
   for (int i=0; i<numberOfPoints; ++i) {
     double temp[6]={0};
     for (int j=0; j<6; ++j){
       l.points[i+1].positions[j] = waypoint_sol_[i*6 + j]; temp[j] = waypoint_sol_[i*6 + j];
       l.points[i+1].velocities[j] = 2* x(0, j) * total_time * (i+1) / double(numberOfPoints) + x(1, j);
       l.points[i+1].time_from_start = ros::Duration(total_time * (i+1) / double(numberOfPoints));
     }
     ROS_INFO("Waypoint index: %d joint to go: %f %f %f %f %f %f", i, temp[0], temp[1], temp[2],
                                                                      temp[3], temp[4], temp[5]);
   }
   for (int i=0; i<6; ++i) {
     l.points[0].positions[i] = joint_[i];
     l.points[0].velocities[i] = 0;
     l.points[numberOfPoints].velocities[i] = 0; 
     l.points[0].time_from_start = ros::Duration(0);
   }
   StartTrajectory(straight_);
   if(last_gripper_dist_ != req.finger_dist) ros::Duration(1.0).sleep();
   cmd.rPR = req.finger_dist / 100.0 * 255;
   pub_gripper_cmd.publish(cmd);
   if(last_gripper_dist_ != req.finger_dist) ros::Duration(1.0).sleep();
   last_gripper_dist_ = req.finger_dist;
   res.plan_result = "find_one_feasible_solution";
   return true;
  }  
  bool GotoJointPoseService(arm_operation::joint_pose::Request  &req,
                            arm_operation::joint_pose::Response &res) {
    trajectory_msgs::JointTrajectory &t = goal_.trajectory;
    double joints_array[6] = {req.joint1, req.joint2, req.joint3,
                              req.joint4, req.joint5, req.joint6};
    for (int i = 0; i < 6; ++i) {
      t.points[0].positions[i] = joint_[i];
      t.points[1].positions[i] = joints_array[i];
      t.points[0].velocities[i] = 0;
      t.points[1].velocities[i] = 0;
    }
    t.points[0].time_from_start = ros::Duration(0);
    t.points[1].time_from_start = ros::Duration(calculate_time(joint_, joints_array));
    StartTrajectory(goal_);
    res.plan_result = "Success";
    return true;
  }
  void StartTrajectory(control_msgs::FollowJointTrajectoryGoal goal) {
    goal.trajectory.header.stamp = ros::Time::now();
    traj_client_->sendGoal(goal);
    // Wait for ur5 to finish joint trajectory
    while(!traj_client_->getState().isDone() && ros::ok()) usleep(100000);
  }
  control_msgs::FollowJointTrajectoryGoal ArmToDesiredPoseTrajectory(geometry_msgs::Pose pose, double factor) {
    trajectory_msgs::JointTrajectory &t = goal_.trajectory;
    // Get closest joint space solution
    double sol[6] = {0};
    if (!PerformIK(pose, sol)) {
      for (int i = 0; i < 6; ++i) 
        sol[i] = joint_[i];
    }
    // Update collision flags
    wrist1_check_bound(sol[3]);
    wrist2_check_bound(sol[4]);
    wrist3_check_bound(sol[5]);
    if(wrist1_collision_ or wrist2_collision_ or wrist3_collision_) {
      ROS_ERROR("Wrist will collide with itself, ignore the solution...");
      for (int i = 0; i < 6; ++i){
        t.points[0].positions[i] = t.points[1].positions[i] = joint_[i];
        t.points[0].velocities[i] = t.points[1].velocities[i] = 0;
      }
      return goal_;
    }
    ROS_INFO("Joints to go: %f %f %f %f %f %f", sol[0], sol[1], sol[2], sol[3], sol[4], sol[5]);
    
    for (int i = 0; i < 6; ++i) {
      t.points[0].positions[i] = joint_[i];
      t.points[1].positions[i] = sol[i];
      t.points[0].velocities[i] = 
      t.points[1].velocities[i] = 0;
    }
    t.points[0].time_from_start = ros::Duration(0);
    t.points[1].time_from_start = ros::Duration(calculate_time(joint_, sol, factor));
    return goal_;
  }
  actionlib::SimpleClientGoalState getState() {
    return traj_client_->getState();
  }
  /*
      Calculate execution time from start pose to target pose
      Input:
        double *now: now joint position array
        double *togo: target pose joint position array
        double factor: joint velocity factor, the larger the faster, default as 0.5
      Output:
        int: time to execute this trajectory
  */
  int calculate_time(const double *now, const double *togo, double factor = 0.5)
  {
    int time;
    if(factor == 0.0) {ROS_ERROR("Invalid factor, set to default..."); factor = 0.5;}
    double dist = 0;
    for(int i=0; i<6; ++i){
      dist += pow(now[i] - togo[i], 2);
    }
    return (time = ceil(sqrt(dist)/factor));
  }
  /*
      Get current TCP pose
      Input:
        None
      Output:
        geometry_msgs::Pose: current tcp pose
  */
  geometry_msgs::Pose getCurrentTCPPose(void) 
  {
    geometry_msgs::Pose pose_now;
    ros::spinOnce();
    double T[16] = {0};
    ur_kinematics::forward(joint_, T);
    tf::Quaternion q;
    tf::Matrix3x3 mat;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        mat[i][j] = T[i*4 + j];
      }
    }
    mat.getRotation(q);
    pose_now.position.x = T[3] + T[0] * tool_length_;
    pose_now.position.y = T[7] + T[4] * tool_length_;
    pose_now.position.z = T[11] + T[8] * tool_length_;
    pose_now.orientation.x = q.getX();
    pose_now.orientation.y = q.getY();
    pose_now.orientation.z = q.getZ();
    pose_now.orientation.w = q.getW();
    return pose_now;
  }
};

int main(int argc, char** argv)
{
  // Create an controller instance
  ros::init(argc, argv, "ur5_control");
  RobotArm arm;
  ros::spin();
  return 0;
}
