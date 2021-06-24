#include "ur_control_server.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ur_control_server_node");
  ros::NodeHandle nh, pnh("~");
  ros::AsyncSpinner spinner(4); // Use 4 threads
  RobotArm ur(nh, pnh);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
