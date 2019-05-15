#include "trajectory_generation/trajectory_generation_server.h"


 int main(int argc, char **argv)
 {
   ros::init(argc, argv, "Trajectory_Generation_Server");

   ROS_INFO("Waiting for waypoints.");
   trajectory_generation_server server;
   ros::NodeHandle nh;

   ros::ServiceServer service = nh.advertiseService("trajectory_generation_server", &trajectory_generation_server::trajectory_generation_call,&server);
   
   ros::spin();

   return 0;
}