#ifndef TRAJECTORY_GENERATION_SERVER_H
#define TRAJECTORY_GENERATION_SERVER_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <atomic>
#include <fstream>
#include <vector>
#include <iostream>
#include <unistd.h>
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include "trajectory_generation/pchip.h"
#include "trajectory_generation/TrajectoryGeneration.h"


class trajectory_generation_server
{

public:
	trajectory_generation_server();
	~trajectory_generation_server();
	bool trajectory_generation_call(trajectory_generation::TrajectoryGeneration::Request  &req,
         trajectory_generation::TrajectoryGeneration::Response  &res);
	void generate_slerp(geometry_msgs::Quaternion q_in,geometry_msgs::Quaternion q_fin,double step, std::vector<geometry_msgs::Quaternion> &q_int);

private:



};

#endif
