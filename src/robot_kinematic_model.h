#ifndef ROBOT_KINEMATIC_MODEL_H
#define ROBOT_KINEMATIC_MODEL_H

#include <vector>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/String.h>
#include "tf_listerner.h"
#include <tf/transform_broadcaster.h>
#include <pthread.h>
#include <actionlib/client/simple_action_client.h>
#include "path_track/path_track_Config.h"
#include <dynamic_reconfigure/server.h>
#include <ctime>
#include <stdio.h>
#include <dynamic_reconfigure/server.h>

using namespace std;
//using namespace boost;

#define max_lin_acc 2.5
#define max_ang_acc 3

boost::shared_ptr<Tf_Listerner> car_in_map_g;

class Robot_Model
{
public:
    Robot_Model();
    ~Robot_Model();

private:
    ros::NodeHandle n_;
	ros::Subscriber sub_vel_;// = nh.subscribe("topic_name", 1000, subCallback);
	ros::Publisher pub_robot_to_map_;// = nh.advertise<std_msgs::String>("topic_name", 1000);
	double linear_vel_;
	double angular_vel_;
	tf::TransformBroadcaster robot_to_map_;
	double real_lin_vel_;
	double real_ang_vel_;

	void subCmdVel(geometry_msgs::Twist msg);
	void changeRobotPosition();
};

#endif
