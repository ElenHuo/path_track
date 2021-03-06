#ifndef DWA_PATH_TRACK_H
#define DWA_PATH_TRACK_H

#pragma once
#include <vector>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/String.h>
#include "tf_listerner.h"
#include <pthread.h>
#include <actionlib/client/simple_action_client.h>
#include "path_track/path_track_Config.h"
#include <dynamic_reconfigure/server.h>

using namespace std;

Tf_Listerner* car_in_map_g;

class One_Particle
{
public:
    One_Particle();
	~One_Particle();
	One_Particle(nav_msgs::Path path , nav_msgs::OccupancyGrid map);
    double move_radius_[38] = {-23.3,-14.4,-8.9,-5.5,-3.4,-2.1,-1.3,-0.8,-0.5,-0.3,-0.2,-0.1,-0.087,-0.054,-0.033,-0.021,-0.011,0.01,-0.001,
	                           0.001,0.01,0.011,0.021,0.033,0.054,0.087,0.1,0.2,0.3,0.5,0.8,1.3,2.1,3.4,5.5,8.9,14.4,23.3};
	// double move_radius_[38] = {0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,
	//                            0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,};
	double evaluate_value_;
	vector<int> speed_encode_;
	vector<geometry_msgs::Point32> trajectory_point_;
	int predict_step_;
	geometry_msgs::Pose car_in_map_;

	void productParticle(nav_msgs::Path path , nav_msgs::OccupancyGrid map);
	void neighbourSearch(One_Particle* particle , nav_msgs::Path path , nav_msgs::OccupancyGrid map);
	void generateTrajectory();
	void setEvaluateValue(nav_msgs::Path path,nav_msgs::OccupancyGrid map);
	void displayTrajectory();
	bool trajectoryVelidCheck();
};



using namespace std;

class Dwa_Path_Track
{
public:
	Dwa_Path_Track();
	~Dwa_Path_Track();

private:
    ros::NodeHandle n_;
	ros::Publisher pub_velocity_;
	ros::Subscriber sub_dwa_map_;
	ros::Subscriber sub_path_;
	nav_msgs::Path path_;
	int get_path_flag_;
	nav_msgs::OccupancyGrid dwa_map_;
	int get_map_flag_;
	vector<One_Particle*> particle_swarm_;
	int particle_num_;
	
	void dynamicCb(path_track::path_track_Config &config,uint32_t level);
	void subDwaMap(nav_msgs::OccupancyGrid map);
	void subPath(nav_msgs::Path path);
	void pubVelocity(One_Particle* particle);

	void pthreadTwo();
    static void *threadTwo(void * arg);
    void threadRunTwo();
    pthread_t m_tid_;
};

ros::Publisher pub_path_rviz_;
void displayTrajectory(vector<geometry_msgs::Point32> trajectory_point , double evaluate_value_);

bool lessThan(One_Particle* one , One_Particle* two)
{
	return one->evaluate_value_ < two->evaluate_value_;
};

#endif
