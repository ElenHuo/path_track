#ifndef DWA__H
#define DWA__H

#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include "path_track/DWA_Config.h" //包名、头文件名
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include "tf_listerner.h"
#include <tf/tf.h>

#define PI 3.1415926535f


// #define sample_time 0.1f  //  控制周期 sample time 画离散点
// #define max_vel 2.0f
// #define max_w   5.0f
// #define max_vel_acc 2.0f
// #define max_w_acc 2.0f
// #define dis_thre 0.2f

// #define a 0.05f
// #define b 0.2f
// #define c 0.1f

// #define R_inflation 0.5f

#define vel_res 0.01f  //resolution
#define w_res 1.0/180.0*PI

////权重系数 前向预测时间

#define predict_time 3.0f



 
using namespace std;

class GoalPoint
{
    public:
        double x;
        double y;
        GoalPoint()
        {
            x=0;
            y=0;
        }
};

class STATE //Store the robot state;
{
public:
    double x;
    double y;
    double theta;
    double v;
    double w;
    double heading;
    double Vel_Evaluation;
    double Obstacle_Penalty;
    double heading_sum ;
    double Vel_sum ;
    double Penatly_sum ;
    vector<double> STORE;
    double Score;
    STATE()
    {
        x = 0;
        y = 0;
        theta = 0;
        v = 0;
        w = 0;
        heading = 0;
        Vel_Evaluation = 0;
        Obstacle_Penalty = 0;
        Score = 100;
        heading_sum = 0;
        Vel_sum = 0;
        Penatly_sum = 0;
    }
private:

};

using namespace std;
class DWA
{
public:
    DWA();
    ~DWA();
    int load_path_success ;
    int running_flag ;
    int count;
    double sample_time;
    double max_vel;
    double max_w;
    double max_vel_acc;
    double max_w_acc;
    double a;
    double b;
    double c;
    double R_inflation;
    double dis_thre;
    
private:
    double Vr[4];
    double goal_x;
    double goal_y;

    vector<GoalPoint> goalpoint;
    vector<double> length;
    tf::Vector3 V1;
    tf::Vector3 V2;

    sensor_msgs::PointCloud Send_PTS;
    STATE Current_State;
    vector<STATE> ROBOT_STATE;
    vector<sensor_msgs::PointCloud> Traj;
    Tf_Listerner* car_in_map_;
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Publisher PC_pub_;
    ros::Publisher Line_pub_;
    ros::Subscriber sub_;
    nav_msgs::Path PATH;

    //void DWA_Algorithm(model);
    void CurrentSTATE();
    void Motion_model();
    void sub_pathCB(nav_msgs::Path GlobalPath);
    void Cal_Velocity_Space();
    double Cal_Heading(STATE A);
    void NormalizeEval();
    double toDegree(double THETA);
    void dynamicCb(path_track::DWA_Config &config,uint32_t level);
    double Cal_Vector_Heading(tf::Vector3 V1, tf::Vector3 V2);
    double distance_Heading(STATE A);
};



#endif