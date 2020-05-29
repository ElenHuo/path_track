#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <iostream>
#include "tf_listerner.h"

#define PI 3.1415926535f

using namespace std;


Tf_Listerner* car_in_map_;
ros::Publisher pub_path_;
ros::Publisher pub_obstacle;

void Obstacle_production()
{
    double car_x = car_in_map_->x();
    double car_y = car_in_map_->y();
    nav_msgs::Path path; 
    path.header.frame_id = "map";
    path.poses.resize(21);
    path.poses[0].pose.position.x = car_x + 2;
    path.poses[0].pose.position.y = car_y ;
    double R = 0.1;
    double theta = 0 ;
    for(int i = 1; i < 21 ; i++)
    {
       path.poses[i].pose.position.x = path.poses[0].pose.position.x + R * cos(theta);
       path.poses[i].pose.position.y = path.poses[0].pose.position.y + R * sin(theta);
       theta += PI/10;
    }
    pub_obstacle.publish(path);

}

void pubPath()
{
    double car_x = car_in_map_->x();
    double car_y = car_in_map_->y();
    double car_oz = car_in_map_->oz();
    double car_ow = car_in_map_->ow();
    double centrol_x = car_x + 0.8;
    double centrol_y = car_y;
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.poses.resize(39);
    for(int i = 0;i < 19;i++)
    {
        path.poses[i].header.frame_id = "map";
        path.poses[i].pose.position.x = centrol_x - 0.8 * cos(i * 0.16);
        path.poses[i].pose.position.y = centrol_y - 0.8 * sin(i * 0.16);
    }
    centrol_x = car_x + 2.4;
    centrol_y = car_y;
    for(int i = 19;i < 39;i++)
    {
        path.poses[i].pose.position.x = centrol_x - 0.8 * cos((i - 19) * 0.16);
        path.poses[i].pose.position.y = centrol_y + 0.8 * sin((i - 19) * 0.16);
    }
    pub_path_.publish(path);
    cout << "product a path." << endl;
}

void pubLine()
{
    double car_x = car_in_map_->x();
    double car_y = car_in_map_->y();
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.poses.resize(25);
    for(int i = 0;i < path.poses.size();i++)
    {
        path.poses[i].pose.position.x = car_x + 0.15 * i;
        path.poses[i].pose.position.y = car_y;
    }
    pub_path_.publish(path);
    cout << "product a path." << endl;
}

void pubLine7()
{
    double car_x = car_in_map_->x();
    double car_y = car_in_map_->y();
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.poses.resize(30);
    for(int i = 0;i < path.poses.size() / 2;i++)
    {
        path.poses[i].pose.position.x = car_x + 0.15 * i;
        path.poses[i].pose.position.y = car_y;
    }
    for(int i = path.poses.size() / 2;i < path.poses.size();i++)
    {
        path.poses[i].pose.position.x = path.poses[path.poses.size() / 2 - 1].pose.position.x;
        path.poses[i].pose.position.y = car_y + 0.15 * (i - path.poses.size() / 2 + 1);
    }
    pub_path_.publish(path);
    cout << "product a path." << endl;
}

void pubLineZ()
{
    double car_x = car_in_map_->x();
    double car_y = car_in_map_->y();
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.poses.resize(45);
    for(int i = 0;i < 15;i++)
    {
        path.poses[i].pose.position.x = car_x + 0.15 * i;
        path.poses[i].pose.position.y = car_y;
    }
    for(int i = 15;i < 30;i++)
    {
        path.poses[i].pose.position.x = path.poses[14].pose.position.x - 0.707 * 0.15 * (i - 14);
        path.poses[i].pose.position.y = path.poses[14].pose.position.y + 0.707 * 0.15 * (i - 14);
    }
    for(int i = 30;i < 45;i++)
    {
        path.poses[i].pose.position.x = path.poses[29].pose.position.x + 0.15 * (i - 29);
        path.poses[i].pose.position.y = path.poses[29].pose.position.y;
    }
    pub_path_.publish(path);
    cout << "product a path." << endl;
}
void pubOnepoint()
{
    double car_x = car_in_map_->x();
    double car_y = car_in_map_->y();   
    nav_msgs::Path path; 
    path.header.frame_id = "map";
    path.poses.resize(1);
    path.poses[0].pose.position.x = car_x + 1;
    path.poses[0].pose.position.y = car_y + 4;
    pub_path_.publish(path);
    cout << "product a path." << endl;    
}

void pubW()
{
    double car_x = car_in_map_->x();
    double car_y = car_in_map_->y();   
    nav_msgs::Path path; 
    path.header.frame_id = "map";
    path.poses.resize(40);
    for(int i = 0;i < 10;i++)
    {
        path.poses[i].pose.position.x = car_x + 0.707 * 0.15 * i;
        path.poses[i].pose.position.y = car_y - 0.707 * 0.15 * i;
    }
    for(int i = 10;i < 20;i++)
    {
        path.poses[i].pose.position.x = path.poses[9].pose.position.x + 0.707 * 0.15 * (i - 9);
        path.poses[i].pose.position.y = path.poses[9].pose.position.y + 0.707 * 0.15 * (i - 9);
    }
    for(int i = 20;i < 30;i++)
    {
        path.poses[i].pose.position.x = path.poses[19].pose.position.x + 0.707 * 0.15 * (i - 19);
        path.poses[i].pose.position.y = path.poses[19].pose.position.y - 0.707 * 0.15 * (i - 19);
    }
    for(int i = 30;i < 40;i++)
    {
        path.poses[i].pose.position.x = path.poses[29].pose.position.x + 0.707 * 0.15 * (i - 29);
        path.poses[i].pose.position.y = path.poses[29].pose.position.y + 0.707 * 0.15 * (i - 29);
    }
    pub_path_.publish(path);
    cout << "product a path." << endl;
    
}



int main(int argc,char** argv)
{
    ros::init(argc, argv, "product_path");
    ros::NodeHandle nh;
    pub_path_ = nh.advertise<nav_msgs::Path>("/own_path", 1);
    pub_obstacle = nh.advertise<nav_msgs::Path>("/obstacle", 1);
    car_in_map_ = new Tf_Listerner("map","base_footprint");
    char path_shape;
    while(ros::ok())
    {
        //Obstacle_production();
        cout << "please input 'l' 's' to product a path." << endl;
        cin >> path_shape;
        if(path_shape == 'l')   
        {
            pubLine();
            //Obstacle_production();
        }
        else if(path_shape == 's') pubPath();
        else if(path_shape == '7') pubLine7();
        else if(path_shape == 'z') pubLineZ();
        else if(path_shape == '1') pubOnepoint();
        else if(path_shape == 'w') pubW();
        else cout << "please a regualte char." << endl;
    }
    return 0;
}
