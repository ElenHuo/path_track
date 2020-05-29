#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/Path.h>
#include <vector>
#include "tf_listerner.h"

using namespace std;

ros::Subscriber sub_path_g;// = nh.subscribe("topic_name", 1000, subCallback);
nav_msgs::Path path_g;
int get_path_flag_g = 0;
boost::shared_ptr<Tf_Listerner> car_in_map_g;

void subPathCb(nav_msgs::Path msg)
{
    path_g = msg;
    get_path_flag_g = 1;
    /*新增输出全局路径 */
/*    for(int i = 0 ; i<path_g.poses.size(); i++)
    {
        cout << path_g.poses[i].pose.position.x << " " << path_g.poses[i].pose.position.y <<endl;
    }
    cout<<"\r\n"<<"\r\n"<<endl;*/

}

void getData()
{
    static double ttt = 0;
    double det_x1 = car_in_map_g->x() - path_g.poses[path_g.poses.size() - 1].pose.position.x;
    double det_y1 = car_in_map_g->y() - path_g.poses[path_g.poses.size() - 1].pose.position.y;
    if(det_x1 * det_x1 + det_y1 * det_y1 < 0.25)
    {
        get_path_flag_g = 0;
        ttt = 0;
        return;
    }

    double min_dis = 100000;
    int n = 0;
    for(int i = 0;i < path_g.poses.size();i++)
    {
        double det_x2 = car_in_map_g->x() - path_g.poses[i].pose.position.x;
        double det_y2 = car_in_map_g->y() - path_g.poses[i].pose.position.y;
        if(min_dis > det_x2 * det_x2 + det_y2 * det_y2)
        {
            min_dis = det_x2 * det_x2 + det_y2 * det_y2;
            n = i;
        }
    }
    double car_ang = 2 * acos(car_in_map_g->ow());
    car_ang = car_in_map_g->oz() * car_in_map_g->ow() < 0 ? (0 - car_ang) : car_ang;
    //double bias_y = (path_g.poses[n].pose.position.x - car_in_map_g->x()) * cos(car_ang) - (path_g.poses[n].pose.position.y - car_in_map_g->y()) * sin(car_ang);
    double det_x = path_g.poses[n].pose.position.x - car_in_map_g->x();
    double det_y = path_g.poses[n].pose.position.y - car_in_map_g->y();
    //double bias_d = sqrt(det_x * det_x + det_y * det_y);
    static double last_time = ros::Time::now().toSec();
    double delta_t = ros::Time::now().toSec() - last_time;
    delta_t = delta_t == 0 ? 0.05 : delta_t;
    ttt += delta_t;
    last_time = ros::Time::now().toSec();
    static double last_car_x = car_in_map_g->x();
    static double last_car_y = car_in_map_g->y();
    static double last_car_w = car_ang;
    double v_x = (car_in_map_g->x() - last_car_x) / delta_t;
    double v_y = (car_in_map_g->y() - last_car_y) / delta_t;
    double w__ = (car_ang - last_car_w) / delta_t;
    last_car_x = car_in_map_g->x();
    last_car_y = car_in_map_g->y();
    last_car_w = car_ang;
    cout << car_in_map_g->x() << " " << car_in_map_g->y() << " " << car_ang << " " 
         //<< bias_y << " " << bias_d << " "
         << path_g.poses[n].pose.position.x << " " << path_g.poses[n].pose.position.y << " "
         << v_x << " " << v_y << " " << w__ << " " 
         << ttt << endl;

    ros::Duration(0.1).sleep();
}

int main(int argc , char** argv)
{
    ros::init(argc , argv , "collect_data");
    ros::NodeHandle n;
    sub_path_g = n.subscribe("/own_path" , 2 , subPathCb);
    car_in_map_g = boost::make_shared<Tf_Listerner>("map" , "base_footprint");
    while(ros::ok())
    {
        ros::spinOnce();
        if(get_path_flag_g == 1)
        {
            getData();
        }
    }
    return 0;
}
