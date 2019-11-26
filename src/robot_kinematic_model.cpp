#include "robot_kinematic_model.h"

Robot_Model::Robot_Model()
:linear_vel_(0),angular_vel_(0),real_lin_vel_(0),real_ang_vel_(0)
{
    robot_to_map_.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(0,0,0)),
                                ros::Time::now(),"map","base_footprint"));
    car_in_map_g = boost::make_shared<Tf_Listerner>("map" , "base_footprint");
    sub_vel_ = n_.subscribe("/cmd_vel", 5, &Robot_Model::subCmdVel,this);

    while(ros::ok())
    {
        changeRobotPosition();
        ros::spinOnce();
    }
}

Robot_Model::~Robot_Model()
{}

void Robot_Model::subCmdVel(geometry_msgs::Twist msg)
{
    linear_vel_ = msg.linear.x;
    angular_vel_ = msg.angular.z;
}

void Robot_Model::changeRobotPosition()
{
    static double last_time = ros::Time::now().toSec();
    double det_t = ros::Time::now().toSec() - last_time;
    last_time = ros::Time::now().toSec();
    double x,y,oz,ow;
    double circle_x,circle_y;
    double robot_x,robot_y,robot_theta;
    robot_theta = acos(2 * car_in_map_g->ow() * car_in_map_g->ow() - 1);
    robot_theta = car_in_map_g->oz() * car_in_map_g->ow() > 0 ? robot_theta:(0 - robot_theta);
    robot_x = car_in_map_g->x();
    robot_y = car_in_map_g->y();
    angular_vel_ = angular_vel_ == 0 ? 0.001 : angular_vel_;

    
    real_lin_vel_ = fabs(linear_vel_ - real_lin_vel_) / det_t > max_lin_acc ? real_lin_vel_ + (linear_vel_ - real_lin_vel_) / fabs(linear_vel_ - real_lin_vel_) * max_lin_acc * det_t : linear_vel_;
    real_ang_vel_ = fabs(angular_vel_ - real_ang_vel_) / det_t > max_ang_acc ? real_ang_vel_ + (angular_vel_ - real_ang_vel_) / fabs(angular_vel_ - real_ang_vel_) * max_ang_acc * det_t : angular_vel_;
    
    //cout << "(" << linear_vel_ << " , " << angular_vel_ << ")";
    //cout << " ,(" << real_lin_vel_ << " , " << real_ang_vel_ << ")" << endl;
    
    circle_x = robot_x - real_lin_vel_ / real_ang_vel_ * sin(robot_theta);
    circle_y = robot_y + real_lin_vel_ / real_ang_vel_ * cos(robot_theta);
    x = circle_x + real_lin_vel_ / real_ang_vel_ * sin(robot_theta + real_ang_vel_ * det_t);
    y = circle_y - real_lin_vel_ / real_ang_vel_ * cos(robot_theta + real_ang_vel_ * det_t);
    ow = cos((robot_theta + real_ang_vel_ * det_t) / 2);
    oz = sin((robot_theta + real_ang_vel_ * det_t) / 2);
    robot_to_map_.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,oz,ow),tf::Vector3(x,y,0)),
                                ros::Time::now(),"map","base_footprint"));
    ros::Duration(0.0095).sleep();
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "robot_kinematic_model");
    
    Robot_Model robot_model_test;

    return 0;
}
