#include "path_track.h"

Path_Track::Path_Track()
:angle_bias_(0),distance_biasY_(0),complete_path_track_flag_(2),calculated_frequency_(20.0)
,pid_p_(3),pid_i_(0),pid_d_(1),ave_vel_(1)
{
    dynamic_reconfigure::Server<path_track::path_track_Config> server;
    dynamic_reconfigure::Server<path_track::path_track_Config>::CallbackType f;
    f = boost::bind(&Path_Track::dynamicCb,this,_1,_2);
    server.setCallback(f);

    current_position_ = new Tf_Listerner("map","base_footprint");
    pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel",2);
    pub_arrived_destination_ = n_.advertise<std_msgs::String>("whether_arrived_destination",1);
    sub_ = n_.subscribe("/own_path",2,&Path_Track::callBackPath,this);
    sub_destination_ = n_.subscribe("/destination_pose",2,&Path_Track::callBackDestination,this);
    ros::Duration(3).sleep();
    cout << "begin the path track program." << endl;

    pthreadTwo();
    ros::spin();
}

Path_Track::~Path_Track()
{}

void Path_Track::pidControl()
{
    getBias();
    //ROS_INFO("angle_bias is:%f",angle_bias_);
    integral_part_ = integral_part_ + angle_bias_ / calculated_frequency_;
    angle_velocity_ = pid_p_ * angle_bias_ +pid_i_ * integral_part_ + pid_d_ * (angle_bias_ - last_bias_) * calculated_frequency_;
    
    last_bias_ = angle_bias_;
}

void Path_Track::callBackPath(const nav_msgs::Path::ConstPtr& path_point)
{
    integral_part_ = 0;
    last_bias_ = 0;

    pose position;
    path_.clear();
    for(int i = 0;i < path_point->poses.size();i++)
    {
        position.x = path_point->poses[i].pose.position.x;
        position.y = path_point->poses[i].pose.position.y;
        if(i < path_point->poses.size() - 1)
        {
            float x;
            float y;
            float ow;
            float oz;
            x = path_point->poses[i + 1].pose.position.x - position.x;
            y = path_point->poses[i + 1].pose.position.y - position.y;
            ow = cos(acos(x / sqrt(x * x + y * y)) / 2);
            oz = sqrt(1 - ow * ow);
            if(y < 0) oz = 0 - oz;
            position.ow = ow;
            position.oz = oz;

            path_.push_back(position);
        }
        else
        {
            float x1 = path_point->poses[i].pose.position.x - destination_.position.x;
            float y1 = path_point->poses[i].pose.position.y - destination_.position.y;
            float ow1 = cos(acos(x1 / sqrt(x1 * x1 + y1 * y1)) / 2);
            float oz1 = sqrt(1 - ow1 * ow1);
            if(y1 < 0) oz1 = 0 - oz1;
            position.ow = ow1;
            position.oz = oz1;

            if((x1 * x1 + y1 * y1) > 0.01)
            {
                path_.push_back(position);
                position.x = destination_.position.x;
                position.y = destination_.position.y;
                position.ow = destination_.orientation.w;
                position.oz = destination_.orientation.z;
                path_.push_back(position);
            }
            else
            {
                position.x = destination_.position.x;
                position.y = destination_.position.y;
                position.ow = destination_.orientation.w;
                position.oz = destination_.orientation.z;
                path_.push_back(position);
            }
        }
    }

    complete_path_track_flag_ = 1;
}

void Path_Track::callBackDestination(const geometry_msgs::Pose::ConstPtr& destination_pose)
{
    if(complete_path_track_flag_ != 0)
    {
        destination_.position.x = destination_pose->position.x;
        destination_.position.y = destination_pose->position.y;
        destination_.position.z = destination_pose->position.z;
        destination_.orientation.x = destination_pose->orientation.x;
        destination_.orientation.y = destination_pose->orientation.y;
        destination_.orientation.z = destination_pose->orientation.z;
        destination_.orientation.w = destination_pose->orientation.w;
        complete_path_track_flag_ = 0;
    }
}

void Path_Track::getBias()
{
    float bias;
    int n = 0;
    float min_distance = 1.0;
    float x;
    float y;
    float oz;
    float ow;
    x = current_position_->x();
    y = current_position_->y();
    oz = current_position_->oz();
    ow = current_position_->ow();
    if(ow < 0)
    {
        ow = 0 - ow;
        oz = 0 - oz;
    }

    for(int i = 0;i < path_.size() - 1;i++)
    {
        float distance;
        distance = sqrt((x-path_[i].x)*(x-path_[i].x)+(y-path_[i].y)*(y-path_[i].y));

        if(distance < min_distance)
        {
            min_distance = distance;
            n = i;
        }
    }
    if(n > path_.size() - 2)
    {
        n = path_.size() - 2;
    }

    if(path_.size() == 1)
    {
        n = -1;
    }

    tf::Quaternion agv_in_map(0,0,oz,ow);
    tf::Matrix3x3 matrix;
    matrix.setRotation(agv_in_map);
    distance_biasY_ = matrix[0][1] * (path_[n + 1].x - x) + matrix[1][1] * (path_[n + 1].y - y);

    double sin_angle_bias;
    double cos_angle_bias;
    double distance_bias;

    distance_bias = sqrt((path_[n + 1].x - x) * (path_[n + 1].x - x) + (path_[n + 1].y - y) * (path_[n + 1].y - y));
    cos_angle_bias = (matrix[0][0] * (path_[n + 1].x - x) + matrix[1][0] * (path_[n + 1].y - y)) / distance_bias;
    sin_angle_bias = distance_biasY_ / distance_bias;
    angle_bias_ = acos(cos_angle_bias);
    if(sin_angle_bias < 0)
    {
        angle_bias_ = 0 - angle_bias_;
    }
    if(matrix[2][2] < 0)
    {
        angle_bias_ = 0 - angle_bias_;
    }
    // ROS_INFO("current:%f,%f,%f,%f.",x,y,ow,oz);
    // ROS_INFO("hope   :%f,%f,%f,%f.",path_[n+1].x,path_[n+1].y,path_[n+1].ow,path_[n+1].oz);
    // ROS_INFO("angle_bias:%f",angle_bias_);

    if(n > 0)
    {
        path_.erase(path_.begin(),path_.begin() + n);
    }
}

void Path_Track::motionConstraint()
{
    cmd_vel_.linear.x = ave_vel_;
    cmd_vel_.linear.y = 0;
    cmd_vel_.linear.z = 0;
    cmd_vel_.angular.x = 0.0;
    cmd_vel_.angular.y = 0.0;
    //cmd_vel_.angular.z = 0.0;
    if(fabs(angle_velocity_) > ave_vel_)
    {
        cmd_vel_.linear.x = cmd_vel_.linear.x * ave_vel_ / fabs(angle_velocity_);
        cmd_vel_.angular.z = angle_velocity_ / fabs(angle_velocity_) * ave_vel_;
    }
    //pub_.publish(cmd_vel_);
}

void Path_Track::sendVelocityCommand()
{
    ros::Duration(1 / calculated_frequency_).sleep();
    pidControl();
    motionConstraint();
    pub_.publish(cmd_vel_);
}

// void Path_Track::goToDestination()
// {
//     double det_x = current_position_->x() - destination_.position.x;
//     double det_y = current_position_->y() - destination_.position.y;
//     if(det_x * det_x + det_y * det_y > 0.3)
//     {
//         motionConstraint();
//     }
//     else if(fabs(goToDestinationAngle()) > 1)
//     {
//         cmd_vel_.linear.x = 0;
//         cmd_vel_.linear.y = 0;
//         cmd_vel_.linear.z = 0;
//         cmd_vel_.angular.x = 0.0;
//         cmd_vel_.angular.y = 0.0;
//         cmd_vel_.angular.z = ave_vel_;
//     }
//     else
//     {
//         complete_path_track_flag_ = 1;
//         path_.clear();
//     }
// }

// double Path_Track::goToDestinationAngle()
// {
//     double angle_of_agv;
//     double angle_of_destination;
//     angle_of_agv = 2 * acos(current_position_->ow());
//     if(current_position_->oz() < 0)
//     {
//         angle_of_agv = 0 - angle_of_agv;
//     }
//     angle_of_destination = 2 * acos(destination_.orientation.w);
//     if(destination_.orientation.z < 0)
//     {
//         angle_of_destination = 0 - angle_of_destination;
//     }
//     return angle_of_agv - angle_of_destination;
// }

void Path_Track::pthreadTwo()
{
    if(pthread_create(&m_tid_,NULL,threadTwo,(void*)this) != 0)
    {
        std::cout << "Start tf listerner thread failed!" << std::endl;
        return; 
    }
}

void* Path_Track::threadTwo(void * arg)
{
    Path_Track *ptr =(Path_Track*) arg;
    ptr->threadRunTwo();
    return NULL;
}

void Path_Track::threadRunTwo()
{
    while(ros::ok())
    {
        if((complete_path_track_flag_ == 1) && (path_.size() > 1))
        {
            sendVelocityCommand();
            // if((current_position_->x() - (path_.end() - 1)->x) * (current_position_->x() - (path_.end() - 1)->x) 
            //  + (current_position_->y() - (path_.end() - 1)->y) * (current_position_->y() - (path_.end() - 1)->y) < 1)
            if(path_.size() <= 3)
            {
                complete_path_track_flag_ = 2;
                cout << "agv arrived the position,but angular is not!" << endl;
                cmd_vel_.linear.x = 0;
                cmd_vel_.linear.y = 0;
                cmd_vel_.angular.z = 0;
                for(int i = 0;i < 5;i++)
                {
                    pub_.publish(cmd_vel_);
                    ros::Duration(0.2).sleep();
                }
            }
        }
        else
        {}
    }
}

void Path_Track::dynamicCb(path_track::path_track_Config &config,uint32_t level)
{
    cout << "load the dynamic param." << endl;
    pid_p_ = config.pid_p;
    pid_i_ = config.pid_i;
    pid_d_ = config.pid_d;
    ave_vel_ = config.ave_vel;
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "path_track");
    
    Path_Track path_track_test;

    return 0;
}
