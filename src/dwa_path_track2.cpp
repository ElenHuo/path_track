#include "dwa_path_track2.h"

//本文件用于手动添加静态障碍物，用于做静态障碍物避障实验


One_Particle::One_Particle()
:predict_step_(2),evaluate_value_(0),predicte_time_(12)
{}

One_Particle::One_Particle(nav_msgs::Path path , nav_msgs::OccupancyGrid map)
:predict_step_(2),evaluate_value_(0),predicte_time_(12)
{
    productParticle(path , map);
}

One_Particle::One_Particle(One_Particle* particle , nav_msgs::Path path , nav_msgs::OccupancyGrid map)
:predict_step_(2),evaluate_value_(0),predicte_time_(12)
{
    first_time_ = particle->first_time_;
    for(int i = 0;i < particle->speed_encode_.size();i++)
    {
        speed_encode_.push_back(particle->speed_encode_[i]);
    }

    generateTrajectory();

    setEvaluateValue(path , map);
}

One_Particle::~One_Particle()
{}

void One_Particle::productParticle(nav_msgs::Path path , nav_msgs::OccupancyGrid map)
{
    speed_encode_.clear();
    for(int i = 0;i < predict_step_;i++)
    {
        speed_encode_.push_back(rand() % 26);
    }
    
    double v,w;
    v = fabs(max_linear_vel);
    w = v / move_radius_[speed_encode_[0]];
    if(fabs(w) > fabs(max_angular_vel))
    {
        v = max_angular_vel * fabs(move_radius_[speed_encode_[0]]);
        w = v / move_radius_[speed_encode_[0]];
    }
    first_time_ = (double)(Random(300)) / 25.0; 
    while(first_time_ > fabs(3.1415926 / w))
    {
        first_time_ = (double)(Random(300)) / 25.0;
    }

    generateTrajectory();

    setEvaluateValue(path , map);
}

void One_Particle::neighbourSearch(One_Particle* particle , nav_msgs::Path path , nav_msgs::OccupancyGrid map)
{
    for(int i = 0;i < speed_encode_.size();i++)
    {
        while(ros::ok())
        {
            speed_encode_[i] = particle->speed_encode_[i] + Random(105) % 7 - 3;
            if(speed_encode_[i] >= 0 && speed_encode_[i] <= 26)
            {
                break;
            }
        }
        while(ros::ok())
        {
            double rand_num = Random(100);
            double det_ttt = rand_num / 100.0;
            first_time_ = particle->first_time_ + det_ttt;
            if(first_time_ > 0 && first_time_ < predicte_time_)
            {
                break;
            }
        }
    }

    generateTrajectory();

    setEvaluateValue(path , map);
}

void One_Particle::generateTrajectory()
{
    car_in_map_.position.x = car_in_map_g->x();
    car_in_map_.position.y = car_in_map_g->y();
    trajectory_point_.clear();
    double v0_x = real_linear_vel;
    double v0oz = real_angular_vel;
    double pre_car_x = car_in_map_g->x();
    double pre_car_y = car_in_map_g->y();
    double pre_car_theta = acos(2 * car_in_map_g->ow() * car_in_map_g->ow() - 1);
    pre_car_theta = car_in_map_g->oz() * car_in_map_g->ow() > 0 ? pre_car_theta : (0 - pre_car_theta);

    for(int i = 0;i < speed_encode_.size();i++)
    {
        double v0 , w0;
        double circle_x , circle_y;
        v0 = 0.3;
        w0 = v0 / move_radius_[speed_encode_[i]];
        if(fabs(w0) > 0.3)
        {
            v0 = 0.3 * fabs(move_radius_[speed_encode_[i]]);
            w0 = v0 / move_radius_[speed_encode_[i]];
        }

        double t1 = (i == 1 ? predicte_time_ - first_time_ : first_time_);
        for(int j = 0;j < t1 * 10;j++)
        {
            double det_tt = 0.1;
            double v , w;

            v = fabs(v0 - v0_x) / det_tt > max_linear_acc ? v0_x + (v0 - v0_x) / fabs(v0 - v0_x) * max_linear_acc * det_tt : v0;
            w = fabs(w0 - v0oz) / det_tt > max_angular_acc ? v0oz + (w0 - v0oz) / fabs(w0 - v0oz) * max_angular_acc * det_tt : w0;
            
            // if(fabs(v0 - v0_x) / det_tt > max_linear_acc)
            // {
            //     v = v0_x + (v0 - v0_x) / fabs(v0 - v0_x) * max_linear_acc * det_tt;
            // }
            // else
            // {
            //     v = v0;
            // }
            // if(fabs(w0 - v0oz) / det_tt > max_angular_acc)
            // {
            //     w = v0oz + (w0 - v0oz) / fabs(w0 - v0oz) * max_linear_acc * det_tt;
            // }
            // else
            // {
            //     v = w0;
            // }
            v = v0;
            w = w0;

            circle_x = pre_car_x - v / w * sin(pre_car_theta);
            circle_y = pre_car_y + v / w * cos(pre_car_theta);
            pre_car_x = circle_x + v / w * sin(pre_car_theta + w * det_tt);
            pre_car_y = circle_y - v / w * cos(pre_car_theta + w * det_tt);
            pre_car_theta = pre_car_theta + w * det_tt;
            geometry_msgs::Point32 point;
            point.x = pre_car_x;
            point.y = pre_car_y;
            point.z = 0;
            trajectory_point_.push_back(point);
        }
    }
}

void One_Particle::setEvaluateValue(nav_msgs::Path path,nav_msgs::OccupancyGrid map)
{
    evaluate_value_ = 0;
    nav_msgs::Path path2 = path;
    double min_dis = 100000;
    int n = 0;
    for(int i = 0;i < path2.poses.size();i++)
    {
        double det_x2 = car_in_map_.position.x - path2.poses[i].pose.position.x;
        double det_y2 = car_in_map_.position.y - path2.poses[i].pose.position.y;
        if(min_dis > det_x2 * det_x2 + det_y2 * det_y2)
        {
            min_dis = det_x2 * det_x2 + det_y2 * det_y2;
            n = i;
        }
    }
    path2.poses.erase(path2.poses.begin(),path2.poses.begin() + n);

    for(int i = 1;i < 5;i++)
    {
        if(i >= path2.poses.size())
        {
            break;
        }
        double det_x = path2.poses[0].pose.position.x - path2.poses[i].pose.position.x;
        double det_y = path2.poses[0].pose.position.y - path2.poses[i].pose.position.y;
        double distance = sqrt(det_x * det_x + det_y * det_y);
        for(int j = 0;j < trajectory_point_.size();j++)
        {
            double x0 = car_in_map_g->x() - trajectory_point_[j].x;
            double y0 = car_in_map_g->y() - trajectory_point_[j].y;
            if(fabs(sqrt(x0 * x0 + y0 * y0) -distance) < 0.05)
            {
                double x1 = path2.poses[i].pose.position.x - trajectory_point_[j].x;
                double y1 = path2.poses[i].pose.position.y - trajectory_point_[j].y;
                evaluate_value_ += sqrt(x1 * x1 + y1 * y1);
                break;
            }
            if(j == trajectory_point_.size() - 1)
            {
                double x1 = path2.poses[i].pose.position.x - trajectory_point_[j].x;
                double y1 = path2.poses[i].pose.position.y - trajectory_point_[j].y;
                evaluate_value_ += 3;
            }
        }
    }

    evaluate_value_ += (1 / exp(fabs(speed_encode_[0] - 12.5)));

    evaluate_value_ += (0.1 / exp(first_time_ / predicte_time_));

    if(trajectoryVelidCheck() == 0)
    {
        evaluate_value_ += 15;
    }
}

void One_Particle::displayTrajectory()
{
    sensor_msgs::PointCloud path;
    path.header.frame_id = "map";
    path.points.resize(trajectory_point_.size());
    for(int i = 0;i < trajectory_point_.size();i++)
    {
        path.points[i].x = trajectory_point_[i].x;
        path.points[i].y = trajectory_point_[i].y;
        path.points[i].z = trajectory_point_[i].z;
    }
    pub_path_rviz_.publish(path);
    cout << "the evaluate value is:" << evaluate_value_ << endl;
    cout << "the sample param is: " << move_radius_[speed_encode_[0]] << ", " <<move_radius_[speed_encode_[1]] << ", " << first_time_ << endl;
}

bool One_Particle::trajectoryVelidCheck()
{
    for(int j = 0;j < trajectory_point_.size();j++)
    {
        double xx = trajectory_point_[j].x - trajectory_point_[0].x;
        double yy = trajectory_point_[j].y - trajectory_point_[0].y;
        if(sqrt(xx * xx + yy * yy) > 1)
        {
            break;
        }
        int x = trajectory_point_[j].x * 10;
        int y = trajectory_point_[j].y * 10 + 15;
        int x1 = x + 1;
        int x2 = x - 1;
        x = x < 0 ? 0 : x;
        x = x > 29 ? 29 : x;
        y = y < 0 ? 0 : y;
        y = y > 29 ? 29 : y;
        if(map_g[x][y] == 1 || map_g[x1][y] == 1 || map_g[x2][y] == 1 )
        {
            return 0;
        }
    }
    return 1;
}

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

Dwa_Path_Track::Dwa_Path_Track()
:get_path_flag_(0),get_map_flag_(0),particle_num_(400),best_particle_(NULL)
{
    car_in_map_g = boost::make_shared<Tf_Listerner>("map" , "base_footprint");
    pub_velocity_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel",5);
	sub_dwa_map_ = n_.subscribe("/move_base/local_costmap/costmap",1,&Dwa_Path_Track::subDwaMap,this);
	sub_path_ = n_.subscribe("/own_path",1,&Dwa_Path_Track::subPath,this);
    pub_path_rviz_ = n_.advertise<sensor_msgs::PointCloud>("/predicte_path",2);
    loadMap();

    pthreadTwo();

    ros::spin();
}

Dwa_Path_Track::~Dwa_Path_Track()
{
    geometry_msgs::Twist velocity;
    velocity.linear.x = 0;
    velocity.angular.z = 0;
    pub_velocity_.publish(velocity);
    ros::Duration(0.3).sleep();
    pub_velocity_.publish(velocity);
    ros::Duration(0.3).sleep();
    pub_velocity_.publish(velocity);
    ros::Duration(0.3).sleep();
    pub_velocity_.publish(velocity);
    cout << "stop the dwa path track2 program." << endl;
}

void Dwa_Path_Track::dynamicCb(path_track::path_track_Config &config,uint32_t level)
{
    cout << "load the dynamic param." << endl;
}

void Dwa_Path_Track::subDwaMap(nav_msgs::OccupancyGrid map)
{
    dwa_map_ = map;
    get_map_flag_ = 1;
}

void Dwa_Path_Track::subPath(nav_msgs::Path path)
{
    path_ = path;
    get_path_flag_ = 1;
}

void Dwa_Path_Track::pthreadTwo()
{
    if(pthread_create(&m_tid_,NULL,threadTwo,(void*)this) != 0)
    {
        std::cout << "Start tf listerner thread failed!" << std::endl;
        return; 
    }
}

void* Dwa_Path_Track::threadTwo(void * arg)
{
    Dwa_Path_Track *ptr =(Dwa_Path_Track*) arg;
    ptr->threadRunTwo();
    return NULL;
}

void Dwa_Path_Track::threadRunTwo()
{
    cout << "start dwa path track." << endl;
    while(ros::ok())
    {
        // if(get_map_flag_ == 0) 
        // {
        //     cout << "there is not a map!" << endl;
        //     ros::Duration(1).sleep();
        //     continue;
        // }

        if(get_path_flag_ == 0)
        {
            cout << "there is not a path!" << endl;
            ros::Duration(1).sleep();
            continue;
        }
        else if(get_path_flag_ == 1)
        {
            get_path_flag_ = 2;
            for(int i = 0;i < particle_swarm_.size();i++)
            {
                delete particle_swarm_[i];
            }
            particle_swarm_.clear();
            if(best_particle_ != NULL)
            {
                particle_swarm_.push_back(new One_Particle(best_particle_  , path_ , dwa_map_));
            }
            for(int i = 1;i < particle_num_;i++)
            {
                particle_swarm_.push_back(new One_Particle(path_,dwa_map_));
            }
            sort(particle_swarm_.begin() , particle_swarm_.end() , lessThan);
        }
        else if(get_path_flag_ == 2)
        {
            get_path_flag_ = 1;
            double x3 = car_in_map_g->x() - path_.poses[path_.poses.size() - 1].pose.position.x;
            double y3 = car_in_map_g->y() - path_.poses[path_.poses.size() - 1].pose.position.y;
            if(sqrt(x3 * x3 + y3 * y3) < 0.5)
            {
                get_path_flag_ = 3;
                cout << "agv is nearby to destination." << endl;
                geometry_msgs::Twist velocity;
                velocity.linear.x = 0;
                velocity.angular.z = 0;
                pub_velocity_.publish(velocity);
                continue;
            }

            for(int i = 0;i < 6;i++)
            {
                for(int j = 1;j < particle_swarm_.size() / 5;j++)
                {
                    particle_swarm_[j]->neighbourSearch(particle_swarm_[0] , path_ , dwa_map_);
                }
                for(int j = particle_swarm_.size() / 5;j < particle_swarm_.size();j++)
                {
                    particle_swarm_[j]->productParticle(path_,dwa_map_);
                }
            }
            sort(particle_swarm_.begin() , particle_swarm_.end() , lessThan);
            particle_swarm_[0]->displayTrajectory();
            pubVelocity(particle_swarm_[0]);
            delete best_particle_;
            best_particle_ = new One_Particle(particle_swarm_[0] , path_ , dwa_map_);
        }
    }
}

void Dwa_Path_Track::pubVelocity(One_Particle* particle)
{
    static double last_time = ros::Time::now().toSec();
    double delta_t = ros::Time::now().toSec() - last_time;
    last_time = ros::Time::now().toSec();
    delta_t = delta_t == 0 ? 0.1 : delta_t;
    geometry_msgs::Twist velocity;
    double v_x, voz;
    v_x = max_linear_vel;
    voz = v_x / particle->move_radius_[particle->speed_encode_[0]];
    if(fabs(voz) > max_angular_vel)
    {
        v_x = max_angular_vel * fabs(particle->move_radius_[particle->speed_encode_[0]]);
        voz = v_x / particle->move_radius_[particle->speed_encode_[0]];
    }
    if(v_x > 3 || voz > 1)
    {
        cout << "velocity command is error!!!" << endl;
    }
    else
    {
        velocity.linear.x = fabs(real_linear_vel - v_x) / delta_t > max_linear_acc ? real_linear_vel + (v_x - real_linear_vel) / fabs(real_linear_vel - v_x) * max_linear_acc * delta_t : v_x;
        velocity.angular.z = fabs(real_angular_vel - voz) / delta_t > max_angular_acc ? real_angular_vel + (voz - real_angular_vel) / fabs(real_angular_vel - voz) * max_angular_acc * delta_t : voz;
        real_linear_vel = velocity.linear.x;
        real_angular_vel = velocity.angular.z;
        cout << "the output velocity is: " << velocity.linear.x << " , " << velocity.angular.z << " , " << delta_t << endl << endl;
        pub_velocity_.publish(velocity);
    }
}

void Dwa_Path_Track::loadMap()
{
    for(int i = 0; i < 30; i++)
    {
        for(int j = 0; j < 30; j++)
        {
            map_g[i][j] = 0;
        }
    }
    // map_g[6][12] = 1;
    // map_g[6][13] = 1;
    // map_g[6][14] = 1;
    // map_g[6][15] = 1;
    // map_g[6][16] = 1;
    // map_g[7][12] = 1;
    // map_g[7][13] = 1;
    // map_g[7][14] = 1;
    // map_g[7][15] = 1;
    // map_g[7][16] = 1;

    // map_g[15][13] = 1;
    // map_g[15][14] = 1;
    // map_g[15][15] = 1;
    // map_g[15][16] = 1;
    // map_g[15][17] = 1;
    // map_g[16][13] = 1;
    // map_g[16][14] = 1;
    // map_g[16][15] = 1;
    // map_g[16][16] = 1;
    // map_g[16][17] = 1;


    // map_g[25][13] = 1;
    // map_g[25][14] = 1;
    // map_g[25][15] = 1;
    // map_g[25][16] = 1;
    // map_g[26][13] = 1;
    // map_g[26][14] = 1;
    // map_g[26][15] = 1;
    // map_g[26][16] = 1;
    

    /////////////sssssssssss///////////
    map_g[6][6] = 1;
    map_g[6][7] = 1;
    map_g[6][8] = 1;
    map_g[6][9] = 1;
    map_g[6][10] = 1;
    map_g[7][6] = 1;
    map_g[7][7] = 1;
    map_g[7][8] = 1;
    map_g[7][9] = 1;
    map_g[7][10] = 1;


    map_g[15][14] = 1;
    map_g[15][15] = 1;
    map_g[15][16] = 1;
    map_g[15][17] = 1;
    map_g[15][18] = 1;
    map_g[16][14] = 1;
    map_g[16][15] = 1;
    map_g[16][16] = 1;
    map_g[16][17] = 1;
    map_g[16][18] = 1;


    // map_g[24][19] = 1;
    // map_g[24][20] = 1;
    // map_g[24][21] = 1;
    // map_g[24][22] = 1;
    // map_g[25][19] = 1;
    // map_g[25][20] = 1;
    // map_g[25][21] = 1;
    // map_g[25][22] = 1;
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "dwa_path_track");
    
    Dwa_Path_Track path_track_test;

    return 0;
}

//////////////////////////////////////////////////test
void displayTrajectory(vector<geometry_msgs::Point32> trajectory_point , double evaluate_value)
{
    sensor_msgs::PointCloud path;
    path.header.frame_id = "map";
    path.points.resize(trajectory_point.size());
    for(int i = 0;i < trajectory_point.size();i++)
    {
        path.points[i].x = trajectory_point[i].x;
        path.points[i].y = trajectory_point[i].y;
        path.points[i].z = trajectory_point[i].z;
    }
    pub_path_rviz_.publish(path);
    cout << "the evaluate value is:" << evaluate_value << endl;
}
//////////////////////////////////////////////////test
