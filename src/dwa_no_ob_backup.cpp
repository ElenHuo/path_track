#include "dwa.h"

DWA::DWA():
load_path_success(0),running_flag(0), count(0), sample_time(0.1),max_vel(1.0), max_w(0.5), max_vel_acc(0.2) 
,max_w_acc(1.0), a(0.05) ,b(0.2) , c(0.1), R_inflation(0.5),dis_thre(0.2) 
{
    dynamic_reconfigure::Server<path_track::DWA_Config> server;
    dynamic_reconfigure::Server<path_track::DWA_Config>::CallbackType f;
    f = boost::bind(&DWA::dynamicCb,this,_1,_2);
    server.setCallback(f);

    car_in_map_ = new Tf_Listerner("map","base_footprint");
    sub_ = n_.subscribe("/own_path", 1 ,&DWA::sub_pathCB,this);
    pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel",2);
    PC_pub_ = n_.advertise<sensor_msgs::PointCloud>("/pc_test",2);
    Line_pub_ = n_.advertise<sensor_msgs::PointCloud>("/Line",2);
    while(ros::ok())
    {
        ros::spinOnce();
        double t0 = ros::Time::now().toSec();
        CurrentSTATE();
        if(load_path_success == 1)
        {
            /* 算出车子位置最靠近到路径点 */
            double contrast = 9999 ;
            int index = 0;
            for(int i = 0; i < PATH.poses.size();i++)
            {
                double delta_x = Current_State.x - PATH.poses[i].pose.position.x;
                double delta_y = Current_State.y - PATH.poses[i].pose.position.y;
                double distance =sqrt( delta_x * delta_x + delta_y * delta_y ) ;
                if(contrast > distance )
                {
                    contrast = distance;
                    index = i;
                }
            }
            count = index; //算出车子位置最靠近到路径点

            ////////////////////////////////
            goalpoint.clear();
            length.clear();
            int k = 1; //往后3个点
            for(; k < 5 ; k++)
            {
                if(count + k  >= PATH.poses.size() - 1 )break; 
            }

            if(count != PATH.poses.size() - 1)
            {
                for(int i = 1;i<= k-1 ;i++)
                {
                    GoalPoint temp_point;
                    temp_point.x = PATH.poses[count+i].pose.position.x ;
                    temp_point.y = PATH.poses[count+i].pose.position.y ;
                   // cout <<"WAY_POINTS: "<< temp_point.x <<"," <<temp_point.y <<endl;
                    goalpoint.push_back(temp_point);
                    
                }

            }
            double de_x = Current_State.x - PATH.poses[PATH.poses.size()-1].pose.position.x;
            double de_y = Current_State.y - PATH.poses[PATH.poses.size()-1].pose.position.y;
            double thre = sqrt(de_x *de_x + de_y*de_y);
            if(thre <  dis_thre)
            {
                ROS_INFO("Arrive destination!");
                running_flag = 0;   
                geometry_msgs::Twist STOP;
                STOP.linear.x = 0;
                STOP.angular.z = 0;       
                Current_State.v = 0;
                Current_State.w = 0;    
                pub_.publish(STOP);
                count = 0;
            }

            if(goalpoint.empty() == false)
            {
                for(int i = 0 ; i < goalpoint.size(); i++)
                {
                    double delta_x1 = goalpoint[i].x - PATH.poses[count].pose.position.x;
                    double delta_y1 = goalpoint[i].y - PATH.poses[count].pose.position.y;
                    double L1 = sqrt (delta_x1*delta_x1 + delta_y1*delta_y1);
                    length.push_back(L1);
                   // cout << "Length: "<<L1<<endl;
                }

            }
        
            if(running_flag == 1 )
            {
                goal_x = PATH.poses[count].pose.position.x;
                goal_y = PATH.poses[count].pose.position.y;
                //ROS_INFO("Calculation Velocity Space");
                Cal_Velocity_Space();
                Motion_model();// Traj generate
            }

        }
        
        cout<<"Running time :" <<ros::Time::now().toSec() - t0<<endl;
    }    

    //ros::spin();//monitor CB function
}
DWA::~DWA()
{
    ROS_INFO("DWA Programme is end!!");
}

void DWA::dynamicCb(path_track::DWA_Config &config,uint32_t level)
{
    cout << "load the dynamic param." << endl;
    sample_time = config.sample_time;
    max_vel = config.max_vel;
    max_w = config.max_w;
    max_vel_acc = config.max_vel_acc;
    max_w_acc = config.max_w_acc;
    a = config.a;
    b = config.b;
    c = config.c;
    R_inflation = config.R_inflation;
    dis_thre = config.dis_thre;


}

void DWA::CurrentSTATE() //机器人当前状态
{
    Current_State.x = car_in_map_->x();
    Current_State.y = car_in_map_->y();
     double tem_theta = acos ( 2 * car_in_map_->ow() * car_in_map_->ow() -1 );
     Current_State.theta = car_in_map_->ow() * car_in_map_->oz() > 0 ? tem_theta : (-tem_theta);
   
}
void DWA::sub_pathCB(nav_msgs::Path GlobalPath)
{
    PATH = GlobalPath;
    load_path_success = 1;
    running_flag = 1;
    geometry_msgs::Twist STOP;
    STOP.linear.x = 0;
    STOP.angular.z = 0;       
    Current_State.v = 0;
    Current_State.w = 0;    
    pub_.publish(STOP);
    count = 0;
    cout<< "Path got!"<<endl;
}

void DWA::Cal_Velocity_Space()
{
    double Vs[4]={ 0.0 , max_vel , -max_w , max_w };
    //根据加速度限制计算动态窗口，依次为：最小速度 最大速度 最小角速度 最大角速度
     double Vd[4]={ Current_State.v - max_vel_acc * sample_time , Current_State.v + max_vel_acc * sample_time ,
      Current_State.w - max_w_acc * sample_time ,  Current_State.w + max_w_acc * sample_time };
    double Vtmp1 = Vs[0] >  Vd[0] ? Vs[0] : Vd[0];
    double Vtmp2 = Vs[1] <  Vd[1] ? Vs[1] : Vd[1];
    double Vtmp3 = Vs[2] >  Vd[2] ? Vs[2] : Vd[2];
    double Vtmp4 = Vs[3] <  Vd[3] ? Vs[3] : Vd[3];
    Vr[0] = Vtmp1, Vr[1] = Vtmp2, Vr[2] = Vtmp3, Vr[3] = Vtmp4;
}

void DWA::Motion_model()
{  
    double heading_sum = 0;
    double Vel_sum = 0;
   
    Traj.clear();
    ROBOT_STATE.clear();
    Send_PTS.header.frame_id = "map";
    Send_PTS.points.clear();
    for(double Vt = Vr[0] ; Vt <= Vr[1] ; )
    {
        for(double Wt = Vr[2]; Wt <= Vr[3] ; )
        {
            STATE temp_state = Current_State;
            sensor_msgs::PointCloud PTS;
            int length_index = 0;
            PTS.header.frame_id = "map";
            for(double i = 0; i <= predict_time ;) //点离散
            {   
                geometry_msgs::Point32 Point;
                if(Wt == 0 )Wt = 0.001;
                temp_state.x = temp_state.x - Vt/Wt * sin(temp_state.theta) + Vt/Wt * sin(temp_state.theta + Wt * sample_time);
                temp_state.y = temp_state.y + Vt/Wt * cos(temp_state.theta) - Vt/Wt * cos(temp_state.theta + Wt * sample_time);
                temp_state.theta = temp_state.theta + Wt * sample_time;
                i = i + sample_time;
                Point.x = temp_state.x;
                Point.y = temp_state.y;               
                PTS.points.push_back(Point);
                Send_PTS.points.push_back(Point);

                if(length_index <= length.size() - 1)
                {
                    double d_x = Point.x - Current_State.x;
                    double d_y = Point.y - Current_State.y;
                    double p2p_dis = sqrt(d_x * d_x + d_y * d_y);

                    if(p2p_dis >= length[length_index])
                    {
                        double del_x = Point.x - goalpoint[length_index].x; 
                        double del_y = Point.y - goalpoint[length_index].y;
                        double del_dis = sqrt(del_x *del_x + del_y * del_y);
                        temp_state.STORE.push_back(del_dis);
                        length_index++;
                    }
                }

                
            }
            if(length_index <=length.size() - 1) //补全STORE储存到点数
            {
                double insert_dis = 2;
                temp_state.STORE.push_back(insert_dis);
                length_index++;
            }

            temp_state.heading = distance_Heading(temp_state);
            /////////////////TEST1/////////////
            //temp_state.heading = Cal_Heading(temp_state);

            ///////////////TEST2/////////////////
            //tf::Vector3 v2(cos(temp_state.theta), sin(temp_state.theta) , 0);  //向量
            //temp_state.heading = Cal_Vector_Heading(V1,v2);

            temp_state.v = Vt;
            temp_state.w = Wt;
            temp_state.Vel_Evaluation = abs(Vt);
            heading_sum  += temp_state.heading;
            Vel_sum +=  temp_state.Vel_Evaluation;
            Traj.push_back(PTS); //储存点云
            ROBOT_STATE.push_back(temp_state);
            Wt = Wt + w_res;
        }
        Vt = Vt + vel_res;
    }

    /* 归一化 */
    // NormalizeEval();
    double comparisson = 9999 ;
    int label = 0;
    for(int i = 0 ; i < ROBOT_STATE.size(); i++)
    {
        if( ROBOT_STATE[i].STORE.size() !=0)
        {
            //ROBOT_STATE[i].Vel_Evaluation /= Vel_sum;
            ROBOT_STATE[i].Score = a * ROBOT_STATE[i].heading;
            if (ROBOT_STATE[i].Score < comparisson) 
            {
                label = i;
                comparisson = ROBOT_STATE[i].Score;
            }
        }
       // ROBOT_STATE[i].heading /=  heading_sum;

        //cout<<"Score: " <<ROBOT_STATE[i].Score<<endl;

        
    }
    //cout<< "V、W 、SCORE："<< ROBOT_STATE[label].v <<","<<ROBOT_STATE[label].w<<","<<ROBOT_STATE[label].Score<<endl;

    // for(int i = 0 ; i < ROBOT_STATE.size(); i++)
    // {
    //     ROBOT_STATE[i].heading /=  heading_sum;
    //     ROBOT_STATE[i].Vel_Evaluation /= Vel_sum;
    //     ROBOT_STATE[i].Score = a * ROBOT_STATE[i].heading + c * ROBOT_STATE[i].Vel_Evaluation;
    //     if (ROBOT_STATE[i].Score > comparisson) 
    //     {
    //         label = i;
    //         comparisson = ROBOT_STATE[i].Score;
    //     }
        
    // }
    geometry_msgs::Twist VW;
    VW.linear.x = ROBOT_STATE[label].v;
    VW.angular.z = ROBOT_STATE[label].w;
    Current_State.v = ROBOT_STATE[label].v;
    Current_State.w = ROBOT_STATE[label].w;
    //cout<< "V,W:"<<Current_State.v <<" , " <<Current_State.w <<endl;
    pub_.publish(VW);
    PC_pub_.publish(Send_PTS);
    Line_pub_.publish(Traj[label]);
    //////////////////////////test///////////////////////////
    // for(int i = 0; i < Traj.size();i++)
    // {
        
    //     Send_PTS.points.push_back(Traj[i].points);
        
    // }
    

}

double DWA::Cal_Heading(STATE A)  //Evaluation of Heading
{
    double theta_degree = toDegree(A.theta);   
    double goal_theta = toDegree ( atan2( (goal_y - A.y) , (goal_x - A.x) ) );
    double target_theta = goal_theta > theta_degree ? (goal_theta-theta_degree) : (theta_degree - goal_theta);
    return (180.0f - target_theta);
}

 
/*                      效果不佳                      */
double DWA::Cal_Vector_Heading (tf::Vector3 Vec1, tf::Vector3 Vec2)
{
    double target_theta = tfAngle(Vec1,Vec2);
    return (180.0f - target_theta);
}

double DWA::distance_Heading(STATE A)
{   
    double integrate = 0;
    for(int i = 0;  i< A.STORE.size() ; i++)
    {
        integrate += A.STORE[i];
    }
    return integrate / A.STORE.size();

}

void DWA::NormalizeEval()
{
    // for(int i = 0 ; i < ROBOT_STATE.size(); i++)
    // {
    //     heading_sum += ROBOT_STATE[i].heading;
    //     Vel_sum +=  ROBOT_STATE[i].Vel_Evaluation;
    // }
    
}

double DWA::toDegree(double THETA)
{
    return  THETA / PI * 180.0f;
}


 int main(int argc, char *argv[])
{
    /* code for main function */
    ros::init(argc, argv, "dwa_local_planning");
    DWA DWA;
    return 0;
}

