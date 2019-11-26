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

    while(ros::ok())
    {
        ros::spinOnce();
        CurrentSTATE();
        if(load_path_success == 1)
        {
            double delta_x = Current_State.x - PATH.poses[count].pose.position.x;
            double delta_y = Current_State.y - PATH.poses[count].pose.position.y;
            double distance =sqrt( delta_x * delta_x + delta_y * delta_y ) ;
            //cout<< "Dis:"<<distance <<endl;
            if(distance < dis_thre)
            {
                count++;
                if(count >= PATH.poses.size())
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
    double tem_theta = 2 * acos ( car_in_map_->ow() );
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
            PTS.header.frame_id = "map";
            for(double i = 0; i <= predict_time ;) //点离散
            {   
                geometry_msgs::Point32 Point;
                if(Wt == 0 )Wt = 0.001;
                temp_state.x = temp_state.x - Vt/Wt * sin(temp_state.theta) + Vt/Wt * sin(temp_state.theta + Wt * sample_time);
                temp_state.y = temp_state.y + Vt/Wt * cos(temp_state.theta) - Vt/Wt * cos(temp_state.theta + Wt * sample_time);
                temp_state.theta = temp_state.theta + Wt * sample_time;
                temp_state.v = Vt;
                temp_state.w = Wt;
                i = i + sample_time;
                Point.x = temp_state.x;
                Point.y = temp_state.y;               
                PTS.points.push_back(Point);
                Send_PTS.points.push_back(Point);
                
            }
            temp_state.heading = Cal_Heading(temp_state);
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
    double comparisson = 0 ;
    int label = 0;
    for(int i = 0 ; i < ROBOT_STATE.size(); i++)
    {
        ROBOT_STATE[i].heading /=  heading_sum;
        ROBOT_STATE[i].Vel_Evaluation /= Vel_sum;
        ROBOT_STATE[i].Score = a * ROBOT_STATE[i].heading + c * ROBOT_STATE[i].Vel_Evaluation;
        if (ROBOT_STATE[i].Score > comparisson) 
        {
            label = i;
            comparisson = ROBOT_STATE[i].Score;
        }
        
    }
    geometry_msgs::Twist VW;
    VW.linear.x = ROBOT_STATE[label].v;
    VW.angular.z = ROBOT_STATE[label].w;
    Current_State.v = ROBOT_STATE[label].v;
    Current_State.w = ROBOT_STATE[label].w;
    cout<< "V,W:"<<Current_State.v <<" , " <<Current_State.w <<endl;
    pub_.publish(VW);
    PC_pub_.publish(Send_PTS);
    //////////////////////////test///////////////////////////
    // for(int i = 0; i < Traj.size();i++)
    // {
        
    //     Send_PTS.points.push_back(Traj[i].points);
        
    // }
    

}

double DWA::Cal_Heading(STATE A)  //Evaluation of Heading
{
    double theta_degree = toDegree(A.theta);   /* GOAL没给！！！！！！！ */
    double goal_theta = toDegree ( atan2( (goal_y - A.y) , (goal_x - A.x) ) );
    double target_theta = goal_theta > theta_degree ? (goal_theta-theta_degree) : (theta_degree - goal_theta);
    return (180.0f - target_theta);
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

