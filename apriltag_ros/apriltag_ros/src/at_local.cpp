#include <ros/ros.h>
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "std_msgs/String.h"
#include <sstream>
#include "geometry_msgs/Twist.h"
#include <Eigen/Dense>
#include <cmath>
#include <stdlib.h>
#include <sensor_msgs/Imu.h>
#include <time.h>

/*
X：二维码相对水平距离
Y：二维码相对垂直距离
Z：二维码相对深度距离

ground_num:平地查询次数
is_upstair:是否处于上楼状态
pitch_imu：IMU的pitch角
up_down:上楼或下楼状态
onlyparallel:是否只需要平行
pitch_limit:pitch阈值
x_limit:水平距离阈值
is_parallel:是否处于平行状态
is_turn:是否处于转向状态
is_correct:是否处于二维码对正状态
z_dir:修正的前进方向
is_first_ground:一层平台
is_all_done:全部上楼/下楼工作完成
z_ang:z轴角速度
x_vel:x轴线速度
is_done:是否完成二维码对准
*/

float X;
float Y;
float Z;

int ground_num=0;
int ground_num_done=0;
bool is_upstair=false;
double pitch_imu,roll_imu,yaw_imu;
bool up_down=true;
bool onlyparallel=false;
const double pitch_limit=5;
const double x_limit=0.18;
bool is_parallel=false;
bool is_turn=false;
bool is_correct=false;
bool z_dir=false;
bool is_first_ground=false;
bool is_all_done=false;
bool continue_stair=false;
float z_ang,x_vel;
//double pitch;
bool is_done=false;
class Timer
{
    long start_time=0;
    public:
    void start()
    {
        struct timezone tz;
        timeval tv;
        gettimeofday(&tv,&tz);
        start_time=tv.tv_sec*1000+tv.tv_usec/1000;
    }
    long gap()
    {
        struct timezone tz;
        timeval tv;
        gettimeofday(&tv,&tz);
        long now_time=tv.tv_sec*1000+tv.tv_usec/1000;
        return now_time-start_time;
    }
};
using namespace Eigen;
ros::Publisher pub;
double xxx(double indata,double x)
{
    double a=indata/x;
    double aa=floor(a);
    return (a-aa)*x;
}


void CallBack_IMU(const sensor_msgs::Imu::ConstPtr &msg)
{
ros::Rate rate_imu(100);
if(is_done){
    geometry_msgs::Twist cmd;
    cmd.linear.x=0.15;
    cmd.angular.z=0.02;
    geometry_msgs::Quaternion ore=msg->orientation;
    Eigen::Quaternion<double> qua(ore.w,ore.x,ore.y,ore.z);
    Vector3d vec;
    vec=qua.toRotationMatrix().eulerAngles(0,1,2);
    pitch_imu=xxx(vec[0],M_PI)*180/M_PI;
    ROS_INFO("PITCH=%lf",pitch_imu);
    if((pitch_imu>10&&pitch_imu<90)||(pitch_imu>90&&pitch_imu<170))
    {
        if(is_first_ground)
        {
            continue_stair=true;
            ROS_INFO("Keep Going upstairs!");
        }//若已到达楼梯平台，则设置继续上楼为true
        else
        {
            is_upstair=true;
            ROS_INFO("Going upstairs!");
        }        
    }
    if(is_upstair&&(pitch_imu<1||abs(pitch_imu-180)<3))
    {
        if(is_first_ground&&continue_stair)
        {
            ROS_INFO("Keep Grounded！");
            ground_num_done++;
        }//已经过平台且进行二次上楼，
        if(!is_first_ground)
        {
            ROS_INFO("Grounded!");
            ground_num++;
        }//未经过平台，一次上楼时
    }
    if(ground_num>=100||ground_num_done>=100)
    {
        if(!is_first_ground)
        {
        cmd.linear.x=0.0;
        ground_num=0;
        Timer timer;
        timer.start();
        while(timer.gap()<1000)
        {
            pub.publish(cmd);
        }//保证车辆停止
        is_first_ground=true;//到达第一平层
        is_done=false;//进行再次二维码校正
        is_turn=false;//设置车辆为未转向状态，保证继续二维码识别对准
        onlyparallel=true;
        }//当到达第一次平层时候，停车，再次二维码校正，并设置仅平行对正
        else
        {
            cmd.linear.x=0.0;
            is_all_done=true;
            onlyparallel=false;
            ROS_INFO("The Upstair mission is completed!");
        }//否则停止车辆,完成上楼任务
    }//当抵达平台或者完成楼梯时，进行判断
    pub.publish(cmd);
    }
    else
    {
    rate_imu.sleep();
    ROS_INFO("IMU");
    }
}
double z=0.0;
double z_error=0.01;
void CallBack(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    geometry_msgs::Twist Geo_msg;
    if(!is_turn)//仅当未执行转弯操作时，才进行二维码对准。
    {
    if(!msg->detections.empty())//若检测到二维码，则进行输出并更新信息
    {
        X=msg->detections.back().pose.pose.pose.position.x;
        Y=msg->detections.back().pose.pose.pose.position.y;
        Z=msg->detections.back().pose.pose.pose.position.z;
        z=msg->detections.back().pose.pose.pose.orientation.z;
        ROS_INFO("z=%lf",z);
        //ROS_INFO("The data is x=%lf,y=%lf",X,Y);
        // geometry_msgs::Quaternion ore=msg->detections.back().pose.pose.pose.orientation;
        // Eigen::Quaternion<double> qua(ore.w,ore.x,ore.y,ore.z);
        // Eigen::Vector3d vec;
        // vec=qua.toRotationMatrix().eulerAngles(0,1,2);

        // pitch=xxx(vec[1],M_PI)*180/M_PI;
        // ROS_INFO("Pitch =%lf",pitch);
        // if(pitch>90&&pitch<180)
        // {
        //     if(abs(180-pitch)<pitch_limit)//姿态对准
        //     {
        //         z_ang=0.0;  
        //         is_parallel=true;
        //         ROS_INFO("The vehicle is parallel to the aprilcode!");

        //     }
        //     else
        //     {
        //         z_ang=-0.13;
        //     }
        // }
        // else
        // {
        //     if(pitch<pitch_limit)//姿态对准
        //     {
        //         z_ang=0.0;
        //         is_parallel=true;
        //         ROS_INFO("The vehicle is parallel to the aprilcode!");
        //     }
        //     else
        //     {
        //         z_ang=0.13;
        //     }
        // }
        if(abs(z)<z_error)
        {
            z_ang=0.0;
            is_parallel=true;
            if(onlyparallel)
            {
                is_correct=true;//二维码对准成功
            }
            ROS_INFO("The vehicle is parallel to the aprilcode!");
        }
        else
        {
            if(z>0)
            {
                z_ang=-0.15;
            }//左转
            else
            {
                z_ang=0.15;
            }//右转
        }
        if(is_parallel&&!onlyparallel)//若姿态对准，且非onlyparallel时，确定二维码的位置方向
        {
            if(up_down)
            {
                if(abs(X)<x_limit)//若位置对准，则位姿均对准
                {
                    is_correct=true;
                    x_vel=z_ang=0.0;
                    ROS_INFO("The position and orientation is correct!");
                }
                else//否则告知对准方向
                {
                    is_correct=false;
                    if(X>0)
                        z_dir=true;
                    else
                        z_dir=false;
                }
            }//上楼对准
            else
            {
                if(X<0&&abs(abs(X)-0.6)<x_limit)
                {
                    is_correct=true;
                    x_vel=z_ang=0.0;
                    ROS_INFO("The position and orientation is correct!");
                }
                else
                {
                    is_correct=false;
                    if(X+0.6<0)
                    {
                        z_dir=false;
                    }
                    else
                    {
                        z_dir=true;
                    }
                }
            }//下楼对准
        }
        
    }
    else
    {
        z_ang=0.32;
        ROS_INFO("No APrilTag is detected!");
    }
    Geo_msg.angular.z=z_ang;
    Geo_msg.linear.x=x_vel;
    if(is_parallel)
    {
        Timer timer;
        timer.start();
        while(timer.gap()<1000)
        {
            pub.publish(Geo_msg);
        }
    }
    else
    {
        pub.publish(Geo_msg);
    }
    }
}

void Turn_Left()
{
    ros::Rate rate(100);
    x_vel=0.0;
    z_ang=0.32;
    geometry_msgs::Twist geo;
    geo.angular.z=z_ang;
    geo.linear.x=x_vel;
    Timer timer;
    timer.start();
    while(timer.gap()<6550)
    {
        rate.sleep();
        pub.publish(geo);
    }//左转弯
    z_ang=x_vel=0.0;
    timer.start();
    geo.angular.z=z_ang;
    geo.linear.x=x_vel;
    while(timer.gap()<500)
    {
        rate.sleep();
        pub.publish(geo);
    }//休息
    z_ang=0.0;
    x_vel=0.15;
    int nums;
    if(up_down)
    {
        nums=round(abs(X)/x_vel);
    }
    else
    {
        nums=round(abs(abs(X)-0.6)/x_vel);
    }
    geo.angular.z=z_ang;
    geo.linear.x=x_vel;
    timer.start();
    while(timer.gap()<nums*800)
    {
        rate.sleep();
        pub.publish(geo);
    }//直行
    z_ang=x_vel=0.0;
    geo.angular.z=z_ang;
    geo.linear.x=x_vel;
    timer.start();
    while(timer.gap()<500)
    {
        rate.sleep();
        pub.publish(geo);
    }//休息
    z_ang=-0.32;
    x_vel=0.0;
    geo.angular.z=z_ang;
    geo.linear.x=x_vel;
    timer.start();
    while(timer.gap()<6550)
    {
        rate.sleep();
        pub.publish(geo);
    }//转回来
    z_ang=x_vel=0.0;
    geo.angular.z=z_ang;
    geo.linear.x=x_vel;
    timer.start();
    while(timer.gap()<1000)
    {
        rate.sleep();
        pub.publish(geo);
    }//休息
    ROS_INFO("Turn Left is completed!");
    is_turn=false;//左转弯操作执行完毕
    is_parallel=false;//此时不可保证姿态对准，因此设置为false
}
void Turn_Right()
{
    ros::Rate rate(100);
    x_vel=0.0;
    z_ang=-0.32;
    geometry_msgs::Twist geo;
    geo.angular.z=z_ang;
    geo.linear.x=x_vel;
    Timer timer;
    timer.start();
    while(timer.gap()<6350)
    {
        rate.sleep();
        pub.publish(geo);
    }//转弯
    z_ang=x_vel=0.0;
    geo.angular.z=z_ang;
    geo.linear.x=x_vel;
    timer.start();
    while(timer.gap()<500)
    {
        rate.sleep();
        pub.publish(geo);
    }//休息
    z_ang=0.0;
    x_vel=0.15;
    int nums;
    if(up_down)
    {
        nums=round(abs(X)/x_vel);
    }
    else
    {
        nums=round(abs(abs(X)-0.6)/x_vel);
    }
    geo.angular.z=z_ang;
    geo.linear.x=x_vel;
    timer.start();
    while(timer.gap()<nums*800)
    {
        rate.sleep();
        pub.publish(geo);
    }//直行
    z_ang=x_vel=0.0;
    geo.angular.z=z_ang;
    geo.linear.x=x_vel;
    timer.start();
    while(timer.gap()<500)
    {
        rate.sleep();
        pub.publish(geo);
    }//休息
    x_vel=0.0;
    z_ang=0.32;
    geo.angular.z=z_ang;
    geo.linear.x=x_vel;
    geo.angular.z=z_ang;
    geo.linear.x=x_vel;
    timer.start();
    while(timer.gap()<6350)
    {
        rate.sleep();
        pub.publish(geo);
    }//转回来
    z_ang=x_vel=0.0;
    geo.angular.z=z_ang;
    geo.linear.x=x_vel;
    timer.start();
    while(timer.gap()<1000)
    {
        rate.sleep();
        pub.publish(geo);
    }//休息
    ROS_INFO("Turn Right is completed!");
    is_turn=false;//右转弯操作执行完毕
    is_parallel=false;//此时不可保证姿态对准，因此设置为false
}
int main(int argc, char **argv)
{

    ros::init(argc,argv,"at_local");
    geometry_msgs::Twist cmd;
    cmd.linear.x=0.1;
    ros::NodeHandle nh;
    pub=nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
    ros::Subscriber sub=nh.subscribe<apriltag_ros::AprilTagDetectionArray>("tag_detections",1,CallBack);
    ros::Subscriber sub_imu=nh.subscribe<sensor_msgs::Imu>("imu",1,CallBack_IMU);
    while(ros::ok())
    {
    //ROS_INFO("is_correct=%d,is_parallel=%d,is_turn=%d",is_correct,is_parallel,is_correct);
        if(is_parallel==true&&is_correct==false&&!onlyparallel)//当车辆与二维码平行时，且非onlyparallel，开启位置对准
        {
            is_turn=true;   
            if(z_dir==false)
                Turn_Right();
            else
                Turn_Left();
        }
        if(is_correct)
        {
            //int code=system("rosnode kill /apriltag_ros_continuous_node");
            is_correct=false;
            is_parallel=false;
            is_turn=true;//禁止继续二维码对正任务
            is_done=true;//完成对正
        }
        if(is_all_done)
        {
            int code=system("rosnode kill /apriltag_ros_continuous_node");
            is_done=false;//关闭上楼/下楼任务
            is_turn=true;//关闭二维码对准任务
        }
            ros::spinOnce();
    }
    return 0;
}
