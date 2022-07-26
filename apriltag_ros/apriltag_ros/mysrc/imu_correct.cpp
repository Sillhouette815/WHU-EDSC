#include "DeviceHandler/DeviceHandler.hpp"
#include "ros/ros.h"
using namespace std;
using namespace Eigen;
using namespace DeviceHandler;
Eigen::Isometry3d imu_pose_init;
Eigen::Isometry3d imu_pose;
Eigen::Isometry3d imu_pose_add=Eigen::Isometry3d::Identity();
int init_cout = 100;
ros::Subscriber imu_sub;
ros::Publisher imu_pub;
ros::Time inittime;
ros::Time lasttime;
ros::Duration delta_time;
Eigen::Vector3d add_rotate=Eigen::Vector3d::Zero();
void CallBack_IMU(const sensor_msgs::Imu::ConstPtr &msg)
{

    geometry_msgs::Quaternion ore = msg->orientation;
    imu_pose = Eigen::Isometry3d::Identity();
    imu_pose.prerotate(Quaternion<double>(ore.w, ore.x, ore.y, ore.z));
    if(init_cout>0)
    {
        init_cout--;
        imu_pose_init = Eigen::Isometry3d::Identity();
        imu_pose_init.prerotate(Quaternion<double>(ore.w, ore.x, ore.y, ore.z));
        if(init_cout==0)
        {
            cout << "初始化完毕" << endl;
            lasttime = ros::Time::now();
            inittime = lasttime;
        }
    }
    else
    {
        ros::Time nowtime = ros::Time::now();
        ros::Duration now_delta_time = nowtime-lasttime;
        delta_time = nowtime - inittime;
        Eigen::Vector3d nowrotate(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        nowrotate = nowrotate*(now_delta_time.toSec());
        add_rotate += nowrotate;
        lasttime = nowtime;
    }
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"imu_correct");
    ros::NodeHandle ROSnh;
    imu_sub = ROSnh.subscribe<sensor_msgs::Imu>("imu", 5,CallBack_IMU);
    imu_pub = ROSnh.advertise<sensor_msgs::Imu>("imu", 5);
    Timer t;
    t.start();
    ros::Rate tick(200);
    while (ros::ok()&&t.nowgap()<60000000)
    {
        tick.sleep();
        ros::spinOnce();
    }
    cout << "add_rotate: "<<add_rotate << endl;
    cout << "delta_time: "<<delta_time << endl;
    Eigen::Quaterniond delta_rotate (imu_pose_init.rotation() * imu_pose.rotation().inverse());
    cout << "delta_rotate: "<< delta_rotate.x()<<","<< delta_rotate.y()<<","<<delta_rotate.z()<<","<<delta_rotate.w()<< endl;
    return 0;
}

