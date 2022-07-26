#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include "actionlib_msgs/GoalID.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include <stdlib.h>

#include <unistd.h>
#include <pthread.h>
#include<time.h>
#include<signal.h>
using namespace std;
class Timer
{
    public:
    long start_time;
    Timer()
    {
        start();
    }
    void start()
    {
        struct timezone tz;
        timeval tv;
        gettimeofday(&tv,&tz);
        start_time=tv.tv_sec*1000000+tv.tv_usec;
    }
    long nowgap()
    {
        struct timezone tz;
        timeval tv;
        gettimeofday(&tv,&tz);
        long now_time=tv.tv_sec*1000000+tv.tv_usec;
        return now_time-start_time;
    }
};
vector<int> subProcess;
ros::Subscriber systemServer ;
void signalCB(int sig)
{
    ROS_INFO("fake_system 节点退出！");
    ros::shutdown();
    for(int i=0;i<subProcess.size();i++)
    {
        int status=kill(subProcess[i],9);
    }
}
void fakeSystem(const std_msgs::String::ConstPtr& msg)
{
    int pid=fork();
    if(pid>0)
    {
        cout<<"收到指令："<<msg->data<<endl;
        subProcess.push_back(pid);
    }
    else if(pid==0)
    {
        cout<<"指令执行："<<msg->data<<endl;
        int status=system(msg->data.c_str());
        cout<<"指令执行完毕"<<endl;
        pthread_exit(NULL);
    }
    else
    {
        cout<<"指令执行失败！"<<endl;
    }
}
int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"fake_system");
    ros::NodeHandle nh;
    systemServer=nh.subscribe<std_msgs::String>("system_cmd",10,fakeSystem);
    subProcess=vector<int>();
    signal(SIGINT,signalCB);
    ros::Rate rate(200);
    while(ros::ok())
    {
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}