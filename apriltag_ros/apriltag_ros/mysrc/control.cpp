#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include "actionlib_msgs/GoalID.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "move_base_msgs/MoveBaseActionResult.h"

#include <stdlib.h>
#include <string>
#include <sstream>

#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <signal.h>
#include "DeviceHandler/DeviceHandler.hpp"
#include "DeviceHandler/LinkMap.hpp"
using namespace std;
using namespace DeviceHandler;

ros::Publisher cmd_pub;
ros::Publisher goal_pub;
ros::Publisher voice_pub;

ros::Subscriber odom_sub;
ros::Subscriber imu_sub;
ros::Subscriber movebase_sub;
ros::Subscriber cmd_sub;
ros::Subscriber ice_sub;


Eigen::Isometry3d odom_pose;
Eigen::Isometry3d imu_pose;
Eigen::Isometry3d imu_base_pose;

LinkMap linkmap;
Vehicle car;
LinkMapNode *baseNode = NULL;
LinkMapNode *NextNode = NULL;

string iceStr;
string cmdStr;
atomic<bool> goalsended;
atomic<bool> reachgoal;
atomic<bool> isworking ;//waitForReach
void mbrCB(const move_base_msgs::MoveBaseActionResult::ConstPtr &msg)
{
    if (msg->status.status == 3 && goalsended.load())
    {
        goalsended.store(false);
        reachgoal.store(true);
        std::cout << "到位置了！" << std::endl;
    }
    else
    {
        reachgoal.store(false);
    }
}
void odomCB(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    double pos_x = odom_msg->pose.pose.position.x;
    double pos_y = odom_msg->pose.pose.position.y;
    double pos_z = odom_msg->pose.pose.position.z;
    double qua_w = odom_msg->pose.pose.orientation.w;
    double qua_x = odom_msg->pose.pose.orientation.x;
    double qua_y = odom_msg->pose.pose.orientation.y;
    double qua_z = odom_msg->pose.pose.orientation.z;
    odom_pose = Eigen::Isometry3d::Identity();
    odom_pose.translate(Eigen::Vector3d(pos_x, pos_y, pos_z));
    odom_pose.rotate(Eigen::Quaternion<double>(qua_w, qua_x, qua_y, qua_z));
}
void imuCB(const sensor_msgs::Imu::ConstPtr &msg)
{
    // cout << "受到imu" << endl;
    geometry_msgs::Quaternion ore = msg->orientation;
    imu_pose = Eigen::Isometry3d::Identity();
    imu_pose.prerotate(Quaternion<double>(ore.w, ore.x, ore.y, ore.z));
    // imu_pose.push(imu_pose);
    // if(imuQueue.size()>queue_limit)
    // {
    //     imuQueue.pop();
    // }
}

void icecreamCB(const std_msgs::String::ConstPtr&msg)
{
    iceStr = msg->data;
}
void commondCB(const std_msgs::String::ConstPtr&msg)
{
    cmdStr=msg->data;
}
void pub_goal(ros::Publisher &pub, const vector<double> &data)
{
    ros::Rate rate(5);
    rate.sleep();
    if (data.size() < 7)
    {
        cout << "pub_goal: 输入不正确! " << endl;
    }
    geometry_msgs::PoseStamped goal_msg;
    goal_msg.header.frame_id = "world";
    // goal_msg.header.seq
    ros::Time nowTime = ros::Time::now();
    goal_msg.header.stamp.sec = nowTime.sec;
    goal_msg.header.stamp.nsec = nowTime.nsec;
    goal_msg.pose.position.x = data[0];
    goal_msg.pose.position.y = data[1];
    goal_msg.pose.position.z = data[2];
    goal_msg.pose.orientation.x = data[3];
    goal_msg.pose.orientation.y = data[4];
    goal_msg.pose.orientation.z = data[5];
    goal_msg.pose.orientation.w = data[6];
    pub.publish(goal_msg);
    goalsended = true;
}
void waitForReach(ros::Rate rate = ros::Rate(200))
{
    reachgoal.store(false);
    while (ros::ok() && !reachgoal.load()&&isworking.load())
    {
        rate.sleep();
        ros::spinOnce();
    }
    reachgoal.store(false);
}
void fakesystem(string cmd)
{
    ros::Rate rate(200);
    rate.sleep();
    std_msgs::String cmd_msg;
    cmd_msg.data = cmd;
    cmd_pub.publish(cmd_msg);
}
void speak(string sentence)
{
    ros::Rate rate(200);
    rate.sleep();
    std_msgs::String sentence_msg;
    sentence_msg.data = sentence;
    voice_pub.publish(sentence_msg);
}
void initImuPose(long times)
{
    Timer t;
    t.start();
    ros::Rate rate(100);
    while (t.nowgap() < times)
    {
        rate.sleep();
        ros::spinOnce();
        imu_base_pose = imu_pose;
        Eigen::Quaterniond qua(imu_base_pose.rotation());
        cout << "初始化: " << qua.x() << "," << qua.y() << "," << qua.z() << "," << qua.w() << endl;
    }
}
bool navToGoal(string name)
{
    vector<LinkMapNode *> pregoalNode = linkmap.findByName(name);
    LinkMapNode *goalNode;
    if (pregoalNode.empty())
    {
        return false;
    }
    else
    {
        goalNode = pregoalNode[0];
    }
    vector<LinkMapNode *> path_1, path_2;
    vector<int> method_1, method_2;
    double cost_1 = 0, cost_2 = 0;
    bool flag_1 = false, flag_2 = false;
    vector<LinkMapNode *> ref_path;
    vector<int> ref_method;
    if (baseNode != NULL)
    {
        flag_1 = linkmap.searchPath(baseNode, goalNode, cost_1, path_1, method_1);
    }
    if (NextNode != NULL)
    {
        flag_2 = linkmap.searchPath(NextNode, goalNode, cost_2, path_2, method_2);
    }
    if (flag_1 && flag_2)
    {
        if (cost_1 < cost_2)
        {
            ref_path = path_1;
            ref_method = method_1;
        }
        else
        {
            ref_path = path_2;
            ref_method = method_2;
        }
    }
    else if (flag_1)
    {
        ref_path = path_1;
        ref_method = method_1;
    }
    else if (flag_2)
    {
        ref_path = path_2;
        ref_method = method_2;
    }
    else
    {
        return false;
    }
    for (int i = 0; i < ref_path.size(); i++)
    {
        cout << "路径节点： " << (ref_path)[i]->name << endl;
    }
    LinkMapNode *lastNode = NULL;
    LinkMapNode *nowNode = NULL;
    for (int i = 0; i < ref_method.size(); i++)
    {

        if (lastNode != NULL)
        {
            if (lastNode->floorNum != nowNode->floorNum)
            {
                if (nowNode->floorNum == 0)
                {
                    fakesystem("rosnode kill map_server move_base");
                    sleep(1);
                    fakesystem("roslaunch cartographer_ros nav_second_floor.launch");
                    sleep(1);
                }
                else
                {
                    fakesystem("rosnode kill map_server move_base");
                    sleep(1);
                    fakesystem("roslaunch cartographer_ros nav_first_floor.launch");
                    sleep(1);
                }
            }
        }
        if(!isworking.load())
        {
            return false;
        }
        else
        {
            nowNode = ref_path[i];
            NextNode = nowNode;
        }
        Eigen::Isometry3d goal_pose = Eigen::Isometry3d::Identity();
        switch ((ref_method)[i])
        {
        case 0:
        {
            pub_goal(goal_pub, {0, 0, 0.0, 0.0, 0.0, 0, 1});
            waitForReach();
            break;
        }
        case 1:
        {
            car.updownstairs(imu_base_pose, goal_pose, true);
            break;
        }
        case 2:
        {
            car.updownstairs(imu_base_pose, goal_pose, false);
            break;
        }
        default:
            break;
        }
        lastNode = nowNode;
        if(!isworking.load())
        {
        }
        else
        {
            baseNode = nowNode;
        }
    }
    NextNode = NULL;
    return true;
}
void sighandler(int num)
{
    ros::shutdown();
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "control_core");
    ros::NodeHandle ROSnh;
    // rosparam
    // rospub
    cmd_pub = ROSnh.advertise<std_msgs::String>("system_cmd", 10);
    goal_pub = ROSnh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
    voice_pub = ROSnh.advertise<std_msgs::String>("speak",10);
    // rossub
    movebase_sub = ROSnh.subscribe<move_base_msgs::MoveBaseActionResult>("move_base/result", 10, mbrCB);
    imu_sub = ROSnh.subscribe<sensor_msgs::Imu>("imu", 10, imuCB);
    odom_sub = ROSnh.subscribe<nav_msgs::Odometry>("odometry", 10, odomCB);
    ice_sub = ROSnh.subscribe<std_msgs::String>("iceice",10,icecreamCB);
    cmd_sub = ROSnh.subscribe<std_msgs::String>("commond",10,commondCB);
    //
    signal(SIGINT,sighandler);
    isworking.store(true);
    goalsended.store(false);
    reachgoal.store(false);
    // initImuPose(1000000);
    // linkmap.read("/home/fufu/edsc/carto_ws/src/slam_tools/data/nodemap.txt", "/home/fufu/edsc/carto_ws/src/slam_tools/data/linkmap.txt");
    // baseNode = &(linkmap.mapNodeVec[0]);
    car.init(ROSnh);
    Eigen::Isometry3d goal_pose = Eigen::Isometry3d::Identity();

    // //第一个 一楼楼梯口
    // pub_goal(goal_pub, {11.6809, 7.6821, 0.0, 0.0, 0.0, -0.4699, 0.8826});
    // waitForReach();
    // //上楼
    // car.updownstairs(imu_base_pose, goal_pose, true);
    // car.updownstairs(imu_base_pose, goal_pose, true);
    // //切二楼
    // fakesystem("rosnode kill map_server move_base");
    // sleep(1);
    // fakesystem("roslaunch cartographer_ros nav_second_floor.launch");
    // sleep(3);
    // //二楼房间前面
    // pub_goal(goal_pub, {34.1265, -4.2627, 0.0, 0.0, 0.0, -0.4848, 0.8746}); //需要查看
    // sleep(2);
    // waitForReach();
    // sleep(3);
    // 二楼楼梯口
    // pub_goal(goal_pub, {9.2268,-7.9720, 0.0, 0.0, 0.0, 0.8857, 0.4642}); //需要查看
    // sleep(2);

    // waitForReach();
    // sleep(1);
    Eigen::Quaterniond qua(0.272,0.0,0.0,-0.96229);    
    Eigen::Isometry3d goal_odom=Eigen::Isometry3d::Identity();
    goal_odom.prerotate(qua);
    //下楼
    // car.updownstairs(imu_base_pose, goal_pose, false);
    // sleep(1);
    // car.updownstairs(imu_base_pose, goal_pose, false);
    car.updownstairs(goal_odom,true);
    sleep(2);
    // car.updownstairs(false,true);

    // //切一楼
    // fakesystem("rosnode kill map_server move_base");
    // sleep(1);
    // fakesystem("roslaunch cartographer_ros nav_first_floor.launch");
    // sleep(3);
    // //一楼楼梯口
    // pub_goal(goal_pub, {11.6809, 7.6821, 0.0, 0.0, 0.0, -0.4699, 0.8826});
    // waitForReach();
    //全部重启
    //暂停
    //停止当前任务
    //导航到某个点

    
    // pthread_exit(NULL);
    return 0;
}
