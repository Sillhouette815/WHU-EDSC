#ifndef _MY_STAIRSHANDLER_HPP_
#define _MY_STAIRSHANDLER_HPP_
#include <sstream>
#include <Eigen/Dense>
#include <cmath>
#include <stdlib.h>
#include <sensor_msgs/Imu.h>
#include <time.h>
#include <queue>
#include "ros/ros.h"

#include "apriltag_ros/AprilTagDetectionArray.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
 #include "tf/transform_listener.h"
#include "tf/tf.h"
using namespace std;
using namespace Eigen;
namespace DeviceHandler
{
    class PID
    {
        public:
            double k=1;
            double kp = 1;
            double ki=0.01;
            double kd=1;
            double p_error=0;
            double i_error=0;
            double d_error=0;
            double result=0;
            PID(double k_p, double k_i, double k_d, double k_k)
            {
                k = k_k;
                kp = k_p;
                ki = k_i;
                kd = k_d;
        }
        PID(double k_p,double k_i,double k_d):PID(k_p,k_i,k_d,1)
        {
        }
        PID():PID(1,0.01,1)
        {
        }
        void setError(double error,double delta_t)
        {
            d_error = (error - p_error) / delta_t;
            i_error+=delta_t*error;
            p_error = error;
            result = k * (kp * p_error + ki * i_error + kd * d_error);
        }
        void setError(double error)
        {
            setError(error, 1.0);
        }
        void reset()
        {
            p_error = 0;
            i_error = 0;
            d_error = 0;
            result = 0;
        }
        double getResult()
        {
            return result;
        }
        void printStr(string header)
        {
            cout<<header<<endl;
            cout << "error: p,i,d: " <<p_error<<","<<i_error<<","<<d_error<< endl;
            cout << "effort: p,i,d: " <<kp*p_error<< ","<<ki*i_error<<","<<kd*d_error<< endl;
            cout<<"result"<<result<<endl;
        }
    };
    class Timer
    {
        long start_time = 0;

    public:
        void start()
        {
            struct timezone tz;
            timeval tv;
            gettimeofday(&tv, &tz);
            start_time = tv.tv_sec * 1000000 + tv.tv_usec;
        }
        long nowgap()
        {
            struct timezone tz;
            timeval tv;
            gettimeofday(&tv, &tz);
            long now_time = tv.tv_sec * 1000000 + tv.tv_usec;
            return now_time - start_time;
        }
    };
    class Vehicle
    {
    public:
        double linear_limit = 0.25;
        double angular_limit = 0.32;
        double times_limit = 30000000;
        double delta_quaz_limit = 0.01;
        double delta_posx_limit = 0.18;
        double delta_roll_limit = 10;

        ros::Publisher cmdvel_pub;
        ros::Publisher status_pub;
        ros::Subscriber apriltag_sub;
        ros::Subscriber imu_sub;
        ros::Subscriber odom_sub;

        int imu_ok = 0;
        int odom_ok = 0;
        int apriltag_ok = 0;
        Eigen::Isometry3d apriltag_pose = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d imu_pose = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d odom_pose = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d map_to_world = Eigen::Isometry3d::Identity();


        size_t queue_limit = 50;
        queue<Eigen::Isometry3d> imuQueue;
        queue<Eigen::Isometry3d> odomQueue;
        bool inited = false;

    public:
        enum status
        {
            unkown_status,
            onground,
            onstairs
        };
        enum action
        {
            unkown_action,
            stop,
            upstairs,
            downstairs,
            moving
        };

    private:

        void CallBack_IMU(const sensor_msgs::Imu::ConstPtr &msg)
        {
            // ROS_INFO("收到IMU消息");
            if (imu_ok < 100000)
            {
                imu_ok++;
            }
            geometry_msgs::Quaternion ore = msg->orientation;
            imu_pose = Eigen::Isometry3d::Identity();
            imu_pose.prerotate(Quaternion<double>(ore.w, ore.x, ore.y, ore.z));
            imuQueue.push(imu_pose);
            if (imuQueue.size() > queue_limit)
            {
                imuQueue.pop();
            }
        }
        void CallBack_Apriltag(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
        {

            if (!msg->detections.empty()) //若检测到二维码，则进行输出并更新信息
            {
                // ROS_INFO("收到Apriltag消息");

                if (apriltag_ok < 100000)
                {
                    apriltag_ok++;
                }
                double pos_x = msg->detections.back().pose.pose.pose.position.x;
                double pos_y = msg->detections.back().pose.pose.pose.position.y;
                double pos_z = msg->detections.back().pose.pose.pose.position.z;
                double qua_w = msg->detections.back().pose.pose.pose.orientation.w;
                double qua_x = msg->detections.back().pose.pose.pose.orientation.x;
                double qua_y = msg->detections.back().pose.pose.pose.orientation.y;
                double qua_z = msg->detections.back().pose.pose.pose.orientation.z;
                apriltag_pose = Eigen::Isometry3d::Identity();
                apriltag_pose.pretranslate(Eigen::Vector3d(pos_x, pos_y, pos_z));
                apriltag_pose.prerotate(Eigen::Quaternion<double>(qua_w, qua_x, qua_y, qua_z));
            }
            else
            {
                apriltag_ok = 0;
            }
        }
        void CallBack_Odometry(const nav_msgs::Odometry::ConstPtr &odom_msg)
        {
            if (odom_ok < 10000)
            {
                odom_ok++;
            }
            geometry_msgs::PoseStamped new_pose;
            new_pose.pose=odom_msg->pose.pose;
            double pos_x =new_pose.pose.position.x;
            double pos_y = new_pose.pose.position.y;
            double pos_z = new_pose.pose.position.z;
            double qua_w = new_pose.pose.orientation.w;
            double qua_x = new_pose.pose.orientation.x;
            double qua_y = new_pose.pose.orientation.y;
            double qua_z = new_pose.pose.orientation.z;
            // ROS_INFO("收到Odometry消息");


            odom_pose = Eigen::Isometry3d::Identity();
            // odom_pose.pretranslate(Eigen::Vector3d(pos_x, pos_y, pos_z));
            odom_pose.prerotate(Eigen::Quaternion<double>(qua_w, qua_x, qua_y, qua_z));
            odom_pose=map_to_world*odom_pose;
            // double px=odom_pose.translation()[0];
            // double py=odom_pose.translation()[1];
            odom_pose.pretranslate(Eigen::Vector3d(-pos_y, -pos_x, pos_z));
            Eigen::Quaterniond odom_qua(odom_pose.rotation());
            cout<<"Odometry: qua x,y,z,w: "<< odom_qua.x()<<","<<odom_qua.y()<<","<<odom_qua.z()<<","<<odom_qua.w() <<endl;
            cout<<"Odometry: x,y,z: "<< odom_pose.translation()[0]<<","<<odom_pose.translation()[1]<<","<<odom_pose.translation()[2] <<endl;
            odomQueue.push(odom_pose);
            if (odomQueue.size() > queue_limit)
            {
                odomQueue.pop();
            }
        }

        double getDeltaAngle2D(const Eigen::Vector2d &base_point, const Eigen::Vector2d &goal_point)
        {
            Eigen::Vector2d nowpoint(odom_pose.translation()[0], odom_pose.translation()[1]);
            double cosValNew = (nowpoint - base_point).normalized().dot((goal_point - nowpoint).normalized()); //角度cos值
            double angle_2D = acos(cosValNew) * 180 / M_PI;                                                    //弧度角
            double delta_angle = angle_2D;
            if (angle_2D >= 0 && angle_2D < 90) //左转
            {
                delta_angle -= 90;
            }
            else if (angle_2D >= 90 && angle_2D <= 180) //右转
            {
                delta_angle -= 90;
            }
            else if (angle_2D < 0 && angle_2D >= -90) //左转
            {
                delta_angle -= 90;
            }
            else //右转
            {
                delta_angle = (delta_angle + 270);
            }
            if (delta_angle < -180)
            {
                delta_angle += 360;
            }
            cout << "角度差值: " << delta_angle << endl;
            return delta_angle;
        }
        
        double getDeltaAngle2D(const Eigen::Isometry3d &base_pose, const Eigen::Isometry3d &goal_pose, bool by_imu)
        {
            Eigen::Isometry3d delta_pose = Eigen::Isometry3d::Identity();
            Eigen::Quaterniond dqua;
            if (by_imu)
            {
                delta_pose.prerotate(imu_pose.rotation());
                delta_pose.prerotate(base_pose.rotation().inverse());
                delta_pose.prerotate(goal_pose.rotation().inverse());
                dqua= Eigen::Quaterniond(delta_pose.rotation());
                cout<<"imu: dquaz: "<<dqua.z()<<endl;

                            return dqua.z() ;

            }
            else
            {
                Eigen::Vector2d base_point(base_pose.translation()[0],base_pose.translation()[1]);
                Eigen::Vector2d goal_point(goal_pose.translation()[0],goal_pose.translation()[1]);
                Eigen::Vector2d now_point(odom_pose.translation()[0],odom_pose.translation()[1]);
                Eigen::Vector2d baseVec(base_point[0]-goal_point[0],base_point[1]-goal_point[1]);
                Eigen::Vector2d nowVec(now_point[0]-goal_point[0],now_point[1]-goal_point[1]);
                double px=baseVec.dot(nowVec)/(baseVec.norm());
                double py=sqrt(pow(nowVec.norm(),2)-pow(px,2));
                double angle=atan2(py,px)*180/M_PI;
                cout<<"odom: angle: "<<angle<<endl;
                return angle;
            } 
            // dqua = imu_pose.rotation() * base_pose.rotation().inverse();
            // cout << "dqua" << dqua.z() << endl;
        }

        bool checkCar(status &sta, bool by_imu)
        {
            Eigen::Isometry3d ref_pose;
            if (by_imu)
            {
                ref_pose = imu_pose;
            }
            else
            {
                ref_pose = odom_pose;
            }
            Eigen::Vector3d imu_euglar = ref_pose.rotation().eulerAngles(0, 1, 2);
            imu_euglar = imu_euglar * 180 / M_PI;
            double delta_roll_1 = abs(abs(imu_euglar[0]) - 0);
            double delta_roll_2 = abs(abs(imu_euglar[0]) - 180);
            double delta_roll = delta_roll_1 < delta_roll_2 ? delta_roll_1 : delta_roll_2;
            if (delta_roll > delta_roll_limit)
            {
                sta = status::onstairs;
            }
            else
            {
                sta = status::onground;
            }
            return true;
        }

    public:
    Eigen::Isometry3d lastBase;
        void init(ros::NodeHandle &ROSnh)
        {
            cmdvel_pub = ROSnh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
            status_pub = ROSnh.advertise<std_msgs::Int32>("vehical_status", 5);
            apriltag_sub = ROSnh.subscribe<apriltag_ros::AprilTagDetectionArray>("tag_detections", 5, boost::bind(&Vehicle::CallBack_Apriltag, this, _1));
            imu_sub = ROSnh.subscribe<sensor_msgs::Imu>("imu", 5, boost::bind(&Vehicle::CallBack_IMU, this, _1));
            odom_sub = ROSnh.subscribe<nav_msgs::Odometry>("odometry", 5, boost::bind(&Vehicle::CallBack_Odometry, this, _1));
            ros::Time time ;
            ros::Rate rate(100);
            ros::Duration timeout(0.5);
            tf::TransformListener tfl;
            tf::StampedTransform stf;
            while(!tfl.waitForTransform("map", "world", time, timeout))
            {
                rate.sleep();
            }
            tfl.lookupTransform("map", "world", time, stf);
            double pos_x =stf.getOrigin().getX();
            double pos_y =stf.getOrigin().getY();
            double pos_z = stf.getOrigin().getZ();
            double qua_w = stf.getRotation().getW();
            double qua_x = stf.getRotation().getX();
            double qua_y = stf.getRotation().getY();
            double qua_z = stf.getRotation().getZ();
            cout<<"map_to_world: x,y,z,w: "<< qua_x<<","<<qua_y<<","<<qua_z<<","<<qua_w <<endl;
            map_to_world = Eigen::Isometry3d::Identity();
            map_to_world.pretranslate(Eigen::Vector3d(pos_x, pos_y, pos_z));
            map_to_world.prerotate(Eigen::Quaternion<double>(qua_w, qua_x, qua_y, qua_z));
            inited = true;
        }
        
        void move(double linear, double angular, long times)
        {
            geometry_msgs::Twist move_msg;
            double abs_linear = abs(linear);
            double abs_angular = abs(angular);
            if (abs_linear > linear_limit)
            {
                linear = linear_limit * (linear / abs_linear);           
            }
            if(abs_angular>angular_limit)
            {
                angular = angular_limit * (angular/abs_angular);
            }
            move_msg.linear.x = linear;
            move_msg.angular.z = angular;
            if (times <= 0)
            {
                ros::Rate hz(500);
                hz.sleep();
                cmdvel_pub.publish(move_msg);
            }
            else
            {
                ros::Rate hz(50);
                Timer t;
                t.start();
                while (t.nowgap() < times)
                {
                    hz.sleep();
                    cmdvel_pub.publish(move_msg);
                }
            }
        }

        bool updownstairs(Eigen::Isometry3d base_pose, Eigen::Isometry3d goal_pose, bool isup)
        {
            double linear = 0;
            double angular = 0;
            ros::Rate rate(100);
            PID pidImu(1,0.002,0,2);
            Vehicle::status laststatus = Vehicle::status::unkown_status;
            Vehicle::status nowstatus = Vehicle::status::unkown_status;
            bool pre_finish = false;
            int move_finish = 0;
            bool pre_move_finish=false;
            while (ros::ok())
            {
                rate.sleep();
                ros::spinOnce();
                double delta_angle = getDeltaAngle2D(base_pose, goal_pose, true);
                pidImu.setError(delta_angle);
                pidImu.printStr("-----imu校准-----");
                checkCar(nowstatus, true);
                if(laststatus == status::onstairs && nowstatus == status::onground)
                {
                    pre_move_finish=true;
                }
                if(pre_move_finish)
                {
                    if( nowstatus == status::onground)
                        move_finish++;
                    else
                    {
                        move_finish=0;
                        pre_move_finish=false;
                    }
                }
                if(move_finish>10)
                {
                
                    cout << "完成" << endl;
                    if (isup)
                    {
                        move(0.18, 0, 4000000);
                    }
                    else
                    {
                        move(-0.18, 0, 6000000);
                    }
                    move(0, 0, 500000);
                    move_finish = true;
                    return true;
            
                }
                else
                {
                if (laststatus != status::onstairs && nowstatus == status::onground && (!pre_finish)) //预备上/下楼
                {
                    cout << "正在预调整姿态" << endl;
                    if (abs(delta_angle) > 0.02) //调整姿态
                    {
                        angular = pidImu.getResult();
                        linear = 0;
                        move(linear, angular, 0);
                    }
                    else //调整完毕
                    {
                        pidImu.reset();
                        move(0, 0, 500000);
                        pre_finish = true;
                    }
                }

                else //正在上/下楼
                {
                    cout << "正在楼梯上" << endl;
                    if (isup)
                    {
                        linear = 0.18;
                    }
                    else
                    {
                        linear = -0.18;
                    }
                    angular = pidImu.getResult();
                    move(linear, 0, 0);
                }
                }
                laststatus = nowstatus;
            }
            return true;
        }
        bool updownstairs(Eigen::Isometry3d goal_odom,bool isup)
        {
            ros::Rate rate(100);
            while (ros::ok()&&(imu_ok<50)&&(odom_ok<50))
            {
               rate.sleep();
               ros::spinOnce();
            }
            Eigen::Isometry3d imu_base_pose = Eigen::Isometry3d::Identity();
            imu_base_pose.prerotate(goal_odom.rotation());
            imu_base_pose.prerotate(odom_pose.rotation().inverse());
            Eigen::Isometry3d imu_copy=imu_pose;
            imu_copy.prerotate(imu_base_pose.rotation());
            imu_base_pose=imu_copy;
            Eigen::Isometry3d imu_goal_pose = Eigen::Isometry3d::Identity();
            Eigen::Quaterniond imu_base_qua=Eigen::Quaterniond(imu_base_pose.rotation());
            Eigen::Quaterniond imu_qua=Eigen::Quaterniond(imu_pose.rotation());
            cout<<"imu_base_qua"<<imu_base_qua.x()<<","<<imu_base_qua.y()<<","<<imu_base_qua.z()<<","<<imu_base_qua.w()<<endl;
            cout<<"imu_qua"<<imu_qua.x()<<","<<imu_qua.y()<<","<<imu_qua.z()<<","<<imu_qua.w()<<endl;
            return updownstairs(imu_base_pose,imu_goal_pose,isup);
        }
        bool updownstairs(bool isup, bool only_quaz)
        {
            int status = 1;
            ros::Rate maintick(100);
            double goal_quaz = 0;
            double goal_posx = 0;
            bool pos_ok = false;
            int ok_num=0;
            bool pre_ok = false;
            Vehicle::status laststatus = Vehicle::status::unkown_status;
            Vehicle::status nowstatus = Vehicle::status::unkown_status;
            PID pidApritag(1,0.002,200,-1);
            Eigen::Isometry3d pre_base_imu;
            while (ros::ok()&&(!pre_ok))
            {
                maintick.sleep();
                ros::spinOnce();
                switch (status)
                {
                case 1: // apriltag 姿态对准
                {
                    if (apriltag_ok == 0)
                    {
                        cout << "apriltag没好" << endl;
                        move(0, 0.32, 0);
                        continue;
                    }
                    Eigen::Quaterniond qua(apriltag_pose.rotation());
                    double delta_quaz = qua.z() - goal_quaz;
                    pidApritag.setError(delta_quaz);
                    pidApritag.printStr("-----apriltag校准----");
                    if((abs(delta_quaz) < delta_quaz_limit))
                    {
                        ok_num++;
                    }
                    else
                    {
                        ok_num=0;
                    }
                    if (ok_num>50)
                    {
                        move(0, 0, 500000);
                        if ((pos_ok || only_quaz))
                        {
                            pre_base_imu = imu_pose;
                            if (!isup)
                            {
                                pre_base_imu.prerotate(Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitZ()));
                            }
                            lastBase=pre_base_imu;
                            pre_ok = true;
                        }
                        else
                        {
                            status = 2;
                        }
                    }
                    else
                    {
                        move(0, pidApritag.getResult(), 0);
                    }
                    break;
                }
                case 2: // apriltag 位置对准
                {
                    if (apriltag_ok == 0)
                    {
                        cout << "apriltag没好" << endl;
                        move(0, 0.32, 0);
                        continue;
                    }
                    double delta_posx = apriltag_pose.translation()[0] - goal_posx;
                    cout << "delta_posx: " << delta_posx << endl;
                    double flag = delta_posx < 0 ? -1 : 1;
                    if (abs(delta_posx) < delta_posx_limit)
                    {
                        cout << "位置对准: " << endl;
                        move(0, 0, 500000);
                        pos_ok = true;
                    }
                    else
                    {
                        move(0, 0.32, 6500000);                                       //旋转
                        move(0, 0, 500000);                                           //停止
                        move(flag * 0.15, 0, abs(delta_posx) * (1000000.0) / (0.15)); //直走
                        move(0, 0, 500000);                                           //停止
                        move(0, -0.32, 6500000);                                      //旋转
                        move(0, 0, 500000);                                           //停止
                    }
                    status = 1;
                }
                default:
                    return false;
                }
            }
            Eigen::Isometry3d imu_goal_pose = Eigen::Isometry3d::Identity();
            return updownstairs(pre_base_imu,imu_goal_pose,isup);
        }
        bool updownstairs(bool isup)
        {
            ros::Rate rate(100);
            rate.sleep();
            ros::spinOnce();
            sleep(3);
            Eigen::Isometry3d imu_base_pose = imu_pose;

            Eigen::Isometry3d imu_goal_pose = Eigen::Isometry3d::Identity();
            return updownstairs(imu_base_pose,imu_goal_pose,isup);
        }
   
    };
}
#endif //_MY_STAIRSHANDLER_HPP_