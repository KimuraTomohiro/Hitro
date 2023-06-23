#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <string>
#include <vector>
#include <iomanip>
#include <cmath>
#include <chrono>
#include <thread>
#include <bits/stdc++.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>
// #include <arm_msgs/ArmCommand.h>

namespace std{};
class kamuy_arm_gui
{
public:
  kamuy_arm_gui(int motor_number){
        sub = nh.subscribe("/joint_states", 1,&kamuy_arm_gui::moveit_cb,this);
        sub1=nh.subscribe("/km_gui",1,&kamuy_arm_gui::glipper_move,this);
        sub2=nh.subscribe("/km_reset",1,&kamuy_arm_gui::reset_move,this);
        joint_rad.header.stamp=ros::Time::now();
        joint_rad.name.resize(motor_number);
        joint_rad.position.resize(motor_number);
        joint_rad.velocity.resize(motor_number);

    }
    ~kamuy_arm_gui(){}
    void moveit_cb(const sensor_msgs::JointState& msg);
    void joint_state(int num,std::string name_joint);
    void dxl2pub();
    void glipper_move(const std_msgs::Float32MultiArray& msg);
    void reset_move(const std_msgs::Float32MultiArray& msg);
private:
    ros::NodeHandle nh;
    ros::Publisher pub= nh.advertise<sensor_msgs::JointState>("/dxl_target_rad",1);//dxlにパブリッシュ;
    ros::Subscriber sub,sub1,sub2;
    sensor_msgs::JointState joint_rad;

    float glipper_open_rad=40/180.0f*3.14f;
    float glipper_close_rad=0;
    // std::vector<float> msg_position={0,0,0,0,0,0,65/180.0f*3.14f/2.0f,15/180.0f*3.14f};
    std::vector<float> msg_position={0,0,0,0,0,0,0,0};
    

};
//joint_state moveit
//0  square2base
//1  2link2hand
//2  handroll
//3  handyaw
//4  top_right
//5  top_left
//6  base21link
//7  1link22link

//config
//0 square2base
//1 b21l_l
//2 b21l_r
//3 1l22l
//4 2l2h
//5 sentan1
//6 sentan2
//7 handr
//8 handl



