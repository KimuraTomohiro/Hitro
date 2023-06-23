#include <ros/ros.h>
// #include <moveit/move_group_interface/move_group_interface.h>
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
#include <arm_msgs/ArmCommand.h>

namespace std{};
class urdf2dxl
{
    public:
        urdf2dxl(int motor_number){
        sub_urdf= nh.subscribe("/joint_states", 1,&urdf2dxl::pub_dxl,this);
        
    // if(!urdf_on){
        joint_rad.header.stamp=ros::Time::now();
        joint_rad.name.resize(motor_number);
        joint_rad.position.resize(motor_number);
        joint_rad.velocity.resize(motor_number);
    // }
    // else{
        
    }
    ~urdf2dxl(){}
    void pub_dxl(const sensor_msgs::JointState& msg);
    void joint_state(int num,std::string name_joint);
    private:
        ros::NodeHandle nh;
        ros::Publisher pub= nh.advertise<sensor_msgs::JointState>("/dxl_target_rad",1);//dxlにパブリッシュ;
        ros::Subscriber sub_urdf;
        sensor_msgs::JointState joint_rad;

};
//km_arm_1st
//urdf joint_state 順番
//0 square2base
//1 2link2hand
//2 handroll　横向く
//3 handyaw  レバー回すのに使用
//4 top_right
//5 top_left
//6 base2link
//7 1link22link

//config
//0 square2base
//1 2link2hand
//2 handroll　横向く
//3 handyaw  レバー回すのに使用
//4 top_right
//5 top_left
//6 base2link_l
//7 base2link_r
//8 1link22link


//km_arm_2nd
//urdf joint_state 順番
//0 2link2hand
//1 handroll
//2 handyaw
//3 1link22link
//4 base21link
//5 square2base
//6 top_right
//7 top_left

// km_arm_3rd
//urdf joint_state 順番
//0 square2base
//1 base21link
//2 1link22link
//3 2link2hand
//4 top_left
//5 top_right
//6 handyaw
//7 handroll

//config
//0 square2base
//1 2link2hand
//2 handroll　横向く
//3 handyaw  レバー回すのに使用
//4 top_left
//5 base2link_l
//6 base2link_r
//7 1link22link