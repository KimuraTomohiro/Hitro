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
class km_arm1
{
public:
  km_arm1(int motor_number){
        sub_ps5= nh.subscribe("/cmd_vel", 1,&km_arm1::joy2xyz,this);
        sub_button=nh.subscribe("/cmd_button", 1,&km_arm1::msg_button,this);
        // sub_ps5= nh.subscribe("/cmd_vel", 1,&km_arm1::urdf_check,this);
        
    // if(!urdf_on){
        joint_rad.header.stamp=ros::Time::now();
        joint_rad.name.resize(motor_number);
        joint_rad.position.resize(motor_number);
        joint_rad.velocity.resize(motor_number);
    // }
    // else{
        joint_rad_urdf.header.stamp=ros::Time::now();
        joint_rad_urdf.name.resize(8);
        joint_rad_urdf.position.resize(8);

        joint_rad_urdf.name[0]="square2base";
        joint_rad_urdf.name[1]="base21link";
        joint_rad_urdf.name[2]="1link22link";
        joint_rad_urdf.name[3]="2link2hand";
        joint_rad_urdf.name[4]="handyaw";
        joint_rad_urdf.name[5]="handroll";
        joint_rad_urdf.name[6]="top_right";
        joint_rad_urdf.name[7]="top_left";
        
    }
    ~km_arm1(){}
    void joy2xyz(const geometry_msgs::Twist& msg);//ps5メッセージを受け取り、xyzを変化
    void kinetics();//運動学
    void inverse_kinetics();//逆運動学
    void joint_state(int num,std::string name_joint);//ダイナミクセルの名前取得
    void ee_rotate();//エンドエフェクタを同期的に動かす
    void ee_dir();//エンドエフェクタの方向算出
    void dxl2pub();//ダイナミクセルにパブリッシュ
    void urdf2pub();//カメラに合わせるモード時のpub
    void urdf2pub_camera();//合わせないモード時のpub
    void urdf_check(const geometry_msgs::Twist& msg);
    void start();
    void msg_button(const std_msgs::Float32MultiArray& msg);
    void arm_updown(bool up);//アームを上下に動かす
    void reset_position();//アームをリセットする関数
    bool ik_check(int num);//xyz座標から距離を算出し、ikでnanにならないようにする
    void hand_ud(float ang_x);//ハンドを上下に変化
    void hand_lr(float lin_z);//ハンドを左右に変化
    void hand_roll(float roll);//ハンドを回転させる
    void vector_rotate(float theta1,float theta2,float theta3);//ベクトルを回転させる
    void hand_reset();//ハンドのみリセット
    void glipper_open_close(bool open);

private:
    ros::NodeHandle nh;
    ros::Publisher pub_dxl= nh.advertise<sensor_msgs::JointState>("/dxl_target_rad",1);//dxlにパブリッシュ;
    ros::Publisher pub_urdf= nh.advertise<sensor_msgs::JointState>("/joint_states",10);//dxlにパブリッシュ;
    ros::Subscriber sub_ps5,sub_button;
    sensor_msgs::JointState joint_rad;
    sensor_msgs::JointState joint_rad_urdf;

    //定数
    //リンクの長さ
    float L01=0.1f;
    float L12=0.4f;
    float L23=0.4f;
    float L34=0.2f;
    float arm_max=0.0f;//start()でL12とL23の長さから計算
    float arm_min=0.05f;//エンドエフェクタのx座標の位置がマイナスになるとよくないため
    float joy_mass=0.005f;//joyによるxyz_eeの変化量
    //実機かurdfか
    bool urdf_on=true;
    //変数
    //関節座標
    std::vector<float> joint1={0,0,0};
    std::vector<float> joint2={0,0,0};
    std::vector<float> joint3={0,0,0};
    std::vector<float> joint4={0,0,0};
    //関節角度
    std::vector<float> joint_angle={0,0,0,0,0,0,0,0};
    //V4-V3
    std::vector<float> v43_save={0,0,0};
    std::vector<float> v43_now={0,0,0};
    std::vector<float> v43_now_rotate={0,0,0};
    //エンドエフェクタのxyz
    std::vector<float> xyz_ee={0,0,0};
    //最初の姿勢のxyzの位置を保存しておく
    std::vector<float> xyz_ee_start={0,0,0};
    //エンドエフェクタを一定の方向に保つための、変化させる角度rad
    float roll_ee=0.0f;//z
    float yaw_ee=0.0f;//y
    float add=0.001f;
    bool once_startposi=true;
    float ch_angle3=0.0f;
    float ch_angle5=0.0f;
    //スイッチ
    bool upordown=false;//アームを上げる下げるか判別 上げるをtrue
    bool rotate_dir=false;//ハンド回転方向、右をtrue
    bool hand_openclose=false;//ハンド開閉、開くをtrue
    bool start_reset_button=false;//スタートに最初にオンにする
    bool camera_sync_sw=true;//カメラの上下に合わせるモード
    bool dxl_on=true;//trueでdxlも使う
    

};
//joint_angle
//0 base 
//1 base21link
//2 1link22link
//3 2link2hand
//4 yaw
//5 roll
//6 glip_left
//7 glip_right

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
