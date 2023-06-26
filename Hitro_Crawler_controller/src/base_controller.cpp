#include <bits/stdc++.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "flipper_msgs/Flipper.h"
#include <geometry_msgs/Twist.h>

ros::Publisher pub_cmd_vel,pub_flipper_deg;
int CONTROL_MODE=0;//0:FLIPPER_MODE,1:ARM_MODE
bool R3_PUSHING=false;
std_msgs::Float32MultiArray vel_msg,flipper_deg_msg; // arduinoに送るメッセージ
flipper_msgs::Flipper current;

const float track_width=0.5;//

        // サブクローラの制御

        // pos.data[2] = -(msg->R1 - msg->R2Button);FR
        // pos.data[3] = msg->L1 - msg->L2Button;FL
        // pos.data[4] = msg->Triangle - msg->Cross;BR
        // pos.data[5] = -msg->CrossUp;BL
    

void cb_cmd_vel_joy(const geometry_msgs::Twist& msgs){
    vel_msg.data.resize(2);
    double v_l=0,v_r=0;
    v_l=msgs.linear.x-(track_width/2)*msgs.angular.z;
    v_r=msgs.linear.x+(track_width/2)*msgs.angular.z;

    const float convert_velocity2command=1;
    vel_msg.data[0]=convert_velocity2command*v_l;
    vel_msg.data[1]=convert_velocity2command*v_r;
    pub_cmd_vel.publish(vel_msg);
}
void cb_flipper_vel(const flipper_msgs::Flipper& msgs){
//    msgs.velocity
    if(msgs.CARIBLATE){
        flipper_deg_msg.data[4]=1;
    }
    else{
        flipper_deg_msg.data[4]=0;
    }
    
    const float convert_flipper_deg=0.5;
    
    if(current.update){
    //position_mode
    flipper_deg_msg.data[5]=1;
    for(int i=0;i<4;i++){
        flipper_deg_msg.data[i]=current.position.at(i)+convert_flipper_deg*msgs.velocity.at(i);
        if(flipper_deg_msg.data[i]>90){flipper_deg_msg.data[i]=90;}
        else if(flipper_deg_msg.data[i]<-90){flipper_deg_msg.data[i]=-90;}
    }
    }
    else{
    //velocity_mode
    flipper_deg_msg.data[5]=0;
    for(int i=0;i<4;i++){
        flipper_deg_msg.data[i]=msgs.velocity.at(i);
    }
    }
    pub_flipper_deg.publish(flipper_deg_msg);
}

void cb_flipper_pos(const std_msgs::Float32MultiArray& msgs){
        current.update=true;
        for(int i=0;i<4;i++){
        current.position.at(i)=msgs.data[i];
        }
}


int main(int argc, char** argv){
    ros::init(argc, argv, "hitro_crawler_controller");
    ros::NodeHandle nh("~");
    vel_msg.data.resize(2);//V_L,V_R
    flipper_deg_msg.data.resize(6);//FR,FL,BR,BL,CALLIBRATE,0:vel_1:pos
    current.position.resize(4);
    pub_cmd_vel= nh.advertise<std_msgs::Float32MultiArray>("/crawler_vel", 1);
    pub_flipper_deg=nh.advertise<std_msgs::Float32MultiArray>("/flipper_deg", 1);
    
    ros::Subscriber sub_cmd_vel_joy = nh.subscribe("/cmd_vel_joy", 1, cb_cmd_vel_joy);
    ros::Subscriber sub_flipper_command=nh.subscribe("/flipper_target",1,cb_flipper_vel);
    ros::Subscriber sub_flipper_position=nh.subscribe("/flipper_current_pos",1,cb_flipper_pos);//fromarduino

   // ros::Subscriber ps5_msg_sub = nh.subscribe<ps5controller::PS5Controller>("/ps5controller", 1, ps5msgCb);
    ros::Rate loop_rate(20);

    ros::spin();
}