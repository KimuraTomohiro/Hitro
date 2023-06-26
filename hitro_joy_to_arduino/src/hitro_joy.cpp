#include <bits/stdc++.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include "ps5controller/PS5Controller.h"
#include <geometry_msgs/Twist.h>
#include <flipper_msgs/Flipper.h>

ros::Publisher pub_arm_twist ;
ros::Publisher pub_arm_button;
ros::Publisher pub_twist,pub_flipper;
flipper_msgs::Flipper flipper_target;

int CONTROL_MODE=0;//0:FLIPPER_MODE,1:ARM_MODE
bool R3_PUSHING=false;
bool CARI_PUSHING=false;
std_msgs::Float32MultiArray pos; // arduinoに送るメッセージ
const float max_vel=0.4;
const float max_angular=0.4;

void pub_crawler(const ps5controller::PS5ControllerConstPtr& msg){
      geometry_msgs::Twist target;

            // クローラの制御
        float LStickUp = msg->LStickUp;
        float LStickLeft = msg->LStickLeft;
        //コントローラのノイズ対策	
		const float spd_thresh = 0.12;	
		if( -spd_thresh <= LStickLeft && LStickLeft <= spd_thresh) LStickLeft = 0;
		if( -spd_thresh <= LStickUp && LStickUp <= spd_thresh) LStickUp = 0;

        target.linear.x=max_vel*LStickUp;
        target.angular.z=max_angular*LStickLeft;

        pub_twist.publish(target);

}

void pub_Flipper(const ps5controller::PS5ControllerConstPtr& msg){
        if(msg->Circle){ CARI_PUSHING=true;}
        else{
            if(CARI_PUSHING){
                flipper_target.CARIBLATE=true;
                  //  pub_flip_cari.publish(cari);
                    CARI_PUSHING=false;}
                else{
                    flipper_target.CARIBLATE=false;
                    }
                }

        flipper_target.velocity.at(0)=-(msg->R1 - msg->R2Button);
        flipper_target.velocity.at(1)=msg->L1 - msg->L2Button;
        flipper_target.velocity.at(2)=msg->Triangle - msg->Cross;
        flipper_target.velocity.at(3)=-msg->CrossUp;
/*
    bool xor_pos=false;
        for(int i=0;i<6;i++){
            if(pos.data[i]==old_pos.data[i]){//donothing
            }
            else{
                xor_pos=true;
                old_pos.data[i]=pos.data[i];
            }
        }
        if(xor_pos){
*/
            pub_flipper.publish(flipper_target);
  //      }
        }

void pub_arm(const ps5controller::PS5ControllerConstPtr& ps5_msg){
double max_speed=0.5;//[m/s]
double max_angle_speed=1;//[rad/s]
double max_up = 0.1/2;//足をどれだけまで上げれるか[m]
//double max_curvature=6.9686;//[1/m]
double X=ps5_msg->LStickUp;
double Y=ps5_msg->RStickLeft;
double X1=ps5_msg->LStickLeft;//LStickの左右の倒し具合、角度関係:右-1.0~1.0左
double Y1=ps5_msg->LStickUp;//前後の倒し具合、角度関係:後ろ-1.0~1.0前
double L2=ps5_msg->RStickLeft;//L2
double RS_lr=ps5_msg->L2Analog;//RStickの前後の倒し具合、角度関係:右-1.0~1.0左
double RS_fb=ps5_msg->R2Analog;//RStickの前後の倒し具合、角度関係:後ろ-1.0~1.0前
double R2=ps5_msg->RStickUp;//R2
double CrossLeft=ps5_msg->CrossLeft;//十字の左:1,右:-1
double CrossUp=ps5_msg->CrossUp;//十字の上:1,下:-1
std_msgs::Float32MultiArray button_array;
button_array.data.resize(13);
geometry_msgs::Twist target;

target.linear.x=RS_lr;//L2
target.linear.y=RS_fb;//R2
target.linear.z=CrossLeft;//十字横
target.angular.x=CrossUp;//十字縦
target.angular.y=Y;//RSの左右//0905追加
target.angular.z=R2;//RSの前後//0905追加

button_array.data[0]=ps5_msg->L1;
button_array.data[1]=ps5_msg->R1;
button_array.data[2]=ps5_msg->Square;
button_array.data[3]=ps5_msg->Triangle;
button_array.data[4]=ps5_msg->Cross;
button_array.data[5]=ps5_msg->L2Button;
button_array.data[6]=ps5_msg->R2Button;
button_array.data[7]=ps5_msg->SHARE;
button_array.data[8]=ps5_msg->OPTION;
button_array.data[9]=ps5_msg->L3;
button_array.data[10]=ps5_msg->R3;
button_array.data[11]=ps5_msg->PSButton;
button_array.data[12]=ps5_msg->Circle;

pub_arm_twist.publish(target);
pub_arm_button.publish(button_array);


}

void ps5toTwist(const ps5controller::PS5ControllerConstPtr& ps5_msg){
   
   if(ps5_msg->R3==true){
        R3_PUSHING=true;
   }
   else{
    if(R3_PUSHING){
        if(CONTROL_MODE==0){CONTROL_MODE=1;}
        else{CONTROL_MODE=0;}
        R3_PUSHING=false;
    }
   }
   
    switch(CONTROL_MODE){
        case 0://FLIPPER_MODE
            pub_Flipper(ps5_msg);
        
        break;
        case 1://ARM_MODE
            pub_arm(ps5_msg);
        break;
        default://not exist
        break;
    }
   pub_crawler(ps5_msg);



}

int main(int argc, char** argv){
    ros::init(argc, argv, "hitro_joy_to_arduino");
    ros::NodeHandle nh("~");

    flipper_target.velocity.resize(4);
    pub_twist= nh.advertise<geometry_msgs::Twist>("/cmd_vel_joy", 1);
    pub_flipper=nh.advertise<flipper_msgs::Flipper>("/flipper_target", 1);
    pub_arm_twist = nh.advertise<geometry_msgs::Twist>("/arm_cmd_vel", 1);
    pub_arm_button = nh.advertise<std_msgs::Float32MultiArray>("/arm_cmd_button", 1);


    ros::Subscriber sub_ps5 = nh.subscribe("/ps5controller", 1, ps5toTwist);
   // ros::Subscriber ps5_msg_sub = nh.subscribe<ps5controller::PS5Controller>("/ps5controller", 1, ps5msgCb);
    ros::Rate loop_rate(20);
    ros::spin();
}