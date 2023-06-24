#include <bits/stdc++.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "ps5controller/PS5Controller.h"
#include <geometry_msgs/Twist.h>

ros::Publisher pub_twist ;
ros::Publisher pub_button;
ros::Publisher pub_ard;
int CONTROL_MODE=0;//0:FLIPPER_MODE,1:ARM_MODE
bool R3_PUSHING=false;
std_msgs::Float32MultiArray pos; // arduinoに送るメッセージ

void pub_Flipper(const ps5controller::PS5ControllerConstPtr& msg){
        pos.data.resize(6);
        // クローラの制御
        float LStickUp = msg->LStickUp;
        float LStickLeft = msg->LStickLeft;
        //コントローラのノイズ対策	
		const float spd_thresh = 0.12;	
		if( -spd_thresh <= LStickLeft && LStickLeft <= spd_thresh) LStickLeft = 0;
		if( -spd_thresh <= LStickUp && LStickUp <= spd_thresh) LStickUp = 0;

		float posR = LStickUp + LStickLeft;
		float posL = LStickUp - LStickLeft;

		const int MAX = 1;
        if(std::abs(posR) > MAX  ||  std::abs(posL) > MAX){  // 絶対値が1超えてるなら　大きい方を1に,　小さい方は比率を変えないように小さくする
            float large = std::max( std::abs(posR), std::abs(posL));
            posR = posR / large * MAX;
            posL = posL / large * MAX;
			//std::cout << " std::abs(pos0row): " << std::abs(pos0row) <<  "   std::abs(pos1row):  " << std::abs(pos1row) << "  large: " << large << "  MAX : " << MAX  << std::endl;
        }
		//std::cout << " joy_axes[0]: " << joy_axes[0] << " joy_axes[1]: " << joy_axes[1] << "  pos0row: " << pos0row << "   pos1row: "  <<  pos1row << std::endl;
		//pos[0].data = -1 * speed * pos0row * abs(pos0row);
		//pos[1].data = -1 * speed * pos1row * abs(pos1row);
        //	pos[0].data = 1 * speed * pos0row;

        if(posL>0.6){pos.data[1]=-1;}
        else if(posL>0.2){pos.data[1]=-0.5;}
        else if(posL <-0.6){pos.data[1] = 1;}
        else if(posL <-0.2){pos.data[1]=0.5;}
        else{pos.data[1]=0;}

        if(posR>0.6){pos.data[0]=1;}
        else if(posR>0.2){pos.data[0]=0.5;}
        else if(posR <-0.6){pos.data[0] = -1;}
        else if(posR <-0.2){pos.data[0]=-0.5;}
        else{pos.data[0]=0;}

        
        // サブクローラの制御
        pos.data[3] = msg->L1 - msg->L2Button;
        pos.data[2] = -(msg->R1 - msg->R2Button);
        pos.data[5] = -msg->CrossUp;
        pos.data[4] = msg->Triangle - msg->Cross;

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
            pub_ard.publish(pos);
  //      }
        }
void cmd_vel_joy(const geometry_msgs::Twist msgs){

}
int main(int argc, char** argv){
    ros::init(argc, argv, "hitro_crawler_controller");
    ros::NodeHandle nh("~");
    pub_ard= nh.advertise<std_msgs::Float32MultiArray>("/hitro_joy_to_arduino", 1);
    pub_twist = nh.advertise<geometry_msgs::Twist>("/arm_cmd_vel", 1);
    pub_button = nh.advertise<std_msgs::Float32MultiArray>("/arm_cmd_button", 1);

    ros::Subscriber sub_ps5 = nh.subscribe<geometry_msgs::Twist>("/cmd_vel_joy", 1, cmd_vel_joy);
   // ros::Subscriber ps5_msg_sub = nh.subscribe<ps5controller::PS5Controller>("/ps5controller", 1, ps5msgCb);
    ros::Rate loop_rate(120);

    ros::spin();
}