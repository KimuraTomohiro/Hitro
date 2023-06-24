#include <bits/stdc++.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "ps5controller/PS5Controller.h"
#include <geometry_msgs/Twist.h>

ros::Publisher pub_twist ;
ros::Publisher pub_button;
ros::Publisher pub_ard;
int CONTROL_MODE=0;//0:FLIPPER_MODE,1:ARM_MODE
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

pub_twist.publish(target);
pub_button.publish(button_array);


}

void ps5toTwist(const ps5controller::PS5ControllerConstPtr& ps5_msg){
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


}

int main(int argc, char** argv){
    ros::init(argc, argv, "hitro_joy_to_arduino");
    ros::NodeHandle nh("~");
    pub_ard= nh.advertise<std_msgs::Float32MultiArray>("/hitro_joy_to_arduino", 1);
    pub_twist = nh.advertise<geometry_msgs::Twist>("/arm_cmd_vel", 1);
    pub_button = nh.advertise<std_msgs::Float32MultiArray>("/arm_cmd_button", 1);

    ros::Subscriber sub_ps5 = nh.subscribe("/ps5controller", 1, ps5toTwist);
   // ros::Subscriber ps5_msg_sub = nh.subscribe<ps5controller::PS5Controller>("/ps5controller", 1, ps5msgCb);
    ros::Rate loop_rate(120);

    ros::spin();
}