#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <ps5controller/PS5Controller.h>
#include <geometry_msgs/Twist.h>
ros::Publisher pub_twist ;
ros::Publisher pub_button;
void ps5toTwist(const ps5controller::PS5ControllerConstPtr& ps5_msg){
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
//0825コメント化　↓
// target.linear.x=max_speed*X;
// target.angular.z=max_angle_speed*Y;
// if(target.linear.y<0){
//    target.linear.x=-target.linear.x;
//    // target.linear.y=target.linear.y;
// }
//0825コメント化　↑

//0825追加　↓
target.linear.x=RS_lr;//L2
target.linear.y=RS_fb;//R2
target.linear.z=CrossLeft;//十字横
target.angular.x=CrossUp;//十字縦
target.angular.y=Y;//RSの左右//0905追加
target.angular.z=R2;//RSの前後//0905追加

//0825追加　↑

//0825コメント化　↓
// target.linear.x=max_speed*X;
// target.angular.z=max_angle_speed*Y;
// if(target.linear.y<0){
//    target.linear.x=-target.linear.x;
//    // target.linear.y=target.linear.y;
// }
//0825コメント化　↑

// button_array.data[0]=ps5_msg->L1;
// button_array.data[1]=ps5_msg->R1;
// button_array.data[2]=ps5_msg->Square;
// button_array.data[3]=ps5_msg->Triangle;
// button_array.data[4]=ps5_msg->Cross;

//コメントは入力ボタン//melodicはプログラム名通り
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


//ROS_INFO("Speed=%lf,curvature=%lf",target.linear.x,target.linear.y);
pub_twist.publish(target);
pub_button.publish(button_array);

}

int main(int argc,char *argv[]){
ros::init(argc,argv,"ps5toTwist");
ros::NodeHandle nh;
ros::NodeHandle pnh("~");

ros::Subscriber sub_ps5 = nh.subscribe("/ps5controller", 1, ps5toTwist);
pub_twist = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
pub_button = nh.advertise<std_msgs::Float32MultiArray>("/cmd_button", 1);
ros::Rate loop_rate(120);

ros::spin();

return 0;
}

