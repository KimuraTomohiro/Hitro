//アームの逆運動学、運動学の計算を自分で考案して行う。
//ノートに考え方を保存、アームは６軸
#include "Hitro_arm.h"
using namespace std;

int main(int argc, char** argv){
    ros::init(argc,argv,"listener");
    XmlRpc::XmlRpcValue member_list; //read config file
    ros::NodeHandle pnh("~");
    pnh.getParam("member_list", member_list);
    ROS_ASSERT(member_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_INFO("member size: %i", (int)member_list.size());

    Hitro_arm ha((int)member_list.size());
  for (int32_t i = 0; i < member_list.size(); ++i)
  {
    ROS_INFO("read [%i]", i);
    int id = 0;
    std::string dxl_name = "";
    std::string port_name = "";
    std::string mode = "";
    int baud_rate=0;
    int model_number;
    if (!member_list[i]["dxl_id"].valid() || !member_list[i]["dxl_name"].valid())
    {
      ROS_WARN("No id or name");
      continue;
    }

    if (member_list[i]["dxl_name"].getType() == XmlRpc::XmlRpcValue::TypeString)
      dxl_name = static_cast<std::string>(member_list[i]["dxl_name"]);

    ROS_INFO("dxl_name: %s",dxl_name.c_str());
    ha.joint_state(i,dxl_name);
  }
    
    ros::spin();
}
//ダイナミクセルの名前取得
void Hitro_arm::joint_state(int num,std::string name_joint){
    joint_rad.name.at(num)=name_joint;
    
}
//運動学
void Hitro_arm::kinetics(){
  //kinetics version theta
  
    float j0=-1.0f*joint_angle[0];
    if(back_mode){
      j0=0.0f;
    }
    float j1=joint_angle[1];
    float j2=joint_angle[2];
    float j3=-1.0f*joint_angle[3];
    float j4=joint_angle[4];
    float j5=joint_angle[5];
    float s=L12*cos(j1)+L23*cos(j2);
    joint2[0]=L12*cos(j1)*cos(j0);
    joint2[1]=L12*cos(j1)*sin(j0);
    joint2[2]=L12*sin(j1);
    // std::cout<<"joint2_x::"<<joint2[0]<<",y::"<<joint2[1]<<",z::"<<joint2[2]<<endl;
    joint3[0]=joint2[0]+L23*cos(j1+j2)*cos(j0);
    joint3[1]=joint2[1]+L23*cos(j1+j2)*sin(j0);
    joint3[2]=joint2[2]+L23*sin(j1+j2);
    // std::cout<<"joint3_x::"<<joint3[0]<<",y::"<<joint3[1]<<",z::"<<joint3[2]<<endl;
    // std::cout<<j1+j2+j3<<endl;
    joint4[0]=joint3[0]+L34*cos(j5+j0)*cos(j1+j2+j3);
    joint4[1]=joint3[1]+L34*sin(j5+j0)*cos(j1+j2+j3);
    joint4[2]=joint3[2]+L34*sin(j1+j2+j3);
    // std::cout<<"joint4_x::"<<joint4[0]<<",y::"<<joint4[1]<<",z::"<<joint4[2]<<endl;
    if(!start_reset_button){
      for(int i=0;i<3;i++){
        xyz_ee_start[i]=joint3[i];
        xyz_ee[i]=joint3[i];
    }
    }
    // std::cout<<""<<endl;
    
}
//逆運動学
void Hitro_arm::inverse_kinetics(){
    float xyz=pow(xyz_ee[0],2)+pow(xyz_ee[1],2)+pow(xyz_ee[2],2);
    float s=pow(xyz_ee[0],2)+pow(xyz_ee[1],2);
    if(!back_mode){
      joint_angle[0]=atan2(xyz_ee[1],xyz_ee[0]);
    }
    joint_angle[1]=(acos((xyz-pow(L23,2)+pow(L12,2))/(2.0f*L12*sqrt(xyz))))+(atan2(xyz_ee[2],sqrt(s)));
    joint_angle[2]=atan2((xyz_ee[2]-L12*sin(joint_angle[1])),(sqrt(s)-L12*cos(joint_angle[1])))-joint_angle[1];
    // std::cout<<"joint_angle[2]::"<<joint_angle[2]<<endl;
}
//ps5メッセージを受け取り、xyzを変化、xyをjoyで変化させる
void Hitro_arm::joy2xyz(const geometry_msgs::Twist& msg){
    if(start_reset_button){
    float ang_y=msg.angular.y;//RStickの左右の倒し具合、角度関係:右-1.0~1.0左
    float ang_z=msg.angular.z;//RStickの上下の倒し具合、角度関係:下-1.0~1.0上
    float ang_x=msg.angular.x;//十字上下　上正
    float lin_x=msg.linear.x;//R2 デフォルト1、押すと-1
    float lin_y=msg.linear.y;//L2 デフォルト1、押すと-1
    float lin_z=msg.linear.z; //十字左右　左正
    float past_xyz_ee0=xyz_ee[0];
    float past_xyz_ee1=xyz_ee[1];
    float past_xyz_ee2=xyz_ee[2];
    if(std::abs(ang_y)<0.01f&&std::abs(ang_z)<0.01f){
      ang_y=0;
      ang_z=0;
    }
    if(std::abs(ang_y)>std::abs(ang_z)){//横の入力のほうが大きいとき
      if(back_mode){//バックモード時は、右ステック左右がターンテーブルのみ
          turn_table(ang_y);
      }
      else{//not backmode
        if(ang_y>0.01f&&ik_check(2)){//左右
          float joy1=joy_mass*change_value(ang_y);
        if(camera_sync_sw){
          vector_rotate_xy(joint_angle[4],0.0f,1.57f);//ベクトルを回転させる
          xyz_ee[0]=past_xyz_ee0+ang_y*joy1*v43_now_rotate[0];
          xyz_ee[1]=past_xyz_ee1+ang_y*joy1*v43_now_rotate[1];
          xyz_ee[2]=past_xyz_ee2+ang_y*joy1*v43_now_rotate[2];
        }
        else{
          xyz_ee[1]=past_xyz_ee1+ang_y*joy1;
        }        
      }
      else if(ang_y<-0.01f&&ik_check(3)){//左右
        float joy2=joy_mass*change_value(ang_y);
        if(camera_sync_sw){
          vector_rotate_xy(joint_angle[4],0.0f,1.57f);//ベクトルを回転させる
          xyz_ee[0]=past_xyz_ee0+ang_y*joy2*v43_now_rotate[0];
          xyz_ee[1]=past_xyz_ee1+ang_y*joy2*v43_now_rotate[1];
          xyz_ee[2]=past_xyz_ee2+ang_y*joy2*v43_now_rotate[2];
        }
        else{
          xyz_ee[1]=past_xyz_ee1+ang_y*joy2;
        }
      }
      }
      
    }
    else{//前後の入力のほうが大きいとき
      if(ang_z>0.01f&&ik_check(0)){//前後
        float joy3=joy_mass*change_value(ang_z);
        if(camera_sync_sw){
          xyz_ee[0]=past_xyz_ee0+ang_z*joy3*v43_now[0];
          xyz_ee[1]=past_xyz_ee1+ang_z*joy3*v43_now[1];
          xyz_ee[2]=past_xyz_ee2+ang_z*joy3*v43_now[2];
        }
        else{
          xyz_ee[0]=past_xyz_ee0+ang_z*joy3;
        }
        
      }
      else if(ang_z<-0.01f&&ik_check(1)){//前後
        float joy4=joy_mass*change_value(ang_z);
        if(camera_sync_sw){
          xyz_ee[0]=past_xyz_ee0+ang_z*joy4*v43_now[0];
          xyz_ee[1]=past_xyz_ee1+ang_z*joy4*v43_now[1];
          xyz_ee[2]=past_xyz_ee2+ang_z*joy4*v43_now[2];
        }
        else{
          xyz_ee[0]=past_xyz_ee0+ang_z*joy4;
        }
      }
    }
    // for(int i=0;i<3;i++){
    //   std::cout<<xyz_ee[i]<<endl;
    // }
    //十字上下選択時
    if(ang_x>0.5f||ang_x<-0.5f){
      hand_ud(ang_x);
    }
    else if(lin_z>0.5f||lin_z<-0.5f){
      hand_lr(lin_z);
    }
    //フリッパモード時、アームモード
    //R2、L2選択時
    if(!flipper_mode){
      if(lin_x<0.9f){
        float joy5=joy_mass*change_value(abs(lin_x-1.0f)/2.0f);
        arm_updown(true,joy5,lin_x);
      }
      else if(lin_y<0.9f){
        float joy6=joy_mass*change_value(abs(lin_y-1.0f)/2.0f);
        arm_updown(false,joy6,lin_y);
      }
    }
    
    //運動学
    kinetics();
    //先端方向算出joint3とjoint4間のベクトル
    ee_dir();
    //逆運動学
    inverse_kinetics();
    if(std::isfinite(joint_angle[2])){//計算値がnanでないとき
      if(camera_sync_sw){
        urdf2pub();
      }
      else{
        urdf2pub_camera();
      }  
    }
    else{
      //  ROS_ERROR("dead position");
    }
    // for(int i=0;i<3;i++){
    //   std::cout<<i<<"::"<<xyz_ee[i]<<endl;
    // }
    // std::cout<<""<<endl;
    // std::cout<<"x::"<<xyz_ee[0]<<",y::"<<xyz_ee[1]<<",z::"<<xyz_ee[2]<<endl;   
    }
    
}

void Hitro_arm::arm_updown(bool up,float value,float origin){
  float past_xyz_ee0=xyz_ee[0];
  float past_xyz_ee1=xyz_ee[1];
  float past_xyz_ee2=xyz_ee[2];
  float joy_controll=abs(origin-1.0f)/2.0f;

  if(up){
    if(ik_check(4)){
      // xyz_ee[2]=past_z+joy_mass;
      vector_rotate_xz(joint_angle[4],1.57f,0.0f);//ベクトルを回転させる
      xyz_ee[0]=past_xyz_ee0-v43_now_rotate[0]*value*joy_controll;
      xyz_ee[1]=past_xyz_ee1-v43_now_rotate[1]*value*joy_controll;
      xyz_ee[2]=past_xyz_ee2-v43_now_rotate[2]*value*joy_controll;
    }
  }
  else if(!up){
    if(ik_check(5)){
      // xyz_ee[2]=past_z-joy_mass;
      vector_rotate_xz(joint_angle[4],1.57f,0.0f);//ベクトルを回転させる
      xyz_ee[0]=past_xyz_ee0+v43_now_rotate[0]*value*joy_controll;
      xyz_ee[1]=past_xyz_ee1+v43_now_rotate[1]*value*joy_controll;
      xyz_ee[2]=past_xyz_ee2+v43_now_rotate[2]*value*joy_controll;
    }
  }
  
  kinetics();
  ee_dir();
  // ee_rotate();
  inverse_kinetics();
  urdf2pub();
}

void Hitro_arm::hand_lr(float ang_x){
  float past_value=joint_angle[5];
  joint_angle[5]=past_value+joy_mass*ang_x;
  ch_angle5+=joy_mass*ang_x;
  kinetics();
  for(int i=0;i<3;i++){
    v43_save[i]=joint4[i]-joint3[i];
  }
}
void Hitro_arm::hand_ud(float lin_z){
  float past_value=joint_angle[3];
  joint_angle[3]=past_value-joy_mass*lin_z;
  ch_angle3+=-joy_mass*lin_z;
  kinetics();
  for(int i=0;i<3;i++){
    v43_save[i]=joint4[i]-joint3[i];
  }
}

void Hitro_arm::hand_roll(float roll){
  float past_roll=joint_angle[4];
  joint_angle[4]=past_roll+roll*joy_mass;
  ch_angle4+=roll*joy_mass;
  
}

void Hitro_arm::ee_dir(){//エンドエフェクタの方向算出、imaxはyaw、pitchでついているため、真横に行ったときにrollを調整する必要があるので、この関数に処理を記述
    v43_now[0]=joint4[0]-joint3[0];
    v43_now[1]=joint4[1]-joint3[1];
    v43_now[2]=joint4[2]-joint3[2];

    
    // std::cout<<"joint4_x::"<<joint4[0]<<",y::"<<joint4[1]<<",z::"<<joint4[2]<<endl;
    // std::cout<<"x_now::"<<v43_now[0]<<endl;
    // std::cout<<"y_now::"<<v43_now[1]<<endl;
    // std::cout<<"z_now::"<<v43_now[2]<<endl;
    
}
void Hitro_arm::ee_rotate(){//エンドエフェクタの現在方向と保存した方向の間の角度を算出
    //長さ算出Vxy,Vxz
    float Vxy_now =sqrt(pow(v43_now[0],2)+pow(v43_now[1],2));
    float Vxy_save=sqrt(pow(v43_save[0],2)+pow(v43_save[1],2));
    float Vxyz_now=sqrt(pow(v43_now[0],2)+pow(v43_now[1],2)+pow(v43_now[2],2));
    float Vxyz_save=sqrt(pow(v43_save[0],2)+pow(v43_save[1],2)+pow(v43_save[2],2));
    // std::cout<<"Vxy_now::"<<Vxy_now<<endl;
    // std::cout<<"Vxy_save::"<<Vxy_save<<endl;
    // std::cout<<"x_save::"<<v43_save[0]<<endl;
    // std::cout<<"y_save::"<<v43_save[1]<<endl;
    // std::cout<<"z_save::"<<v43_save[2]<<endl;
    // std::cout<<"x_now::"<<v43_now[0]<<endl;
    // std::cout<<"y_now::"<<v43_now[1]<<endl;
    // std::cout<<"z_now::"<<v43_now[2]<<endl;
    //内積算出
    float diagram_xy=(v43_now[0]*v43_save[0]+v43_now[1]*v43_save[1])*99.99999f/100.0f;
    float diagram_xyz=(v43_now[0]*v43_save[0]+v43_now[1]*v43_save[1]+v43_now[2]*v43_save[2])*99.99f/100.0f;
    // std::cout<<"dia_xy::"<<diagram_xy<<"::dia_xyz::"<<diagram_xyz<<endl;
    // std::cout<<"Vxy_save::"<<Vxy_save<<"::Vxz_save::"<<Vxz_save<<endl;
    //角度算出
    float angle_xy =acos(diagram_xy/Vxy_now/Vxy_save);
    float angle_xyz=acos(diagram_xyz/Vxyz_now/Vxyz_save);
    float past_a5=joint_angle[5];
    float past_a3=joint_angle[3];
    joint_angle[5]=past_a5+angle_xy;
    joint_angle[3]=past_a3+angle_xyz;
    // std::cout<<"a5::"<<angle_xy<<"::a3::"<<angle_xyz<<"::ee::"<<diagram_xy/Vxy_now/Vxy_save<<endl;

}
void Hitro_arm::vector_rotate_xy(float theta1,float theta2,float theta3){
  // std::cout<<theta1<<endl;
  // std::cout<<"1::"<<v43_now[0]<<"::2::"<<v43_now[1]<<"::3::"<<v43_now[2]<<endl;
  float theta1_=theta1+1.57f;
  if(theta1<0.01f&&theta1>-0.01f){
    theta1_=theta1;
  }
  else if(theta1>0.01f){
    theta1_=theta1-1.57f;
  }

  float theta2_=theta2-ch_angle3;
  float theta3_=theta3;
  float vxy=v43_now[0]*cos(theta2_)+v43_now[2]*sin(theta2_);
  float vyy=v43_now[1];
  float vzy=-1.0f*v43_now[0]*sin(theta2_)+v43_now[2]*cos(theta2_);
  // std::cout<<"cos::"<<cos(theta2_)<<endl;
  // std::cout<<"sin::"<<sin(theta2_)<<endl;
  // std::cout<<"11::"<<vxy<<"::22::"<<vyy<<"::33::"<<vzy<<endl;
  float vxz=vxy*cos(theta3_)-vyy*sin(theta3_);
  float vyz=vxy*sin(theta3_)+vyy*cos(theta3_);
  float vzz=vzy;
  // std::cout<<"111::"<<vxz<<"::222::"<<vyz<<"::333::"<<vzz<<endl;
  v43_now_rotate[0]=vxz/L34;
  v43_now_rotate[1]=(vyz*cos(theta1_)-vzz*sin(theta1_))/L34;
  v43_now_rotate[2]=(vyz*sin(theta1_)+vzz*cos(theta1_))/L34;
  // std::cout<<theta2_<<endl;
  // for(int i=0;i<3;i++){
  //   std::cout<<i<<"::"<<v43_now_rotate[i]<<endl;
    
  // }
  //  std::cout<<sqrt(pow(v43_now_rotate[0],2)+pow(v43_now_rotate[1],2)+pow(v43_now_rotate[2],2))<<endl;
  // std::cout<<""<<endl;
}

void Hitro_arm::vector_rotate_xz(float theta1,float theta2,float theta3){
  // std::cout<<theta1<<endl;
  // std::cout<<"vxx::"<<vxx<<"::vyx::"<<vyx<<"::vzx::"<<vzx<<endl;
  // std::cout<<"1::"<<v43_now[0]<<"::2::"<<v43_now[1]<<"::3::"<<v43_now[2]<<endl;
  float theta1_=theta1+1.57f;
  if(theta1<0.01f&&theta1>-0.01f){
    theta1_=theta1;
  }
  else if(theta1>0.01f){
    theta1_=theta1-1.57f;
  }
  float theta2_=theta2;
  float theta3_=theta3-ch_angle5;
  float vxy=v43_now[0]*cos(theta3_)-v43_now[1]*sin(theta3_);
  float vyy=v43_now[0]*sin(theta3_)+v43_now[1]*cos(theta3_);
  float vzy=v43_now[2];
  // std::cout<<"11::"<<vxy<<"::22::"<<vyy<<"::33::"<<vzy<<endl;
  float vxz=vxy*cos(theta2_)+vzy*sin(theta2_);
  float vyz=vyy;
  float vzz=-1.0f*vxy*sin(theta2_)+vzy*cos(theta2_);
  // std::cout<<"111::"<<vxz<<"::222::"<<vyz<<"::333::"<<vzz<<endl;
  v43_now_rotate[0]=vxz/L34;
  v43_now_rotate[1]=(vyz*cos(theta1_)-vzz*sin(theta1_))/L34;
  v43_now_rotate[2]=(vyz*sin(theta1_)+vzz*cos(theta1_))/L34;

  // for(int i=0;i<3;i++){
  //   std::cout<<i<<"::"<<v43_now_rotate[i]<<endl;
    
  // }
  // std::cout<<sqrt(pow(v43_now_rotate[0],2)+pow(v43_now_rotate[1],2)+pow(v43_now_rotate[2],2))<<endl;
  // std::cout<<""<<endl;
}

void Hitro_arm::dxl2pub(){
    joint_rad.position[0]=joint_angle[0];
    joint_rad.position[1]=joint_angle[1];
    joint_rad.position[2]=joint_angle[1];
    joint_rad.position[3]=joint_angle[2];
    joint_rad.position[4]=joint_angle[3];
    joint_rad.position[5]=joint_angle[4];
    joint_rad.position[6]=joint_angle[5];
    joint_rad.position[7]=joint_angle[6];
    joint_rad.position[8]=joint_angle[7];
    pub_dxl.publish(joint_rad);
}
void Hitro_arm::urdf_check(const geometry_msgs::Twist& msg){
    joint_rad_urdf.header.stamp=ros::Time::now();
    if(roll_ee>3.0f){
      add=-0.001f;
    }
    else if(roll_ee<-3.0f){
      add=0.001f;
    }
    roll_ee+=add;
    
//     for(int i=0;i<8;i++){
//       joint_rad_urdf.position[i]=roll_ee;
//     }
//     pub_urdf.publish(joint_rad_urdf);
    start();
}

bool Hitro_arm::ik_check(int num){
  bool check=false;
  if(num==0){//x　プラス方向
    if((arm_max>sqrt(pow(xyz_ee[0]+joy_mass,2)+pow(xyz_ee[1],2)+pow(xyz_ee[2],2)))&&xyz_ee[0]+joy_mass>arm_min){
      check=true;
    }
  }
  else if(num==1){//x マイナス方向
    if((arm_max>sqrt(pow(xyz_ee[0]-joy_mass,2)+pow(xyz_ee[1],2)+pow(xyz_ee[2],2)))&&xyz_ee[0]-joy_mass>arm_min){
      check=true;
    }
  }
  else if(num==2){//y プラス方向
    if((arm_max>sqrt(pow(xyz_ee[1]+joy_mass,2)+pow(xyz_ee[0],2)+pow(xyz_ee[2],2)))){
      check=true;
    }
  }
  else if(num==3){//y マイナス方向
    if((arm_max>sqrt(pow(xyz_ee[1]-joy_mass,2)+pow(xyz_ee[0],2)+pow(xyz_ee[2],2)))){
      check=true;
    }
  }
  else if(num==4){//z プラス方向
    if((arm_max>sqrt(pow(xyz_ee[2]+joy_mass,2)+pow(xyz_ee[0],2)+pow(xyz_ee[1],2)))){
      check=true;
    }
  }
  else if(num==5){//z マイナス方向
    if((arm_max>sqrt(pow(xyz_ee[2]-joy_mass,2)+pow(xyz_ee[0],2)+pow(xyz_ee[1],2)))){
      check=true;
    }
  }
  
  return check;
}

void Hitro_arm::urdf2pub(){
    if(start_reset_button){
      calculate_tf();
    }
    joint_rad_urdf.header.stamp=ros::Time::now();
    joint_rad_urdf.position[0]=joint_angle[0]+ch_angle0;
    joint_rad_urdf.position[1]=-1.0f*(joint_angle[1]-1.569f);
    joint_rad_urdf.position[2]=joint_angle[2]+1.569f;
    // joint_rad_urdf.position[3]=joint_angle[3];
    joint_angle[3]=3.14f-(3.14f-std::abs(joint_angle[1]))-std::abs(joint_angle[2])+ch_angle3;
    joint_rad_urdf.position[3]=(3.14f-(3.14f-std::abs(joint_angle[1]))-std::abs(joint_angle[2])+ch_angle3);
    // joint_rad_urdf.position[4]=joint_angle[4]-1.0f*theta4_diff;
    joint_rad_urdf.position[4]=joint_angle[4];
    if(!back_mode){
      // joint_rad_urdf.position[5]=-1.0f*(-1.0f*joint_angle[0]+ch_angle5);
      joint_rad_urdf.position[5]=-1.0f*controll_theta5();
      joint_angle[5]=-1.0f*joint_angle[0]+ch_angle5;
      
    }
    else{
      joint_rad_urdf.position[5]=ch_angle5;
    }
    joint_rad_urdf.position[6]=joint_angle[6]*-1.0f;
    joint_rad_urdf.position[7]=joint_angle[7]*-1.0f;
    joint_rad_urdf.position[8]=cam_angle;
    joint_rad_urdf.position[9]=flipper_angle[2];
    joint_rad_urdf.position[10]=flipper_angle[3];
    joint_rad_urdf.position[11]=flipper_angle[0];
    joint_rad_urdf.position[12]=flipper_angle[1];
    pub_urdf.publish(joint_rad_urdf);

  if(dxl_on){
    // joint_rad.header.stamp=ros::Time::now();
    joint_rad.position[0]=(joint_angle[0]-ch_angle0)*-2.36f;
    joint_rad.position[5]=(joint_angle[1]-1.57f)*-2.03f;
    joint_rad.position[6]=(joint_angle[1]-1.57f)*-2.03f;
    joint_rad.position[7]=(joint_angle[2]+1.569f)*-2.03f;
    joint_rad.position[1]=joint_angle[3]*1.0f;
    joint_rad.position[3]=joint_angle[4]*-1.0f;
    if(!back_mode){
      joint_rad.position[2]=-1.0f*joint_angle[0]+ch_angle5;
      joint_angle[5]=1.0f*joint_angle[0]+ch_angle5;
    }
    else{
      joint_rad.position[2]=ch_angle5;
    }
    // joint_rad.position[2]=joint_angle[6]*-1.0f;
    joint_rad.position[4]=joint_angle[7]*-1.0f;
    joint_rad.position[8]=cam_angle;
    pub_dxl.publish(joint_rad);
  }
    
}


void Hitro_arm::urdf2pub_camera(){
    joint_rad_urdf.header.stamp=ros::Time::now();
    joint_rad_urdf.position[0]=joint_angle[0];
    joint_rad_urdf.position[1]=joint_angle[1]-1.569f;
    joint_rad_urdf.position[2]=joint_angle[2]+1.569f;
    joint_rad_urdf.position[3]=joint_angle[3];
    joint_rad_urdf.position[4]=joint_angle[4];
    joint_rad_urdf.position[5]=joint_angle[5];
    joint_rad_urdf.position[6]=joint_angle[6];
    joint_rad_urdf.position[7]=joint_angle[7];
    pub_urdf.publish(joint_rad_urdf);
}

void Hitro_arm::start(){
    arm_max=L12+L23;
    joint_angle[0]=0.0f;
    joint_angle[1]=135.0f/180.0f*3.14f;
    joint_angle[2]=-135.0f/180.0f*3.14f;
    joint_angle[3]=0.0f;
    joint_angle[4]=0.0f;
    joint_angle[5]=0.0f;
    joint_angle[6]=0.0f;
    joint_angle[7]=0.0f;
    
    kinetics();
   
    // for(int i=0;i<3;i++){
    //   v43_save[i]=joint4[i]-joint3[i];
    // }
    ee_dir();  
    // ee_rotate();
    inverse_kinetics();
    urdf2pub();
}

void Hitro_arm::reset_position(){
  // joint_angle[0]=0.0f;
  // joint_angle[1]=135.0f/180.0f*3.14f;
  // joint_angle[2]=-135.0f/180.0f*3.14f;
  // joint_angle[3]=0.0f;
  // joint_angle[4]=0.0f;
  // joint_angle[5]=0.0f;
  // joint_angle[6]=0.0f;
  // joint_angle[7]=0.0f;
  //現在地を保存
  // for(int i=0;i<joint_angle.size();i++){
  //   joint_angle_keep[i]=joint_angle[i];
  //   // std::cout<<i<<"::"<<joint_angle_keep[i]<<endl;
  // }
  // //
  // for(int i=0;i<100;i++){
  //   for(int k=0;k<joint_angle.size();k++){
  //     joint_angle[k]=joint_angle_keep[k]+(joint_st_reset[k]-joint_angle_keep[k])/100*i;
  //   }
  //   urdf2pub();
  //   std::this_thread::sleep_for(std::chrono::milliseconds(10));
  // }
  // //スタート時を保存
  // for(int i=0;i<joint_angle.size();i++){
  //   joint_angle[i]=joint_st_reset[i];
  // }
  go_position(joint_st_reset);
  // go_position(joint_around);
  for(int i=0;i<3;i++){
    xyz_ee[i]=xyz_ee_start[i];
    std::cout<<i<<"::"<<xyz_ee[i]<<endl;
  }
  std::cout<<""<<endl;
  ch_angle3=0.0f;
  ch_angle4=0.0f;
  ch_angle5=0.0f;
  kinetics();
  ee_dir();
  inverse_kinetics();
  urdf2pub();
}

void Hitro_arm::go_position(std::vector<float> position){//引数に指定したポジションにいく
  //現在地を保存
  for(int i=0;i<joint_angle.size();i++){
    joint_angle_keep[i]=joint_angle[i];
    // std::cout<<i<<"::"<<joint_angle_keep[i]<<endl;
  }
  //
  for(int i=0;i<100;i++){
    for(int k=0;k<joint_angle.size();k++){
      joint_angle[k]=joint_angle_keep[k]+(position[k]-joint_angle_keep[k])/100*i;
    }
    urdf2pub();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  //スタート時を保存
  for(int i=0;i<joint_angle.size();i++){
    joint_angle[i]=position[i];
  }
}

void Hitro_arm::hand_reset(){
  joint_angle[4]=0.0f;
  joint_angle[6]=0.0f;
  joint_angle[7]=0.0f;
  kinetics();
  ee_dir();
  inverse_kinetics();
  urdf2pub();
}

void Hitro_arm::glipper_open_close(bool open){
  if(open){
    joint_angle[6]=-20.0f/180.0f*3.14f;
    joint_angle[7]=20.0f/180.0f*3.14f;
  }
  else{
    joint_angle[6]=0.0f;
    joint_angle[7]=0.0f;
  }
  if(camera_sync_sw){
    urdf2pub();
  }
  else{
    urdf2pub_camera();
  }
  
}

void Hitro_arm::turn_table(float ang_y){
  ch_angle0-=ang_y*joy_mass;
}

float Hitro_arm::change_value(float analog_value){
  float abs_value=abs(analog_value);
  float analog_value_=pow(ch_value,abs_value)/ch_value;
  if(analog_value<0.01f&&analog_value>-0.01f){
    analog_value_=0.0f;
  }
  // std::cout<<analog_value<<endl;
  // std::cout<<analog_value_<<endl;
  // std::cout<<""<<endl;
  return analog_value_;
}

void Hitro_arm::controll_camera(float value){
  cam_angle+=3.14f*value/180.0f;
  urdf2pub();
}

void Hitro_arm::controll_flipper(float value,int num){
  if(value>0){
    float past_value=flipper_angle[num];
    flipper_angle[num]=past_value+flipper_mass;
  }
  else if(value<0){
    float past_value=flipper_angle[num];
    flipper_angle[num]=past_value-flipper_mass;
  }
  urdf2pub();
}

float Hitro_arm::controll_theta5(){
  // float theta3=joint_angle[3];
  float theta3=ch_angle3;
  float theta4=-1.0f*joint_angle[0]+ch_angle5;
  float l1=abs(L34*sin(theta3));
  float l2=abs(L34*cos(theta3));
  float l3=abs(l2*sin(theta4)/cos(theta4));
  float l4=sqrt(pow(l2,2)+pow(l3,2));
  float l5=sqrt(pow(l1,2)+pow(l4,2));
  float res=acos((pow(L34,2)+pow(l5,2)-pow(l3,2))/(2*L34*l5))*theta4/abs(theta4);
  if(!std::isfinite(res)){
    res=0.0f;
  }
  // std::cout<<"th3::"<<theta3<<endl;
  // std::cout<<"l1_::"<<l1<<endl;
  // std::cout<<"l2_::"<<l2<<endl;
  // std::cout<<"l3_::"<<l3<<endl;
  // std::cout<<"l4_::"<<l4<<endl;
  // std::cout<<"l5_::"<<l5<<endl;
  // std::cout<<"th4::"<<theta4<<endl;
  // std::cout<<"res::"<<res<<endl;
  // std::cout<<""<<endl;
  return res;
}
void Hitro_arm::calculate_tf(){
  try
  {
    listener.waitForTransform(target_frame_id, source_frame_id, ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform(target_frame_id, source_frame_id, ros::Time(0), transform);
    // ROS_INFO("Translation: (%f, %f, %f)", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
    // ROS_INFO("Rotation: (%f, %f, %f, %f)", transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
    
    double roll;
    double pitch;
    double yaw;
    tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw); //tfのQuartanionをRPY角に変更
    float past=theta4_diff;
    theta4_diff=-1.0f*ch_angle4-roll;
    if(theta4_diff<0.05f){
      theta4_diff=past;
    }
   
    
    std::cout<<theta4_diff<<endl;
    // ROS_INFO("rpy::%f,%f,%f",roll,pitch,yaw);
    // ROS_INFO("th::%f,%f,%f",ch_angle4,transform.getRotation().x(),theta5_diff);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("Failed to lookup transform: %s", ex.what());
  }

}
void Hitro_arm::msg_button(const std_msgs::Float32MultiArray& msg){
    if(msg.data[0]>0){//L1ボタン//ハンド左回転
      if(start_reset_button){
        if(flipper_mode){
          controll_flipper(1.0f,1);
        }
        else{
          hand_roll(1.0f);
        }
        
      } 
    }
    if(msg.data[1]>0){//R1//ハンド右回転
      if(start_reset_button){
        if(flipper_mode){
          controll_flipper(1.0f,0);
        }
        else{
          hand_roll(-1.0f);
        }       
      }
    }
    
    if(msg.data[4]>0){//Cross//ハンド閉まる
      if(flipper_mode){
          controll_flipper(-1.0f,2);
        }
      else{
          hand_openclose=false;
          glipper_open_close(false);
       }     
    }
    if(msg.data[12]>0){//Circle//ハンド開ける
      hand_openclose=true;
      glipper_open_close(true);
      
    }
    if(msg.data[2]>0){//Square//アームリセット
      if(start_reset_button){
        reset_position();  
        std::this_thread::sleep_for(std::chrono::seconds(1)); 
      }  
             
    }

    if(msg.data[3]>0){//Triangle//ハンドリセット
      if(start_reset_button){
        if(flipper_mode){
          controll_flipper(1.0f,2);
        }
        else{
          hand_reset();
        }
        
      }
    }
    if(msg.data[5]>0){//L2
      
      if(start_reset_button){
        if(flipper_mode){
          controll_flipper(-1.0f,1);
        }
      }      
    }
    if(msg.data[6]>0){//R2
      
      if(start_reset_button){
        if(flipper_mode){
          controll_flipper(-1.0f,0);
        }
      }    
    }
    if(msg.data[5]>0&&msg.data[6]>0){
      if(!start_reset_button){//最初の一回のみ
        start();
        start_reset_button=true;
      }
    }
    if(msg.data[7]>0){
      if(start_reset_button){
        controll_camera(1.0f);
        std::this_thread::sleep_for(chrono::milliseconds(100));
      }
    }
    if(msg.data[8]>0){
      if(start_reset_button){
        controll_camera(-1.0f);
        std::this_thread::sleep_for(chrono::milliseconds(100));
      }
    }
    // if(msg.data[8]>0){//|||ボタン、カメラ同期モード
    //   if(camera_sync_sw){
    //     camera_sync_sw=false;
    //     std::cout<<"camera mode off"<<endl;
    //     std::this_thread::sleep_for(chrono::milliseconds(500));
    //   }
    //   else if(!camera_sync_sw){
    //     camera_sync_sw=true;
    //     std::cout<<"camera mode on"<<endl;
    //     std::this_thread::sleep_for(chrono::milliseconds(500));
    //   }
    // }
    if(msg.data[10]>0){//R3　アーム、フリッパ変更
      if(start_reset_button){
        if(flipper_mode){
          std::cout<<"Arm mode"<<endl;
          flipper_mode=false;
          std::this_thread::sleep_for(chrono::milliseconds(500));
        }
        else{
          std::cout<<"Flipper mode"<<endl;
          flipper_mode=true;
          std::this_thread::sleep_for(chrono::milliseconds(500));
        }
        
      }
    }
    if(msg.data[11]>0){//psボタン、バックモード
      if(back_mode){
        back_mode=false;
        std::cout<<"back_mode off"<<endl;
        std::this_thread::sleep_for(chrono::milliseconds(500));
      }
      else if(!back_mode){
        back_mode=true;
        std::cout<<"back_mode on"<<endl;
        std::this_thread::sleep_for(chrono::milliseconds(500));
      }
    }

}
