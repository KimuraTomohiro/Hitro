#include "urdf2dxl.h"
using namespace std;

int main(int argc, char** argv){
    ros::init(argc,argv,"listener");
    XmlRpc::XmlRpcValue member_list; //read config file
    ros::NodeHandle pnh("~");
    pnh.getParam("member_list", member_list);
    ROS_ASSERT(member_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_INFO("member size: %i", (int)member_list.size());

    urdf2dxl udd((int)member_list.size());
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
    udd.joint_state(i,dxl_name);
  }
    
    ros::spin();
}
//ダイナミクセルの名前取得
void urdf2dxl::joint_state(int num,std::string name_joint){
    joint_rad.name.at(num)=name_joint;
    
}

void urdf2dxl::pub_dxl(const sensor_msgs::JointState& msg){
  //km_arm_1st
    // joint_rad.position[0]=msg.position[0];
    // joint_rad.position[1]=msg.position[1];
    // joint_rad.position[2]=msg.position[2];
    // joint_rad.position[3]=msg.position[3];
    // joint_rad.position[4]=msg.position[4];
    // joint_rad.position[5]=msg.position[5];
    // joint_rad.position[6]=msg.position[6];
    // joint_rad.position[7]=msg.position[6];
    // joint_rad.position[8]=msg.position[7];
  //km_arm_2nd
    // joint_rad.position[0]=msg.position[5];
    // joint_rad.position[1]=msg.position[0];
    // joint_rad.position[2]=msg.position[1];
    // joint_rad.position[3]=msg.position[2];
    // joint_rad.position[4]=msg.position[7];
    // joint_rad.position[5]=msg.position[4];
    // joint_rad.position[6]=msg.position[4];
    // joint_rad.position[7]=msg.position[3];
  //km_arm_3rd
    joint_rad.position[0]=msg.position[0];
    joint_rad.position[1]=msg.position[3];
    joint_rad.position[2]=msg.position[6];
    joint_rad.position[3]=msg.position[7];
    joint_rad.position[4]=msg.position[4];
    joint_rad.position[5]=msg.position[1];
    joint_rad.position[6]=msg.position[1];
    joint_rad.position[7]=msg.position[2];

    pub.publish(joint_rad);
}