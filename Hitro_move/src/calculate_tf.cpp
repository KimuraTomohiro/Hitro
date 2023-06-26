#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_example");

  ros::NodeHandle nh;

  tf::TransformListener listener;

  std::string target_frame_id = "base_link";//origin
  std::string source_frame_id = "hand_roll_1";//target

  tf::StampedTransform transform;
  try
  {
    listener.waitForTransform(target_frame_id, source_frame_id, ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform(target_frame_id, source_frame_id, ros::Time(0), transform);
    ROS_INFO("Translation: (%f, %f, %f)", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
    ROS_INFO("Rotation: (%f, %f, %f, %f)", transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("Failed to lookup transform: %s", ex.what());
  }

  return 0;
}
