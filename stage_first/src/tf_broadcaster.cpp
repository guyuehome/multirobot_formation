
//Source file for transformation broadcaster for both the robots

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "tf/transform_datatypes.h"

std::string robot_name;

  //call back decleration and implementation for pose
  void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0) );
  tf::Quaternion q;
  //Quaternion to Roll Pitch and Yaw conversion
  double roll, pitch, yaw;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  q.setRPY(0.0, 0.0, yaw);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", robot_name));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  if (argc != 2){ROS_ERROR("need robot name as argument"); return -1;};
  robot_name = argv[1];

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe(robot_name+"/base_pose_ground_truth", 10, &poseCallback);

  ros::spin();
  return 0;
};
