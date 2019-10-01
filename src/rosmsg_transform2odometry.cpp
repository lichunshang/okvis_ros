#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

ros::Publisher pub;

void callback(const geometry_msgs::TransformStamped &msg) {
  nav_msgs::Odometry odom;
  odom.header = msg.header;
  odom.child_frame_id = msg.child_frame_id;
  odom.pose.pose.orientation = msg.transform.rotation;
  odom.pose.pose.position.x = msg.transform.translation.x;
  odom.pose.pose.position.y = msg.transform.translation.y;
  odom.pose.pose.position.z = msg.transform.translation.z;
  pub.publish(odom);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "transform2odometry_node");
  // set up the node
  ros::NodeHandle nh("transform2odometry_node");

  pub = nh.advertise<nav_msgs::Odometry>("/vicon/gt_measurements", 1);
  ros::Subscriber sub = nh.subscribe("/vicon/firefly_sbx/firefly_sbx", 1, callback);
  ros::spin();

  return 0;
}
