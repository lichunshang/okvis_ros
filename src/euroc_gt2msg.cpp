#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <fstream>
#include <iostream>
#include <sstream>

ros::Publisher pub;

int main(int argc, char **argv) {
  ros::init(argc, argv, "transform2odometry_node");

  if (argc != 2) {
    std::cout << "Usage: ./" << argv[0] << "gt-file-path" << std::endl;
    return -1;
  }

  // set up the node
  ros::NodeHandle nh("transform2odometry_node");

  std::string gt_file_path(argv[1]);
  ros::Rate loop_rate(1.0 / 200);

  pub = nh.advertise<nav_msgs::Odometry>("/euroc/gt_measurements", 1);

  std::ifstream gt_file;
  gt_file.open(gt_file_path);

  std::string line;
  std::getline(gt_file, line);  // first line header file

  std::vector<nav_msgs::Odometry> gt_data;

  unsigned int i = 0;
  while (std::getline(gt_file, line)) {
    std::stringstream ss(line);
    std::string c;
    ss >> c;
    double ts = std::stod(c);

    ss >> c;
    double x = std::stod(c);
    ss >> c;
    double y = std::stod(c);
    ss >> c;
    double z = std::stod(c);

    ss >> c;
    double qw = std::stod(c);
    ss >> c;
    double qx = std::stod(c);
    ss >> c;
    double qy = std::stod(c);
    ss >> c;
    double qz = std::stod(c);

    nav_msgs::Odometry odom;
    odom.header.seq = i;
    odom.header.stamp = ros::Time(ts);
    odom.child_frame_id = "gt_frame";
    odom.pose.pose.orientation.w = qw;
    odom.pose.pose.orientation.x = qx;
    odom.pose.pose.orientation.y = qy;
    odom.pose.pose.orientation.z = qz;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = z;
    gt_data.push_back(odom);
  }

  ros::Time data_start_time = ros::Time(0);
  ros::Time ros_start_time = ros::Time::now();
  std::vector<nav_msgs::Odometry>::iterator iter = gt_data.begin();
  while (ros::ok()) {
    ros::Time ros_curr_time = ros::Time::now();
    ros::Time data_curr_time = iter->header.stamp;

    while (ros_curr_time - ros_start_time > data_curr_time - data_start_time) {
      iter->header.stamp = ros::Time::now();
      pub.publish(*iter);
      iter++;
      data_curr_time = iter->header.stamp;
    }
  }

  return 0;
}
