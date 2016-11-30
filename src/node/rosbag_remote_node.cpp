
/**
 *  rosbag_remote_node.cpp
 *  author: Lukas Meier <lukasdanmeier@gmail.com>
 *  ROS_Node to remotely trigger the recording of rosbags (eg. from Matlab)
 */

#include "rosbag_remote/rosbag_remote.hpp"

#include <ros/ros.h>



int main(int argc, char *argv[]) {
  ros::init(argc, argv, "rosbag_remote");

  rosbag_remote::RosbagRemote bag_remote;

  ros::spin();
  return 0;
}
