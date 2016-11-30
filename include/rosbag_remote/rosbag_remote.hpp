
/**
 *  rosbag_remote.hpp
 *  author: Lukas Meier <lukasdanmeier@gmail.com>
 *  ROS_Node to remotely trigger the recording of rosbags (eg. from Matlab)
 */
 #include <unistd.h>
 #include <sys/types.h>
 #include <pwd.h>

 #ifndef __ROSBAG_REMOTE_HPP__
 #define __ROSBAG_REMOTE_HPP__

#include <rosbag/bag.h>
#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <ctime>

#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/FluidPressure.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Image.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64MultiArray.h"

 namespace rosbag_remote {

 class RosbagRemote {
 public:
   RosbagRemote();
   ros::Timer timer;

 private:
   struct passwd *pw;
   const char *homedir;

   bool is_recording=0;

   rosbag::Bag bag;


   ros::NodeHandle nh_;
   ros::Subscriber start_sub_;
   ros::Subscriber stop_sub_;

   ros::Subscriber target1_pos_sub_;
   ros::Subscriber target1_press_sub_;

   //Define callbacks for quads (only one quad at the moment)
   ros::Subscriber quad1_joint_sub_;
   ros::Subscriber quad1_alt_sub_;
   ros::Subscriber quad1_pos_sub_;
   ros::Subscriber quad1_image_sub_;
   ros::Subscriber quad1_odom_sub_;

   //Define callback for visualization topics
   ros::Subscriber vis_quad_state_sub_;
   ros::Subscriber vis_target_state_sub_;
   ros::Subscriber vis_model_sub_;
   ros::Subscriber vis_p_sub_;
   ros::Subscriber vis_setpoints_;
   ros::Subscriber vis_setpoints_quad_;
   ros::Subscriber vis_setpoints_target_;
   ros::Subscriber vis_x_planned_;

   ros::Subscriber vis_x_debug_mode_;


   void startMsgCb(const std_msgs::String::ConstPtr &msg);
   void stopMsgCb(const std_msgs::Empty::ConstPtr &msg);

   void NavSatFixCb(const ros::MessageEvent<sensor_msgs::NavSatFix const> &event);
   void FluidPressureCb(const ros::MessageEvent<sensor_msgs::FluidPressure const> &event);
   void JointStatesCb(const ros::MessageEvent<sensor_msgs::JointState const>& event);
   void AltitudeCb(const ros::MessageEvent<std_msgs::Float64 const>& event);
   void ImageCb(const ros::MessageEvent<sensor_msgs::Image const>& event);
   void OdomCb(const ros::MessageEvent<nav_msgs::Odometry const>& event);

   void MultiArray64Cb(const ros::MessageEvent<std_msgs::Float64MultiArray const>& event);
   void EmptyCb(const ros::MessageEvent<std_msgs::Empty const>& event);

 };

 } // namespace rosbag_remote

 #endif
