/**
 *  rosbag_remote.cpp
 *  author: Lukas Meier <lukasdanmeier@gmail.com>
 *  ROS_Node to remotely trigger the recording of rosbags (eg. from Matlab)
 */

#include "rosbag_remote/rosbag_remote.hpp"


namespace rosbag_remote {

RosbagRemote::RosbagRemote() : nh_("~") {
      start_sub_ = nh_.subscribe<std_msgs::String>("/remote/start", 1, &RosbagRemote::startMsgCb, this);
      stop_sub_ = nh_.subscribe<std_msgs::Empty>("/remote/stop", 1, &RosbagRemote::stopMsgCb, this);

      //Define Callbacks for targets (only one target at the moment)
      target1_pos_sub_ = nh_.subscribe("/camera_target_1/position", 1, &RosbagRemote::NavSatFixCb, this);
      target1_press_sub_ = nh_.subscribe("/camera_target_1/pressure", 1, &RosbagRemote::FluidPressureCb, this);

      //Define Callbacks for Quads (only one quad at the moment)
      quad1_joint_sub_ = nh_.subscribe("/q1/real/joint_states", 1, &RosbagRemote::JointStatesCb, this);
      quad1_alt_sub_ = nh_.subscribe("/q1/real/altitude", 1, &RosbagRemote::AltitudeCb, this);
      quad1_pos_sub_ = nh_.subscribe("/q1/real/fix", 1, &RosbagRemote::NavSatFixCb, this);
      quad1_image_sub_ = nh_.subscribe("/q1/real/image_raw", 1, &RosbagRemote::ImageCb, this);
      quad1_odom_sub_ = nh_.subscribe("/q1/real/odom", 1, &RosbagRemote::OdomCb, this);

      //Define visualization Subcribers
      vis_quad_state_sub_ = nh_.subscribe("/visualization/QuadrotorState", 1, &RosbagRemote::MultiArray64Cb, this);
      vis_target_state_sub_ = nh_.subscribe("/visualization/TargetState", 1, &RosbagRemote::MultiArray64Cb, this);
      vis_model_sub_ = nh_.subscribe("/visualization/model", 1, &RosbagRemote::MultiArray64Cb, this);
      vis_p_sub_ = nh_.subscribe("/visualization/p", 1, &RosbagRemote::MultiArray64Cb, this);
      vis_setpoints_ = nh_.subscribe("/visualization/setpoints", 1, &RosbagRemote::MultiArray64Cb, this);
      vis_setpoints_quad_ = nh_.subscribe("/visualization/setpointsQuadrotor", 1, &RosbagRemote::MultiArray64Cb, this);
      vis_setpoints_target_ = nh_.subscribe("/visualization/setpointsTarget", 1, &RosbagRemote::MultiArray64Cb, this);
      vis_x_planned_ = nh_.subscribe("/visualization/x_planned", 1, &RosbagRemote::MultiArray64Cb, this);

      vis_x_debug_mode_ = nh_.subscribe("/visualization/debug_mode", 1, &RosbagRemote::EmptyCb, this);

      pw = getpwuid(getuid());

      homedir = pw->pw_dir;

      //Define Callbacks for Visualization


      printf("\nRosbag Remote Node Started!\n");
}

      void RosbagRemote::startMsgCb(const std_msgs::String::ConstPtr &msg) {
        if (is_recording==0){
          //Get the directory name
          std::string hdir(homedir);
          std::string folder = "/rosbags/";
          std::string prefix = msg->data;
          //Get Date string to name the rosbag
          time_t rawtime;
          struct tm * timeinfo;
          char buffer[80];
          time (&rawtime);
          timeinfo = localtime(&rawtime);
          strftime(buffer,80,"_%Y-%m-%d-%H:%M:%S",timeinfo);
          std::string date(buffer);

          //Open a rosbag and start recording
          bag.open(hdir + folder + prefix + date + ".bag", rosbag::bagmode::Write);
          printf("Started Recording\n");
          is_recording=1;
        }
      }

      void RosbagRemote::stopMsgCb(const std_msgs::Empty::ConstPtr &msg) {
        bag.close();
        printf("Stopped Recording\n");
        is_recording=0;
      }



      void RosbagRemote::NavSatFixCb(const ros::MessageEvent<sensor_msgs::NavSatFix const>& event) {

        if (is_recording==1)
        {
          const ros::M_string& header = event.getConnectionHeader();
          std::string topic = header.at("topic");
          const sensor_msgs::NavSatFix::ConstPtr& msg = event.getMessage();
          ros::Time receipt_time = event.getReceiptTime();

          // std::cout << "Write message from Topic: " << topic << "\n";

          bag.write(topic,receipt_time,msg);
        }
      }

      void RosbagRemote::FluidPressureCb(const ros::MessageEvent<sensor_msgs::FluidPressure const>& event) {
        if (is_recording==1)
        {
          const ros::M_string& header = event.getConnectionHeader();
          std::string topic = header.at("topic");
          const sensor_msgs::FluidPressure::ConstPtr& msg = event.getMessage();
          ros::Time receipt_time = event.getReceiptTime();

          // std::cout << "Write message from Topic: " << topic << "\n";

          bag.write(topic,receipt_time,msg);
        }
      }

      void RosbagRemote::JointStatesCb(const ros::MessageEvent<sensor_msgs::JointState const>& event) {
        if (is_recording==1)
        {
          const ros::M_string& header = event.getConnectionHeader();
          std::string topic = header.at("topic");
          const sensor_msgs::JointState::ConstPtr& msg = event.getMessage();
          ros::Time receipt_time = event.getReceiptTime();

          // std::cout << "Write message from Topic: " << topic << "\n";

          bag.write(topic,receipt_time,msg);
        }
      }

      void RosbagRemote::AltitudeCb(const ros::MessageEvent<std_msgs::Float64 const>& event) {
        if (is_recording==1)
        {
          const ros::M_string& header = event.getConnectionHeader();
          std::string topic = header.at("topic");
          const std_msgs::Float64::ConstPtr& msg = event.getMessage();
          ros::Time receipt_time = event.getReceiptTime();

          // std::cout << "Write message from Topic: " << topic << "\n";

          bag.write(topic,receipt_time,msg);
        }
      }

      void RosbagRemote::ImageCb(const ros::MessageEvent<sensor_msgs::Image const>& event) {
        if (is_recording==1)
        {
          const ros::M_string& header = event.getConnectionHeader();
          std::string topic = header.at("topic");
          const sensor_msgs::Image::ConstPtr& msg = event.getMessage();
          ros::Time receipt_time = event.getReceiptTime();

          // std::cout << "Write message from Topic: " << topic << "\n";

          bag.write(topic,receipt_time,msg);
        }
      }

      void RosbagRemote::OdomCb(const ros::MessageEvent<nav_msgs::Odometry const>& event) {
        if (is_recording==1)
        {
          const ros::M_string& header = event.getConnectionHeader();
          std::string topic = header.at("topic");
          const nav_msgs::Odometry::ConstPtr& msg = event.getMessage();
          ros::Time receipt_time = event.getReceiptTime();

          // std::cout << "Write message from Topic: " << topic << "\n";

          bag.write(topic,receipt_time,msg);
        }
      }

      void RosbagRemote::MultiArray64Cb(const ros::MessageEvent<std_msgs::Float64MultiArray const>& event) {
        if (is_recording==1)
        {
          const ros::M_string& header = event.getConnectionHeader();
          std::string topic = header.at("topic");
          const std_msgs::Float64MultiArray::ConstPtr& msg = event.getMessage();
          ros::Time receipt_time = event.getReceiptTime();

          // std::cout << "Write message from Topic: " << topic << "\n";

          bag.write(topic,receipt_time,msg);
        }
      }

      void RosbagRemote::EmptyCb(const ros::MessageEvent<std_msgs::Empty const>& event) {
        if (is_recording==1)
        {
          const ros::M_string& header = event.getConnectionHeader();
          std::string topic = header.at("topic");
          const std_msgs::Empty::ConstPtr& msg = event.getMessage();
          ros::Time receipt_time = event.getReceiptTime();

          // std::cout << "Write message from Topic: " << topic << "\n";

          bag.write(topic,receipt_time,msg);
        }
      }


} // namespace bebop_vel_control
