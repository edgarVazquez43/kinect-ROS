#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include "sensor_msgs/PointCloud2.h"


std::string name       = "user_1/action";
std::string path       = "/home/roboworks/actions_dataset/kinect_rosbag/";
std::string filename   =  path + name + ".bag";        // name of the output video file
int         indexVideo = 0;
bool        recording  = false;

rosbag::Bag              bag;
sensor_msgs::PointCloud2 msgCloudKinect;

std::string topic_name = "/kinect/camera/depth_registered/points";
//std::string topic_name = "/xtion/xtion/depth_registered/points";



void callback_pc(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  bag.write(topic_name, ros::Time::now(), msg);
}

void callback_start_record(const std_msgs::Bool::ConstPtr& msg)
{
  recording = true;
  indexVideo++;

  std::string c_in = std::to_string(indexVideo);
  std::string new_filename = path + name + c_in + ".bag";        // name of the output video file
  std::cout << new_filename << std::endl;

  bag.open(new_filename, rosbag::bagmode::Write);

  std::cout << " ------------------------------------------------ " << std::endl
	    << "Start recording" << std::endl
	    << "Press any key to terminate" << std::endl;
}

void callback_stop_record(const std_msgs::Bool::ConstPtr& msg)
{
  recording = false;
  std::cout << " ----- " << std::endl
	    << "STOP recording" << std::endl;

  bag.close();
}



int main(int argc, char** argv)
{
  std::cout << "Initializing rosbag_record_node (Kinect)  BY Edd-2" << std::endl;
  ros::init(argc, argv, "rosbag_record_node");
  ros::NodeHandle n;
  //Subcribers
  ros::Subscriber  subPC     = n.subscribe(topic_name, 1, callback_pc);
  ros::Subscriber  subStart  = n.subscribe("/video_record/start", 1, callback_start_record);
  ros::Subscriber  subSave   = n.subscribe("/video_record/stop",  1, callback_stop_record);
  
  ros::Time::init();
  ros::Rate  loop(60);

  std::cout << "  rosbagRecordNode -> Waiting for start record topic..." << std::endl;
  while( ros::ok() )
  {
    ros::spinOnce();
    loop.sleep();
  }
  
}
