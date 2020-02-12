#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <tf2_msgs/TFMessage.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"

std::string user       = "user_";
std::string name       = "/action";
std::string path       = "/home/roboworks/actions_dataset/hsrb_rosbag/";
int         indexVideo = 0;
int         indexUser  = 1;
bool        recording  = false;

rosbag::Bag bag;

//string with topics names to be saved
std::string topic_cc = "/hsrb/head_center_camera/image_raw";
std::string topic_cl = "/hsrb/head_l_stereo_camera/image_rect_color";
std::string topic_cr = "/hsrb/head_r_stereo_camera/image_rect_color";
std::string topic_pc = "/hsrb/head_rgbd_sensor/depth_registered/rectified_points";
std::string topic_ch = "/hsrb/hand_camera/image_raw";
std::string topic_tf = "/tf";



//CALLBACKS TO WRITE INTO THE BAG
void callback_cc(const sensor_msgs::Image::ConstPtr& msg)
{
  if(recording)
  {
    bag.write(topic_cc, ros::Time::now(), msg);
    std::cout << ". ";
  }
}

void callback_cl(const sensor_msgs::Image::ConstPtr& msg)
{
  if(recording)
  {
    bag.write(topic_cl, ros::Time::now(), msg);
    std::cout << ". ";
  }
}

void callback_cr(const sensor_msgs::Image::ConstPtr& msg)
{
  if(recording)
  {
    bag.write(topic_cr, ros::Time::now(), msg);
    std::cout << ". ";
  }
}

void callback_ch(const sensor_msgs::Image::ConstPtr& msg)
{
  if(recording)
  {
    bag.write(topic_ch, ros::Time::now(), msg);
    std::cout << ". ";
  }
}

void callback_pc(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  if(recording)
  {
    bag.write(topic_pc, ros::Time::now(), msg);
    std::cout << ". ";
  }
}


void callback_tf(const tf2_msgs::TFMessage::ConstPtr& msg)
{
  if(recording)
  {
    bag.write(topic_tf, ros::Time::now(), msg);
    std::cout << ". " << std::endl;
  }
}


//CALLBACKS TO SAVE THE ROSBAG
void callback_start_record(const std_msgs::Bool::ConstPtr& msg)
{
  recording = true;
  indexVideo++;

  std::string c_in   = std::to_string(indexVideo);
  std::string c_user = std::to_string(indexUser);
  // name of the output video file
  std::string new_filename = path + user + c_user + name + c_in + ".bag";        
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

void callback_new_user(const std_msgs::Bool::ConstPtr& msg)
{
  indexUser++;
  std::cout << "Current recording user: " << indexUser << std::endl;
}



int main(int argc, char** argv)
{
  std::cout << "Initializing rosbag_record_node (HSRB)  BY Edd-2" << std::endl;
  ros::init(argc, argv, "rosbag_record_hsrb");
  ros::NodeHandle n;

  //Subcribers for topics
  ros::Subscriber subEyeRightCamera = n.subscribe(topic_cr, 1, callback_cr);
  ros::Subscriber subEyeLeftCamera  = n.subscribe(topic_cl, 1, callback_cl);
  ros::Subscriber subCenterCamera   = n.subscribe(topic_cc, 1, callback_cc);
  ros::Subscriber subHandCamera   = n.subscribe(topic_ch, 1, callback_ch);
  ros::Subscriber subPC   = n.subscribe(topic_pc, 1, callback_pc);
  ros::Subscriber subTF   = n.subscribe(topic_tf, 1, callback_tf);

  

  ros::Subscriber  subStart   = n.subscribe("/video_record/start", 1, callback_start_record);
  ros::Subscriber  subSave    = n.subscribe("/video_record/stop",  1, callback_stop_record);
  ros::Subscriber  subNewUser = n.subscribe("/video_record/new_user",  1, callback_new_user);
  
  ros::Time::init();
  ros::Rate  loop(60);

  std::cout << "  rosbagRecordNode -> Waiting for start record topic..." << std::endl;
  while( ros::ok() )
  {
    ros::spinOnce();
    loop.sleep();
  }
  
}
