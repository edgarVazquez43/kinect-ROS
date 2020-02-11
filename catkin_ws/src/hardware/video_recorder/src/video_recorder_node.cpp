#include <iostream>
#include <string.h> 
#include <time.h>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Bool.h"
#include "point_cloud_manager/GetRgbd.h"
#include "tf/transform_listener.h"
#include "pcl_ros/transforms.h"
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH


std::string name       = "user_1/action";
std::string path       = "/home/roboworks/actions_dataset/playstationEye_camera/";
std::string filename   =  path + name + ".avi";        // name of the output video file
int         codec      = CV_FOURCC('M', 'J', 'P', 'G');  // select desired codec (must be available at runtime)
int         indexVideo = 0;
bool        recording  = false;
double      fps        = 30.0;                          // framerate of the created video stream


cv::VideoWriter   outputVideo;
//1280x 720
cv::Size          size(1280, 720);
cv::Mat           bgrImage;


void callback_start_record(const std_msgs::Bool::ConstPtr& msg)
{
  recording = true;
  indexVideo++;

  std::string c_in = std::to_string(indexVideo);
  std::string new_filename = path + name + c_in + ".avi";        // name of the output video file
  std::cout << new_filename << std::endl;
  outputVideo.open(new_filename, codec, fps, size, true);

  std::cout << " ------------------------------------------------ " << std::endl
	    << "Start recording" << std::endl
	    << "Press any key to terminate" << std::endl;

}

void callback_stop_record(const std_msgs::Bool::ConstPtr& msg)
{
  recording = false;
  std::cout << " ----- " << std::endl
	    << "STOP recording" << std::endl;

}


int main(int argc, char** argv)
{   
    std::cout << "INITIALIZING VIDEO RECORDER NODE  by EDD-2..." << std::endl;
    std::cout << "VideoRecorder.->Using Device 0 and OPENCV..." << std::endl;
    
    ros::init(argc, argv, "camera_record_node");
    ros::NodeHandle n;
    ros::Subscriber  subStart = n.subscribe("/video_record/start", 1, callback_start_record);
    ros::Subscriber  subSave  = n.subscribe("/video_record/stop",  1, callback_stop_record);

    ros::Rate loop(30);


    
    //###### SETUP FOR VIDEO RECORDING
    //--- INITIALIZE VIDEOWRITER
    cv::VideoCapture  capture(0);
    //1280x 720
    // capture.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
    // capture.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
    capture.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
    
    //Check if the device was succesfully opened
    if(!capture.isOpened())
    {
      std::cout << "VideoRecorder.-> Error opening video device" << std::endl;
      return -1;
    } 
    std::cout << "VideoRecorder.-> Web camera 0 started :D" << std::endl;

    
     while(ros::ok())
    {
      capture >> bgrImage;
      if(bgrImage.empty())
      {
	loop.sleep();
	//ros::spinOnce();
	continue;
      }
      
  
      //#########   Code for video recording  #############
      // encode the frame into the videofile stream
      if(recording)
	outputVideo.write(bgrImage);

      // show live and wait for a key with timeout long enough to show images
      cv::imshow("WebCam", bgrImage);
      if (cv::waitKey(5) >= 0)
	break;
      
      ros::spinOnce();
      loop.sleep();

    }

	    


    //delete tf_listener;	       
    return 0;
}
