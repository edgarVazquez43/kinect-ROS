#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <signal.h>
#include <termios.h>
#include <stdio.h>


#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71


int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();     // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}


int main(int argc, char** argv)
{
  std::cout << "INITIALIZING KEYBOARD-TELEOP  by EDD-2" << std::endl;
  ros::init(argc, argv, "keyboard_teleop");
  ros::NodeHandle n;
  ros::Rate loop(30);

  ros::Publisher pubStart   = n.advertise<std_msgs::Bool>("/video_record/start", 1);
  ros::Publisher pubStop    = n.advertise<std_msgs::Bool>("/video_record/stop", 1);
  ros::Publisher pubNewUser = n.advertise<std_msgs::Bool>("/video_record/new_user", 1);

  std_msgs::Bool msg;

  std::cout << "Press:" << std::endl
	    << "   [r] --> record "   << std::endl
	    << "   [s] --> stop "     << std::endl
	    << "   [n] --> new user " << std::endl
	    << "   [q] --> quit "     << std::endl; 

  while (ros::ok())
    {
      int c = getch();   // call your non-blocking input function

      if(c == 's')
      {
	std::cout << ":  keyboard-> Stop record..." << std::endl;
        pubStop.publish(msg);
      }
      else if(c == 'r')
      {
	std::cout << ":  keyboard-> Start record..." << std::endl;
	pubStart.publish(msg);
      }
      else if(c == 'n')
      {
	std::cout << ":  keyboard-> New user" << std::endl;
	pubNewUser.publish(msg);
      }
      else if(c == 'q')
      {	
	std::cout << ":  Exit" << std::endl;
	return 0;
      }

      ros::spinOnce();
      loop.sleep();
     
    }
  
  return 0;
}
