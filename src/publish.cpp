#include "ros/ros.h" 
#include "std_msgs/String.h" 
#include <sstream> 
#include <chrono>

int main(int argc, char **argv) 
{ 
  ros::init(argc, argv, "simple_publish"); 
  ros::NodeHandle n; 
  ros::Publisher chatter_pub = 
     n.advertise<std_msgs::String>("message", 1000); 
  ros::Rate loop_rate(10); 
  while (ros::ok()) 
  { 
    std_msgs::String msg; 
    std::stringstream ss; 

    auto tinenow = std::chrono::system_clock::now(); 
    std::time_t _time = std::chrono::system_clock::to_time_t(tinenow);

    ss<< " I am the publish node. Current time: " << std::ctime(&_time); 
    msg.data = ss.str(); 
    //ROS_INFO("%s", msg.data.c_str()); 
    chatter_pub.publish(msg); 
    ros::spinOnce(); 
    loop_rate.sleep(); 
  } 
  return 0; 
} 