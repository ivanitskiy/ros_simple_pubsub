# ROS Node example

## Creating package
We need to setup our new workspace with command-line tools `catkin_create_pkg`. Then we will create a new ROS package. 

Create a new Workspace: 

    $ mkdir -p ~/dev/catkin_ws/src
    $ cd ~/dev/catkin_ws/src

Create a new package and specify dependencies to include:

    $ catkin_create_pkg simple_pubsub  roscpp std_msgs

Build package

    catkin_make

Remember to run `catkin_make` in the *workspace* folder.

## Implement publish subscribe

We need to implement a publish subscribe logic.

### Add publisher implementation:

Create a new file `~/dev/catkin_ws/src/simple_pubsub/src/publish.cpp`

```c++
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
```


### Add subscriber implementation:

Create a new file `~/dev/catkin_ws/src/simple_pubsub/src/subscribe.cpp`

```c++
#include "ros/ros.h" 
#include "std_msgs/String.h" 
 
void chatterCallback(const std_msgs::String::ConstPtr msg) 
{ 
  ROS_INFO("I heard: [%s]", msg->data.c_str()); 
} 
 
int main(int argc, char **argv) 
{ 
  ros::init(argc, argv, "simple_subscribe"); 
  ros::NodeHandle n; 
  ros::Subscriber sub = n.subscribe("message", 1000, 
     chatterCallback); 
  ros::spin(); 
  return 0; 
} 
```
Update 
```cmake
cmake_minimum_required(VERSION 2.8.3)
project(simple_pubsub)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
    include ${catkin_INCLUDE_DIRS}
)

add_executable(
    ${PROJECT_NAME}_publish src/publish.cpp
)
add_executable(
    ${PROJECT_NAME}_subscribe src/subscribe.cpp
)

add_dependencies(
    ${PROJECT_NAME}_publish ${PROJECT_NAME}_generate_messages_cpp
)
add_dependencies(
    ${PROJECT_NAME}_subscribe ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(
    ${PROJECT_NAME}_publish ${catkin_LIBRARIES}
)
target_link_libraries(
    ${PROJECT_NAME}_subscribe ${catkin_LIBRARIES}
)
```

## 

Start a master node in a new terminal:
    
    roscore

Start publisher in a new terminal:

    rosrun simple_pubsub simple_pubsub_publish

Start subscriber in a new terminal:

    rosrun simple_pubsub simple_pubsub_publish

You should see messages that publisher sends. 

