#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace std;

void subscriberCallback(const std_msgs::String& msg){
    ROS_INFO("%s",msg.data.c_str());

}
int main(int argc,char **argv){
  ros::init(argc, argv, "controller");
  ros::NodeHandle nodeHandle;
  ros::Subscriber subscriber=nodeHandle.subscribe("/echo",10,subscriberCallback);
  ros::spin();
  return 0;  
}