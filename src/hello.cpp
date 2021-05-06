#include <ros/ros.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "hello_cpp");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("HELLO ROS!");
}