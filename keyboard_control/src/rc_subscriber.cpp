#include <ros/ros.h>
#include <mavros_msgs/OverrideRCIn.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "rc_subscriber_node");
    ros::NodeHandle nh;
    
    ros::Rate loop_rate(10); // Publish at 10 Hz
    while (ros::ok()){

    }
}