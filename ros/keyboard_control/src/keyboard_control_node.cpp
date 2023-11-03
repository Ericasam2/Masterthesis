#include <ros/ros.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/RCIn.h>
#include <sensor_msgs/Imu.h>
#include <ncurses.h> // Include the ncurses library

class KeyboardControlNode
{
public:
    KeyboardControlNode() : nh("~"){
        rc_override_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1000);
        // imu_sub = nh.subscribe("/mavros/imu/data", 10, &KeyboardControlNode::imuCallback, this);
        RCIn_sub = nh.subscribe("/mavros/rc/in", 1000, &KeyboardControlNode::rcInCallback, this);
        initialize_rc_mapping();
        // Initialize ncurses for keyboard input
        initscr();
        cbreak();
        noecho();
	// keypad(stdscr, TRUE);
        nodelay(stdscr, TRUE);
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg){
        // Callback function for processing IMU data
        // You can add your IMU data processing code here
        // For example, you can access imu_msg->orientation, imu_msg->angular_velocity, imu_msg->linear_acceleration, etc.
    }

    void rcInCallback(const mavros_msgs::RCIn::ConstPtr& rcIn_msg){
        // Callback function for processing RC override In data
	ch = getch();
	// if (ch == ERR) { ROS_INFO("NO Input"); }
	// else { ROS_INFO("keyboard input: %c", ch); }
        for (int i = 0; i < 8; ++i){ rcIn_channels[i] = rcIn_msg->channels[i]; }
        rc_control_logic(rcIn_channels);
        for (int i = 0; i < 8; ++i){ rc_override_msg.channels[i] = rcOverride_channels[i]; }
    }

    void rcOverridePublish(){
	// ch = getch();
	// if (ch == ERR) { ROS_INFO("NO Input"); }
	// else { ROS_INFO("keyboard input: %c", ch); }
	if (ch != ERR) { 
		std::cout << "keyboard input: " << ch << std::endl; 
		std::cout << "channel 1: %d << " << rc_override_msg.channels[0] << ", channel 3: " << rc_override_msg.channels[2] << std::endl; 
		}

	// ROS_INFO("channel 1: %d, channel 3: %d \n", rc_override_msg.channels[0], rc_override_msg.channels[2]);
	// refresh();
        rc_override_pub.publish(rc_override_msg);
    }

    void initialize_rc_mapping(){
        charToVectorMap['w'] = {0, 0, 50, 0, 0, 0, 0, 0};
        charToVectorMap['s'] = {0, 0, -50, 0, 0, 0, 0, 0};
        charToVectorMap['a'] = {-50, 0, 0, 0, 0, 0, 0, 0};
        charToVectorMap['d'] = {50, 0, 0, 0, 0, 0, 0, 0};
    }

    void rc_control_logic(std::vector<int> rcIn_channels){
        if (ch == 'q'){
            // if "q" is pressed, switch to transmitter control
            rcOverride_channels = {0, 0, 0, 0, 0, 0, 0, 0};
        } 
        else if (ch == ERR) {
            // if no key is pressed, return to the default
            // rcOverride_channels = {1500, 0, 1400, 0, 0, 0, 0, 0};
        } 
        else{
            if (charToVectorMap.find(ch) != charToVectorMap.end()) {
                // if the key is within {"w","s","a","d"}
                rcOverride_channels = rcIn_channels;
                rcOverride_channels[0] += charToVectorMap[ch][0];
		rcOverride_channels[2] += charToVectorMap[ch][2];
		rcOverride_channels[0] = std::min(1900, std::max(rcOverride_channels[0], 1100));
		rcOverride_channels[2] = std::min(1900, std::max(rcOverride_channels[2], 1100));
            }
            else {
                // Handle invalid keys
                rcOverride_channels = {0, 0, 0, 0, 0, 0, 0, 0};
            }
        }
        // the car need to handle multiple input as well
        // tbd
    }



private:
    ros::NodeHandle nh;
    ros::Publisher rc_override_pub;
    // ros::Subscriber imu_sub;
    mavros_msgs::OverrideRCIn rc_override_msg;
    ros::Subscriber RCIn_sub;
    // KeyboardInput keyboard_reader;
    char ch;
    std::map<char, std::vector<int>> charToVectorMap;
    std::vector<int> rcIn_channels{ 0,0,0,0,0,0,0,0 };
    std::vector<int> rcOverride_channels{ 0,0,0,0,0,0,0,0 };
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_control_node");
    KeyboardControlNode keyboard_control_node;

    ros::Rate loop_rate(30); // Publish at 10 Hz
    ros::spinOnce();
    while (ros::ok())
    {
        // Publish the RC override message
        keyboard_control_node.rcOverridePublish();
        // run callback
        ros::spinOnce();
        loop_rate.sleep();
    }
    endwin();
    return 0;
}
