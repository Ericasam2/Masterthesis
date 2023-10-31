#include <ros/ros.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <sensor_msgs/Imu.h>
#include <ncurses.h> // Include the ncurses library

class KeyboardControlNode
{
public:
    KeyboardControlNode() : nh("~"), keyboard_reader(){
        rc_override_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);
        imu_sub = nh.subscribe("/mavros/imu/data", 10, &KeyboardControlNode::imuCallback, this);
        rc_override_sub = nh.subscribe("/mavros/rc/in", 10, &KeyboardControlNode::rcInCallback, this);
        initialize_rc_mapping();
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg){
        // Callback function for processing IMU data
        // You can add your IMU data processing code here
        // For example, you can access imu_msg->orientation, imu_msg->angular_velocity, imu_msg->linear_acceleration, etc.
    }

    void rcInCallback(const mavros_msgs::RCIn::ConstPtr& rcIn_msg){
        // Callback function for processing RC override In data
        rcIn_channels = rcIn_msg->channels[0];
        rcOverride_channels = rc_control_logic(rcIn_channels);
        rc_override_msg.channels = rcOverride_channels; 
    }

    void rcOverridePublish(){
        rc_override_pub.publish(rc_override_msg);
    }

    void initialize_rc_mapping(){
        charToVectorMap['w'] = {0  , 0,  10, 0, 0, 0, 0, 0};
        charToVectorMap['s'] = {0  , 0, -10, 0, 0, 0, 0, 0};
        charToVectorMap['a'] = {-10, 0,  0, 0, 0, 0, 0, 0};
        charToVectorMap['d'] = {10 , 0,  0, 0, 0, 0, 0, 0};
    }

    void rc_control_logic(vector<int> rcIn_channels){
        ch = keyboard_reader.get_keyboard_input();
        if (ch == 'q'){
            // if "q" is pressed, switch to transmitter control
            rcOverride_channels = {0, 0, 0, 0, 0, 0, 0, 0};
        } 
        else if (ch == ERR) {
            // if no key is pressed, return to the default
            rcOverride_channels = {1500, 0, 1400, 0, 0, 0, 0, 0};
        } 
        else{
            if (charToVectorMap.find(ch) != charToVectorMap.end()) {
                // if the key is within {"w","s","a","d"}
                rcOverride_channels = rcIn_channels;
                for (int i = 0; i < 4; ++i)
                {
                    rcOverride_channels[i] += charToVectorMap[ch][i];
                }
            }
            else {
                // Handle invalid keys
                rcOverride_channels = {1500, 0, 1400, 0, 0, 0, 0, 0};
            }
        }
        // the car need to handle multiple input as well
        // tbd
    }


private:
    ros::NodeHandle nh;
    ros::Publisher rc_override_pub;
    ros::Subscriber imu_sub;
    mavros_msgs::OverrideRCIn rc_override_msg;
    KeyboardInput keyboard_reader;
    char ch;
    std::map<char, std::vector<int>> charToVectorMap;
    std::vector<int> rcIn_channels;
    std::vector<int> rcOverride_channels;
};


class KeyboardInput
{
public:
    KeyboardInput(){
        // Initialize ncurses for keyboard input
        initscr();
        cbreak();
        noecho();
        keypad(stdscr, TRUE);
    }

    char get_keyboard_input(){
        // Check for keyboard input
        ch = getch();
        return ch;
    }

    ~KeyboardInput() {
        // End ncurses
        endwin();
    }

private:
    char ch;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_control_node");
    KeyboardControlNode keyboard_control_node;

    ros::Rate loop_rate(10); // Publish at 10 Hz

    while (ros::ok())
    {
        // Publish the RC override message
        keyboard_control_node.rcOverridePublish();
        // run callback
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
