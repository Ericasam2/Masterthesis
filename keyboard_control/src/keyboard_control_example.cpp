#include <ros/ros.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <map>

// Map for speed keys
std::map<char, std::vector<float>> charToVectorMap
{
  {'w', {0, 0, 10, 0, 0, 0, 0, 0}},
  {'a', {-10, 0, 0, 0, 0, 0, 0, 0}},
  {'s', {0, 0, -10, 0, 0, 0, 0, 0}},
  {'d', {10, 0, 0, 0, 0, 0, 0, 0}}
};

// Reminder message
const char* msg = R"(

Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
        w     
   a    s    d


anything else : stop

CTRL-C to quit

)";

// Init variables
int channel1(1500);
int channel3(1500);
char key(' ');

// For non-blocking keyboard inputs
int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

int main(int argc, char** argv)
{
  // Init ROS node
  ros::init(argc, argv, "keyboard_control_node");
  ros::NodeHandle nh;

  // Init cmd_vel publisher
  ros::Publisher rc_override_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);

  // Create Override message
  mavros_msgs::OverrideRCIn rc_override_msg;

  // Initial RC Override data
  std::vector<int> rcOverride_channels{ 0,0,0,0,0,0,0,0 };

  printf("%s", rc_override_msg);
  printf("\rCurrent: channel1 %d\tchannel3 %d | Awaiting command...\r", channel1, channel3);

  while(true){

    // Get the pressed key
    key = getch();

    // If the key corresponds to a key in moveBindings
    if (charToVectorMap.count(key) == 1)
    {
      // Grab the direction data
      steer = charToVectorMap[ch][0];
      accel = charToVectorMap[ch][2];

      printf("\rCurrent: channel1 %d\tchannel3 %d |  Last command: %c   ", channel1, channel3, key);
    }
    // Otherwise, set the robot to stop
    else
    {
      accel = 0;

      // If ctrl-C (^C) was pressed, terminate the program
      if (key == '\x03')
      {
        printf("\n\n                 .     .\n              .  |\\-^-/|  .    \n             /| } O.=.O { |\\\n\n                 CH3EERS\n\n");
        break;
      }

      printf("\rCurrent: channel1 %d\tchannel3 %d |  Invalid command! %c", channel1, channel3, key);
    }

    // Update the RCOverride message
    rcOverride_channels[0] += steer;
    rcOverride_channels[2] += accel;
    rcOverride_channels[0] = std::min(1900, std::max(rcOverride_channels[0], 1100));
    rcOverride_channels[2] = std::min(1900, std::max(rcOverride_channels[2], 1100));
    for (int i = 0; i < 8; ++i){ 
      rc_override_msg.channels[i] = rcOverride_channels[i]; 
    }

    // Publish it and resolve any remaining callbacks
    rc_override_pub.publish(rc_override_msg);
    ros::spinOnce();
  }

  return 0;
}