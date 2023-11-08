
#include <ros/ros.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <mavros_msgs/OverrideRCIn.h>

#include <map>

// Map for speed keys
std::map<char, std::vector<float>> SpeedMap
{
  {'w', {0, 0, 1, 0, 0, 0, 0, 0}},
  {'a', {-1, 0, 0, 0, 0, 0, 0, 0}},
  {'s', {0, 0, -1, 0, 0, 0, 0, 0}},
  {'d', {1, 0, 0, 0, 0, 0, 0, 0}}
};

// Map for command
std::map<char, bool> CommandMap
{
  {',', true},
  {'.', false}
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
  // Init service client
  ros::ServiceClient client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");


  // Init the incremental variables
  int steer = 0;
  int accel = 0;

  // Create Override message
  mavros_msgs::OverrideRCIn rc_override_msg;

  // Initial RC Override data
  std::vector<int> rcOverride_channels{ 0,0,0,0,0,0,0,0 };

  // Sampling time
  int rate(20);
  ros::Rate loop_rate(rate);

  printf("\rCurrent: channel1 %d\tchannel3 %d | Awaiting command...\r", channel1, channel3);

  while(true){

    // Get the pressed key
    key = getch();

    // If the key corresponds to a key in moveBindings
    if (SpeedMap.count(key) == 1)
    {
      // set control magnitude
      int mag = 800 / 1 / rate;
      // Grab the direction data
      steer = SpeedMap[key][0];
      accel = SpeedMap[key][2];
      channel1 += mag * steer;
      channel3 += mag * accel;
      if (channel1 != 0) {channel1 = std::min(1900, std::max(channel1, 1100));}
      if (channel3 != 0) {channel3 = std::min(1900, std::max(channel3, 1100));}

      printf("\rCurrent: channel1 %d\tchannel3 %d |  Last command: %c   ", channel1, channel3, key);
    }
    else if (CommandMap.count(key) == 1)
    {
      mavros_msgs::CommandBool srv;
      srv.request.arming = CommandMap[key]; // Set the request data
      if (CommandMap[key]) { printf("\rCurrent: Arming  |  Last command: %c   ", key); }
      else { printf("\rCurrent: DisArming  |  Last command: %c   ", key); }
    }
    // Otherwise, set the robot to stop
    else
    {
      channel1 = 0;
      channel3 = 0;
      steer = 0;
      accel = 0;

      // If ctrl-C (^C) was pressed, terminate the program
      if (key == '\x03')
      {
        for (int i = 0; i < 8; ++i){ 
          rc_override_msg.channels[i] = 0; 
        }
	      rc_override_pub.publish(rc_override_msg);
        printf("\n\n                 .     .\n              .  |\\-^-/|  .    \n             /| } O.=.O { |\\\n\n                 CH3EERS\n\n");
        break;
      }

      printf("\rCurrent: channel1 %d\tchannel3 %d |  Invalid command! %c", channel1, channel3, key);
    }

    // Update the RCOverride message
    rcOverride_channels[0] = channel1;
    rcOverride_channels[2] = channel3;
    // Update the RCOverride messagenm
    for (int i = 0; i < 8; ++i){ 
      rc_override_msg.channels[i] = rcOverride_channels[i]; 
    }

    // Publish it and resolve any remaining callbacks
    rc_override_pub.publish(rc_override_msg);

    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
