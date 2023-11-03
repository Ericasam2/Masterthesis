#include <ros/ros.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/RCIn.h>
#include <sensor_msgs/Imu.h>
#include <ncurses.h> // Include the ncurses library




int main(int argc, char **argv)
{
        // Initialize ncurses for keyboard input
        initscr();
        cbreak();
        noecho();
	// keypad(stdscr, TRUE);
        nodelay(stdscr, TRUE);
	while(1)
	{
		char ch = getch();
		if (ch != ERR) { 
			std::cout << "keyboard input: " << ch << std::endl; }
	}
	endwin();

    return 0;
}
