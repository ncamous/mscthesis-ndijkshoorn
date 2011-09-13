#include "global.h"
#include "bot_ardrone.h"
#include "bot_ardrone_keyboard.h"
#include "bot_ardrone_behavior.h"


/* global variables */
bool exit_application	= false;
bool stop_behavior		= false;


int main(int argc, char *argv[])
{
	HWND consoleWindow = GetConsoleWindow();
	MoveWindow(consoleWindow, 0, 0, 600, 400, false);


	int nr_bots = 0;
	bot_ardrone *bots[1];

	/* bot 1: USARSim ARDRONE */
	// deployment location set in bot_ardrone_usarsim.cpp (top)
	bot_ardrone ardrone(0x00, BOT_ARDRONE_INTERFACE_USARSIM, SLAM_MODE_MAP | /*SLAM_MODE_VISUALMOTION |*/ SLAM_MODE_ACCEL);
	//bot_ardrone ardrone(0x01, BOT_ARDRONE_INTERFACE_ARDRONELIB, SLAM_MODE_MAP | SLAM_MODE_VEL);
	//bot_ardrone ardrone(0x00, BOT_ARDRONE_INTERFACE_USARSIM, SLAM_MODE_MAP | SLAM_MODE_ACCEL);
	bots[nr_bots++] = &ardrone;

	//ardrone.set_record();


	Sleep(3000);

	//Sleep(1000);
	//ardrone.set_slam(true);


	bot_ardrone_behavior autonomous(&ardrone);


	bot_ardrone_keyboard kb(bots, nr_bots);


	Sleep(999999);
	return 0;
}