#include "global.h"
#include "bot_ardrone.h"
#include "bot_ardrone_keyboard.h"


/* global variables */
bool exit_application = false;


int main(int argc, char *argv[])
{
	HWND consoleWindow = GetConsoleWindow();
	MoveWindow(consoleWindow, 0, 0, 600, 400, false);


	/**** PLAYBACK ****/
	/*
	bot_ardrone ardrone(BOT_ARDRONE_INTERFACE_NONE);
	ardrone.set_slam(true);
	ardrone.set_playback("001");
	Sleep(10000);
	*/


	/**** RECORD ****/
	int nr_bots = 0;
	bot_ardrone *bots[1];

	/* bot 1: USARSim ARDRONE */
	// deployment location set in bot_ardrone_usarsim.cpp (top)
	bot_ardrone ardrone(0x00, BOT_ARDRONE_INTERFACE_USARSIM, /*SLAM_MODE_VISUAL |*/ SLAM_MODE_ACCEL);
	bots[nr_bots++] = &ardrone;


	printf("Standby\n");
	Sleep(5000);

	ardrone.set_slam(true);
	ardrone.flyto(3000.0f, -2000.0f);
	ardrone.control_reset();
	ardrone.control_update();


	/* bot 2: REAL ARDRONE */
	//bot_ardrone ardrone2(0x01, BOT_ARDRONE_INTERFACE_NONE, /*SLAM_MODE_VISUAL |*/ SLAM_MODE_VEL);
	//bots[nr_bots++] = &ardrone2;
	//ardrone2.set_record();


	//bot_ardrone_keyboard kb(bots, nr_bots);



	Sleep(999999);
	return 0;
}