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
	//bot_ardrone ardrone(0x00, BOT_ARDRONE_INTERFACE_USARSIM, /*SLAM_MODE_VISUAL |*/ SLAM_MODE_ACCEL);
	bot_ardrone ardrone(0x01, BOT_ARDRONE_INTERFACE_ARDRONELIB, /*SLAM_MODE_VISUAL |*/ SLAM_MODE_VEL);
	bots[nr_bots++] = &ardrone;

	ardrone.recover(true);
	printf("Standby\n");
	Sleep(3000);

	ardrone.set_slam(true);
	Sleep(500);
	ardrone.take_off();

	Sleep(5000); // safe?
	/*
	ardrone.flyto(1000.0f, 0.0f);
	ardrone.flyto(1500.0f, -1000.0f);
	ardrone.flyto(1000.0f, -1500.0f);
	ardrone.flyto(-1000.0f, -1500);
	ardrone.flyto(-1500.0f, -1000.0f);
	ardrone.flyto(-1000.0f, 0.0f);
	ardrone.flyto(0.0f, 0.0f);
	ardrone.control_reset();
	ardrone.control_update();
	ardrone.land();
	*/

	while (1)
	{
	ardrone.flyto(1000.0f, 0.0f);
	ardrone.flyto(1000.0f, -2000.0f);
	ardrone.flyto(-1000.0f, -2000.0f);
	ardrone.flyto(-1000.0, 0.0f);
	ardrone.flyto(0.0f, 0.0f);
	ardrone.control_reset();
	ardrone.control_update();
	printf("ROUND Done\n");
	Sleep(4000);
	}


	ardrone.land();


	bot_ardrone_keyboard kb(bots, nr_bots);



	Sleep(999999);
	return 0;
}