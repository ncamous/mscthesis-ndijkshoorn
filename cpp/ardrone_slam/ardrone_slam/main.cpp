#include "global.h"
#include "bot_ardrone.h"
#include "bot_ardrone_keyboard.h"


/* global variables */
bool exit_application = false;


int main(int argc, char *argv[])
{
	/**** PLAYBACK ****/
	/*
	bot_ardrone ardrone(BOT_ARDRONE_INTERFACE_NONE);
	ardrone.set_slam(true);
	ardrone.set_playback("001");
	Sleep(10000);
	*/


	/**** RECORD ****/
	int nr_bots = 0;
	bot_ardrone *bots[2];

	/* bot 1: USARSim ARDRONE */
	// deployment location set in bot_ardrone_usarsim.cpp (top)
	bot_ardrone ardrone(BOT_ARDRONE_INTERFACE_USARSIM);
	bots[nr_bots++] = &ardrone;
	//ardrone.set_slam(true);
	//ardrone.set_playback("001");
	//ardrone.set_record();


	/* bot 2: REAL ARDRONE */
	//bot_ardrone ardrone2(BOT_ARDRONE_INTERFACE_ARDRONELIB);
	//bots[nr_bots++] = &ardrone2;
	//ardrone2.set_record();


	bot_ardrone_keyboard kb(bots, nr_bots);



	Sleep(999999);
	return 0;
}