#include "global.h"
#include "bot_ardrone.h"
#include "bot_ardrone_keyboard.h"


/* global variables */
bool exit_dataset_collector = false;


int main(int argc, char *argv[])
{
	//bot_ardrone ardrone(BOT_ARDRONE_INTERFACE_ARDRONELIB /*BOT_ARDRONE_INTERFACE_USARSIM*/);
	/*
	ardrone.control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_LateralVelocity, -1.0f);
	ardrone.control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_LinearVelocity, 1.0f);
	ardrone.control_update();
	*/


	/**** PLAYBACK ****/
	//bot_ardrone ardrone(BOT_ARDRONE_INTERFACE_NONE);
	//ardrone.set_playback("004");
	//Sleep(10000);



	/**** RECORD ****/
	int nr_bots = 0;
	bot_ardrone *bots[2];

	/* bot 1: REAL ARDRONE */
	//bot_ardrone ardrone(BOT_ARDRONE_INTERFACE_ARDRONELIB);
	//bots[nr_bots++] = &ardrone;
	//ardrone.set_record();


	/* bot 2: USARSim ARDRONE */
	bot_ardrone ardrone2(BOT_ARDRONE_INTERFACE_USARSIM);
	bots[nr_bots++] = &ardrone2;
	//ardrone2.set_record();


	bot_ardrone_keyboard kb(bots, nr_bots);



	Sleep(100);
	return 0;
}