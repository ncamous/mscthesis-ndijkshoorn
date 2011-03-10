#include "global.h"
#include "bot_ardrone.h"
#include "bot_ardrone_keyboard.h"


int main(int argc, char *argv[])
{
	//bot_ardrone ardrone(BOT_ARDRONE_INTERFACE_ARDRONELIB /*BOT_ARDRONE_INTERFACE_USARSIM*/);
	/*
	ardrone.control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_LateralVelocity, -1.0f);
	ardrone.control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_LinearVelocity, 1.0f);
	ardrone.control_update();
	*/


	/**** PLAYBACK ****/
	/*
	bot_ardrone ardrone(BOT_ARDRONE_INTERFACE_NONE);
	ardrone.set_playback("001");
	*/


	/**** RECORD ****/
	//bot_ardrone ardrone(BOT_ARDRONE_INTERFACE_ARDRONELIB);
	bot_ardrone ardrone(BOT_ARDRONE_INTERFACE_USARSIM);
	//ardrone.set_record();
	bot_ardrone_keyboard kb(&ardrone);
	return 0;

	/*
	ardrone.take_off();
	Sleep(3000);
	ardrone.land();
	Sleep(30000);
	*/
}