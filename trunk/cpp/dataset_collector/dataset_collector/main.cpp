#include "global.h"
#include "bot_ardrone.h"


int main(int argc, char *argv[])
{
	/*
	//bot_ardrone ardrone(BOT_ARDRONE_INTERFACE_USARSIM);
	bot_ardrone ardrone(BOT_ARDRONE_INTERFACE_ARDRONELIB);

	ardrone.control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_LateralVelocity, -1.0f);
	ardrone.control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_LinearVelocity, 1.0f);
	ardrone.control_update();

	Sleep(10000);
	*/


	/**** PLAYBACK ****/
	/*
	bot_ardrone ardrone(BOT_ARDRONE_INTERFACE_NONE);
	ardrone.set_playback("001");
	*/


	/**** RECORD ****/
	bot_ardrone ardrone(BOT_ARDRONE_INTERFACE_USARSIM /*BOT_ARDRONE_INTERFACE_ARDRONELIB*/);
	ardrone.set_record();
	Sleep(10000);
}