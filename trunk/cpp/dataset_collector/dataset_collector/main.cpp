#include "global.h"
#include "bot_ardrone.h"


int main(int argc, char *argv[])
{
	bot_ardrone ardrone(BOT_ARDRONE_INTERFACE_USARSIM);

	ardrone.control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_AltitudeVelocity, 0.1f);
	ardrone.control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_LinearVelocity, 0.4f);
	ardrone.control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_RotationalVelocity, 0.2f);
	ardrone.control_update();

	Sleep(200000);


	/**** PLAYBACK ****/
	/*
	bot_ardrone ardrone(BOT_ARDRONE_INTERFACE_NONE);
	ardrone.set_playback("009");
	//Sleep(10000)
	*/

	/**** RECORD ****/
	/*
	bot_ardrone ardrone(BOT_ARDRONE_INTERFACE_USARSIM);

	ardrone.control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_AltitudeVelocity, 0.4f);
	ardrone.control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_LinearVelocity, 0.4f);
	ardrone.control_update();
	ardrone.set_record();

	Sleep(200000);
	*/
}