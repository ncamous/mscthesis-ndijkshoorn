#include "global.h"
#include "bot_ardrone.h"


int main(int argc, char *argv[])
{
	
	
	/**** PLAYBACK ****/
	bot_ardrone ardrone(BOT_ARDRONE_INTERFACE_NONE);
	ardrone.set_playback("001");
	Sleep(10000);

	/**** RECORD ****/
	/*
	bot_ardrone ardrone(BOT_ARDRONE_INTERFACE_USARSIM);
	ardrone.set_record();

	ardrone.control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_AltitudeVelocity, (float) 0.3);
	ardrone.control_update();

	Sleep(2000);

	ardrone.control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_AltitudeVelocity, (float) 0.5);
	ardrone.control_update();

	Sleep(5000);
	*/
}