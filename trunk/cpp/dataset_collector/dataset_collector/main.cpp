#include "global.h"
#include "bot_ardrone.h"


int main(int argc, char *argv[])
{
	//bot_ardrone ardrone(BOT_ARDRONE_INTERFACE_USARSIM);
	bot_ardrone ardrone(BOT_ARDRONE_INTERFACE_ARDRONELIB);

	ardrone.control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_LateralVelocity, -1.0f);
	ardrone.control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_LinearVelocity, 1.0f);
	//ardrone.control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_RotationalVelocity, 0.01f);
	ardrone.control_update();

	float tmp;
	tmp = 0.0f;

	while (true) {
		Sleep(2000);
		if (tmp > 0.4f)
			tmp = -0.9f;
		else
			tmp = 0.9f;
		//ardrone.control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_AltitudeVelocity, tmp);
		//ardrone.control_update();
	}


	/**** PLAYBACK ****/
	/*
	bot_ardrone ardrone(BOT_ARDRONE_INTERFACE_NONE);
	ardrone.set_playback("001");
	*/


	/**** RECORD ****/
	/*
	bot_ardrone ardrone(BOT_ARDRONE_INTERFACE_USARSIM);

	ardrone.control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_LateralVelocity, -1.0f);
	ardrone.control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_LinearVelocity, 1.0f);
	//ardrone.control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_RotationalVelocity, 0.01f);
	ardrone.control_update();

	ardrone.set_record();

	Sleep(20000);
	*/
}