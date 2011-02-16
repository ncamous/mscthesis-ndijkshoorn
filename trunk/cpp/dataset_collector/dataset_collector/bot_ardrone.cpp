#include "bot_ardrone.h"
#include "global.h"


bot_ardrone::bot_ardrone(botinterface *i) : bot(i)
{
	controls[BOT_ARDRONE_AltitudeVelocity] = 0.0;
	controls[BOT_ARDRONE_LinearVelocity] = 0.0;
	controls[BOT_ARDRONE_LateralVelocity] = 0.0;
	controls[BOT_ARDRONE_RotationalVelocity] = 0.0;

	i->control_send("INIT {ClassName USARBot.ARDrone} {Location 0.0,0.0,0.8}\r\n");
}


bot_ardrone::~bot_ardrone(void)
{
}


void bot_ardrone::set(int opt, float val)
{
	controls[opt] = val;

	update();
}


void bot_ardrone::update()
{
	char msg[200];
	sprintf_s(msg, 200, "DRIVE {AltitudeVelocity %f} {LinearVelocity %f} {LateralVelocity %f} {RotationalVelocity %f} {Normalized false}\r\n",
		controls[BOT_ARDRONE_AltitudeVelocity],
		controls[BOT_ARDRONE_LinearVelocity],
		controls[BOT_ARDRONE_LateralVelocity],
		controls[BOT_ARDRONE_RotationalVelocity]);

	printf("%s\n", msg);

	i->control_send(msg);
}