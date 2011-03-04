#include "global.h"
#include "bot_ardrone_ardronelib.h"
#include "bot_ardrone.h"
#include "ardrone_tool_win32.h"

using namespace std;


/* allocate and initialize static instance member (only allocate pointer) */
bot_ardrone_ardronelib *bot_ardrone_ardronelib::myinstance = 0;


bot_ardrone_ardronelib* bot_ardrone_ardronelib::instance()
{
	return myinstance;
}


void bot_ardrone_ardronelib_process_navdata(navdata_demo_t*)
{
	char *message = "Test";

	bot_ardrone_ardronelib::instance()->process_measurement(message, 5);
}


bot_ardrone_ardronelib::bot_ardrone_ardronelib(bot_ardrone *bot)
{
	myinstance = (bot_ardrone_ardronelib*) this;

	this->bot = bot;

	frame = new bot_ardrone_frame;

	ardronewin32();
}


bot_ardrone_ardronelib::~bot_ardrone_ardronelib(void)
{
}


void bot_ardrone_ardronelib::init(void)
{

}


void bot_ardrone_ardronelib::control_update(void *control)
{
	bot_ardrone_control *control_struct = (bot_ardrone_control*) control;
/*
	char msg[200];

	sprintf_s(msg, 200, "DRIVE {AltitudeVelocity %f} {LinearVelocity %f} {LateralVelocity %f} {RotationalVelocity %f} {Normalized false}\r\n",
		control_struct->velocity[BOT_ARDRONE_AltitudeVelocity],
		control_struct->velocity[BOT_ARDRONE_LinearVelocity],
		control_struct->velocity[BOT_ARDRONE_LateralVelocity],
		control_struct->velocity[BOT_ARDRONE_RotationalVelocity]);

	control_send(msg);
*/
}


void bot_ardrone_ardronelib::control_send(char *message)
{

}

void bot_ardrone_ardronelib::socket_callback(int id, char *message, int bytes)
{

}


void bot_ardrone_ardronelib::process_measurement(char *message, int bytes)
{
	printf("callback naar eigen app werkt!\n");
}


void bot_ardrone_ardronelib::process_frame(char *message, int bytes)
{
}