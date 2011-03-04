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


void bot_ardrone_ardronelib_navdata_callback(navdata_demo_t *navdata)
{
	bot_ardrone_ardronelib::instance()->process_measurement(navdata);
}


void bot_ardrone_ardronelib_video_callback(unsigned char* rgbtexture, int w, int h)
{
	bot_ardrone_ardronelib::instance()->process_frame(rgbtexture, w, h);
}


bot_ardrone_ardronelib::bot_ardrone_ardronelib(bot_ardrone *bot)
{
	myinstance = (bot_ardrone_ardronelib*) this;

	this->bot = bot;

	//ardronewin32();
	DWORD ThreadID;
	ardrone_thread = CreateThread(NULL, 0, start_ardrone_api_thread, (void*) this, 0, &ThreadID);
	Sleep(5000);
}





bot_ardrone_ardronelib::~bot_ardrone_ardronelib(void)
{
}


static DWORD WINAPI start_ardrone_api_thread(void* Param)
{
	ardronewin32();

	return 0;
}


void bot_ardrone_ardronelib::init(void)
{
	// performed in ardronewin32 -> video_stage
	// ardrone_at_zap(ZAP_CHANNEL_VERT);
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


void bot_ardrone_ardronelib::process_measurement(navdata_demo_t *navdata)
{
	bot_ardrone_measurement m;

	// map parameters
	m.ctrl_state = navdata->ctrl_state;
	m.altitude = navdata->altitude;
	m.battery = navdata->vbat_flying_percentage;
	m.ins_or[0] = navdata->theta;
	m.ins_or[1] = navdata->phi;
	m.ins_or[2] = navdata->psi;
	m.ins_vel[0] = navdata->vx;
	m.ins_vel[1] = navdata->vy;
	m.ins_vel[2] = navdata->vz;

	bot->measurement_received(&m);
}


void bot_ardrone_ardronelib::process_frame(unsigned char* rgbtexture, int w, int h)
{
	frame = NULL;
	frame = new bot_ardrone_frame;

	printf("%i, %i\n", w, h);

	frame->data = (char*)rgbtexture;
	frame->data_size = w*h*3;

	bot->frame_received(frame);
}


// not needed here
void bot_ardrone_ardronelib::socket_callback(int id, char *message, int bytes)
{
}