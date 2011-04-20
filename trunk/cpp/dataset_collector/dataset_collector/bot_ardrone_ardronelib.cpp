#include "global.h"
#include "bot_ardrone_ardronelib.h"
#include "bot_ardrone.h"
#include "ardrone_tool_win32.h"


extern "C" {
	HANDLE ardrone_ready;
}

using namespace std;


/* allocate and initialize static instance member (only allocate pointer) */
bot_ardrone_ardronelib *bot_ardrone_ardronelib::myinstance = 0;


static DWORD WINAPI start_ardrone_api_thread(void* Param)
{
	ardronewin32();
	return 0;
}


bot_ardrone_ardronelib* bot_ardrone_ardronelib::instance()
{
	return myinstance;
}


void bot_ardrone_ardronelib_process_navdata(navdata_unpacked_t *n)
{
	bot_ardrone_ardronelib::instance()->process_measurement(n);
}


void bot_ardrone_ardronelib_process_frame(unsigned char* rgbtexture, int w, int h)
{
	bot_ardrone_ardronelib::instance()->process_frame(rgbtexture, w, h);
}



bot_ardrone_ardronelib::bot_ardrone_ardronelib(bot_ardrone *bot)
{
	myinstance = (bot_ardrone_ardronelib*) this;

	this->bot = bot;

	frame = new bot_ardrone_frame;

	// temp
	m_counter = 1;

	// start thread
	printf("Connecting to ARDrone\n");
	DWORD ThreadID;
	ardrone_ready = CreateEvent(NULL, false, false, NULL);
	ardrone_thread = CreateThread(NULL, 0, start_ardrone_api_thread, (void*) this, 0, &ThreadID);

	// wait untill the Drone is completely ready (communication started)
	WaitForSingleObject(ardrone_ready, INFINITE);
}


bot_ardrone_ardronelib::~bot_ardrone_ardronelib(void)
{
}


void bot_ardrone_ardronelib::init(void)
{
}


void bot_ardrone_ardronelib::control_update(void *control)
{
	bot_ardrone_control *c = (bot_ardrone_control*) control;

	ardronewin32_progress(
		(c->state == BOT_ARDRONE_STATE_FLY)?1:0,
		c->velocity[BOT_ARDRONE_LateralVelocity],
		-c->velocity[BOT_ARDRONE_LinearVelocity],
		c->velocity[BOT_ARDRONE_AltitudeVelocity],
		c->velocity[BOT_ARDRONE_RotationalVelocity]
	);
}


void bot_ardrone_ardronelib::take_off()
{
	ardronewin32_take_off();
}


void bot_ardrone_ardronelib::land()
{
	ardronewin32_land();
}


void bot_ardrone_ardronelib::process_measurement(navdata_unpacked_t *n)
{
	bot_ardrone_measurement m;

	// print initial battery state
	if (bot->battery == NULL)
		printf("Battery: %i\n", n->navdata_demo.vbat_flying_percentage);

	// print low battery state
	bot->battery = n->navdata_demo.vbat_flying_percentage;
	if (bot->battery < 10)
		printf("Low battery: %i\n", bot->battery);

	m.altitude = n->navdata_demo.altitude;
	m.or[0] = n->navdata_demo.theta;
	m.or[1] = n->navdata_demo.phi;
	m.or[2] = n->navdata_demo.psi;

	// Accelerations are received in mg
	// They are displayed in g: conversion gain is 1/1000
	m.accel[0] = n->navdata_phys_measures.phys_accs[0]; // x-dir
	m.accel[1] = n->navdata_phys_measures.phys_accs[1]; // y-dir
	m.accel[2] = n->navdata_phys_measures.phys_accs[2] + 1000.0f; // z-dir, gravity = 1g */

	m.vel[0] = n->navdata_demo.vx;
	m.vel[1] = n->navdata_demo.vy;
	//m.ins_vel[2] = n->navdata_demo.vz; // is always zero
	m.vel[2] = n->navdata_altitude.altitude_vz;

	bot->measurement_received(&m);
}


void bot_ardrone_ardronelib::process_frame(unsigned char* rgbtexture, int w, int h)
{
	int bufpos, y;
	char *rgb_src = (char *)rgbtexture;

	// size check
	if (w*h*3 + 4 > BOT_ARDRONE_FRAME_BUFSIZE)
	{
		printf("ARDRONE FRAME TOO LARGE... SKIPPING FRAME (w: %i, h: %i\n", w, h);
		return;
	}

	frame->time = bot->get_clock(); // get clock time now

	// write width and height to first 4 bytes
	unsigned short w_bytes = htons(w);
	unsigned short h_bytes = htons(h);
	memcpy_s(frame->data, BOT_ARDRONE_FRAME_BUFSIZE, &w_bytes, 2);
	memcpy_s(frame->data + 2, BOT_ARDRONE_FRAME_BUFSIZE, &h_bytes, 2);

	bufpos = 4;

	for(y=0; y<h; y++)
	{
		rgb_src = (char *)rgbtexture + y*DRONE_VIDEO_MAX_WIDTH*3;
		memcpy_s(frame->data + bufpos, BOT_ARDRONE_FRAME_BUFSIZE - bufpos, rgb_src, w*3);
		bufpos += w*3;
	}

	frame->data_size = 4 + w*h*3;

	bot->frame_received(frame);
}