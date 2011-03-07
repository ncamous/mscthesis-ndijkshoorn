#include "global.h"
#include "bot_ardrone.h"
#include "bot_ardrone_usarsim.h"
#include "bot_ardrone_recorder.h"

using namespace std;

clock_t bot_ardrone::start_clock = 0;


bot_ardrone_control::bot_ardrone_control()
{
	memset(this, 0, sizeof(bot_ardrone_control));
}

bot_ardrone_measurement::bot_ardrone_measurement()
{
	memset(this, 0, sizeof(bot_ardrone_measurement));
	time = bot_ardrone::get_clock();
	usarsim = false;
}

bot_ardrone_frame::bot_ardrone_frame()
{
	memset(this, 0, sizeof(bot_ardrone_frame));
	time = bot_ardrone::get_clock();

	this->data = new char[USARSIM_FRAME_BUFSIZE];
	this->data_start = this->data; // important
}


bot_ardrone::bot_ardrone(int botinterface)
{
	start_clock = clock();
	i = NULL;
	recorder = NULL;
	record = false;
	playback = false;

	control_reset();

	switch (botinterface)
	{
		case BOT_ARDRONE_INTERFACE_USARSIM:
			i = new bot_ardrone_usarsim((bot_ardrone*) this);
			break;

		case BOT_ARDRONE_INTERFACE_ARDRONELIB:
			i = new bot_ardrone_ardronelib((bot_ardrone*) this);
			break;

		default:
			printf("ARDRONE: NO INTERFACE USED\n");
	}

	if (i != NULL)
		i->init();

	Sleep(100);
}


bot_ardrone::~bot_ardrone(void)
{
}


void bot_ardrone::control_set(int type, int opt, float val)
{
	switch(type)
	{
		case BOT_ARDRONE_Velocity:
			control.velocity[opt] = val;
			break;

		default:
			break;
	}
}


void bot_ardrone::control_update()
{
	control.time = get_clock();

	control_update(&control);
}


void bot_ardrone::control_update(bot_ardrone_control *c)
{
	if (PRINT_DEBUG)
		printf("%f - ARDRONE: control update!\n", c->time);

	if(record)
		recorder->record_control(&control);

	if (i != NULL)
		i->control_update((void*) &control);
}


void bot_ardrone::control_reset()
{
	control.velocity[BOT_ARDRONE_AltitudeVelocity] = 0.0;
	control.velocity[BOT_ARDRONE_LinearVelocity] = 0.0;
	control.velocity[BOT_ARDRONE_LateralVelocity] = 0.0;
	control.velocity[BOT_ARDRONE_RotationalVelocity] = 0.0;
}


void bot_ardrone::measurement_received(bot_ardrone_measurement *m)
{
	if (PRINT_DEBUG)
		printf("%f - ARDRONE: measurement received!\n", m->time);

	//if (record)
	//	recorder->record_measurement(m);
}


void bot_ardrone::frame_received(bot_ardrone_frame *f)
{
	if (PRINT_DEBUG)
		printf("%f - ARDRONE: frame received: %s!\n", f->time, f->filename);

	if (record)
		recorder->record_frame(f);
}


float bot_ardrone::get_clock()
{
	return (float) (clock() - start_clock) / CLOCKS_PER_SEC;
}


void bot_ardrone::set_record()
{
	if (recorder == NULL)
	{
		recorder = new bot_ardrone_recorder((bot_ardrone*) this);
		recorder->prepare_dataset();
		record = true;
	}
}


void bot_ardrone::set_playback(char *dataset)
{
	if (recorder == NULL)
	{
		recorder = new bot_ardrone_recorder((bot_ardrone*) this);
		recorder->playback(dataset);
		playback = true;
	}
}