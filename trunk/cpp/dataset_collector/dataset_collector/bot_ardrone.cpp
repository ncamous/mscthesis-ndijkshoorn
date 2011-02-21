#include "global.h"
#include "bot_ardrone.h"
#include "bot_ardrone_usarsim.h"
#include <fstream>

using namespace std;

bot_ardrone::bot_ardrone(int botinterface)
{
	TMP_img_nr = 1;

	control_reset();

	switch (botinterface)
	{
		case BOT_ARDRONE_INTERFACE_USARSIM:
			i = new bot_ardrone_usarsim((bot_ardrone*) this);
			break;

		case BOT_ARDRONE_INTERFACE_ARDRONELIB:
			break;

		default:
			printf("BOT_ARDRONE: INTERFACE NOT FOUND\n");

	}

	i->init();
}


bot_ardrone::~bot_ardrone(void)
{
}


void bot_ardrone::control_set(int opt, float val)
{
	control.velocity[opt] = val;
}


void bot_ardrone::control_update()
{
	i->control_update(control);
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
	printf("battery: %i\n", m->battery);
}


void bot_ardrone::cam_received(bot_ardrone_frame *frame)
{
	char filename[20];
	sprintf_s(filename, 20, "img/%i.jpg", TMP_img_nr++);

	//printf("OK %i\n", TMP_img_nr);

	ofstream f(filename, ios::out | ios::binary);
	f.write(frame->data, frame->data_size);
	f.close();
}