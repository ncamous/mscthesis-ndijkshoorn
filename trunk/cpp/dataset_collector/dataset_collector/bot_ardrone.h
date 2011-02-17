#pragma once

#include "botinterface.h"
#include "bot_ardrone_usarsim.h"

#define BOT_ARDRONE_AltitudeVelocity 0
#define BOT_ARDRONE_LinearVelocity 1
#define BOT_ARDRONE_LateralVelocity 2
#define BOT_ARDRONE_RotationalVelocity 3

#define BOT_ARDRONE_INTERFACE_USARSIM 0
#define BOT_ARDRONE_INTERFACE_ARDRONELIB 1


class bot_ardrone
{
public:
	bot_ardrone(int botinterface);
	~bot_ardrone(void);
	void control_set(int opt, float val);
	void control_update();
	void control_reset();
	void measurement_received();
	void cam_received(char *image, int bytes);

	bot_ardrone_usarsim *i;
	int TMP_img_nr;

private:
	float controls[4];
};

