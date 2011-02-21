#pragma once

#include "botinterface.h"
#include "bot_ardrone_usarsim.h"

#define BOT_ARDRONE_AltitudeVelocity 0
#define BOT_ARDRONE_LinearVelocity 1
#define BOT_ARDRONE_LateralVelocity 2
#define BOT_ARDRONE_RotationalVelocity 3

#define BOT_ARDRONE_INTERFACE_USARSIM 0
#define BOT_ARDRONE_INTERFACE_ARDRONELIB 1

#define BOT_ARDRONE_MEASUREMENT_SEN 0
#define BOT_ARDRONE_MEASUREMENT_STA 0

struct bot_ardrone_control {
	float velocity[4];
};

struct bot_ardrone_measurement {
	int type;
	double groundtruth_loc[3];
	double groundtruth_or[3];
	int battery;

	bot_ardrone_measurement() : type(0),battery(0){}
};

struct bot_ardrone_frame {
	char header[4];
	char *data;
	int header_size;
	int data_size;
	int dest_size;

	bot_ardrone_frame() : header_size(0),data_size(0),dest_size(0){}
};

class bot_ardrone
{
public:
	bot_ardrone(int botinterface);
	~bot_ardrone(void);
	void control_set(int opt, float val);
	void control_update();
	void control_reset();
	void measurement_received(bot_ardrone_measurement *m);
	void cam_received(bot_ardrone_frame *frame);

	bot_ardrone_usarsim *i;
	bot_ardrone_control control;
	int TMP_img_nr;
};

