#pragma once

#include "botinterface.h"
#include "bot_ardrone_usarsim.h"
#include "bot_ardrone_ardronelib.h"
#include "bot_ardrone_recorder.h"
#include <time.h>

#define BOT_ARDRONE_EVENT_CONTROL 0
#define BOT_ARDRONE_EVENT_MEASUREMENT 1
#define BOT_ARDRONE_EVENT_FRAME 2

#define BOT_ARDRONE_Velocity 0
#define BOT_ARDRONE_AltitudeVelocity 0
#define BOT_ARDRONE_LinearVelocity 1
#define BOT_ARDRONE_LateralVelocity 2
#define BOT_ARDRONE_RotationalVelocity 3

#define BOT_ARDRONE_INTERFACE_NONE 0
#define BOT_ARDRONE_INTERFACE_USARSIM 1
#define BOT_ARDRONE_INTERFACE_ARDRONELIB 2

#define BOT_ARDRONE_MEASUREMENT_SEN 0
#define BOT_ARDRONE_MEASUREMENT_STA 1

#define BOT_ARDRONE_SENSOR_UNKNOW 0
#define BOT_ARDRONE_SENSOR_GT 1
#define BOT_ARDRONE_SENSOR_INS 2
#define BOT_ARDRONE_SENSOR_SONAR 3


struct bot_ardrone_control {
	float time;
	float velocity[4];
	// todo: final var for state?
	bool hover;
	bool landed;

	bot_ardrone_control();
};

struct bot_ardrone_measurement {
	float time;

	int altitude;		// UAV's altitude in mm
	int battery;		// Format: unknown?

	float ins_vel[3];	// UAV's estimated linear velocity: x, y, z
	float ins_or[3];	// UAV's pitch, roll, yaw

	/* USARSim only */
	bool usarsim;
	int type;			// usarsim message type: SEN, NFO, etc
	int sensor;			// temp, i prefer to receive all sensors in one socket message
	float gt_loc[3];
	float gt_or[3];

	bot_ardrone_measurement();
};

struct bot_ardrone_frame {
	float time;
	char *data;
	char *data_start;
	int data_size;
	int dest_size;
	char filename[25];

	bot_ardrone_frame();
};

class bot_ardrone
{
public:
	bot_ardrone(int botinterface);
	~bot_ardrone(void);
	void control_set(int type, int opt, float val);
	void control_update();
	void control_update(bot_ardrone_control *c);
	void control_reset();
	void take_off();
	void land();
	void measurement_received(bot_ardrone_measurement *m);
	void frame_received(bot_ardrone_frame *f);
	static float get_clock();
	void set_record();
	void set_playback(char *dataset);

	static clock_t start_clock;
	botinterface *i;
	bot_ardrone_control control;
	bot_ardrone_recorder *recorder;
	bool record, playback;
};

