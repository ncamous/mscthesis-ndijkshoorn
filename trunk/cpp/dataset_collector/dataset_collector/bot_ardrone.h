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

#define BOT_ARDRONE_FRAME_BUFFER 30720


struct bot_ardrone_control {
	float time;
	float velocity[4];

	bot_ardrone_control();
};

struct bot_ardrone_measurement {
	float time;
	bool usarsim;
	//int type; // message type: actually dont want this

	/* usarsim only */
	int sensor; // dont want this: all sensors at one!
	float gt_loc[3];
	float gt_or[3];
	float gt_vel[3];

	int ctrl_state;		/*!< Flying state (landed, flying, hovering, etc.) defined in CTRL_STATES enum. */
	int altitude;		/*!< UAV's altitude in mm */
	int battery;		/*!< battery voltage filtered (mV) */

	/*!< UAV's pitch in milli-degrees: pitch, roll, yaw */
	float ins_or[3];

	/*!< UAV's estimated linear velocity: x, y, z */
	float ins_vel[3];

	bot_ardrone_measurement();
};

struct bot_ardrone_frame {
	float time;
	char *data;
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

