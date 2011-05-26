#include "global.h"
#include "bot_ardrone.h"
#include "bot_ardrone_usarsim.h"
#include "bot_ardrone_recorder.h"

using namespace std;

clock_t bot_ardrone::start_clock = 0;

#undef memset

bot_ardrone_control::bot_ardrone_control()
{
	memset(this, 0, sizeof(bot_ardrone_control));
	state = BOT_STATE_LANDED;
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
	this->data = new char[BOT_ARDRONBOT_EVENT_FRAME_BUFSIZE];
	data_start = this->data;
	usarsim = false;
}


bot_ardrone::bot_ardrone(int botinterface)
{
	start_clock = clock();
	i = NULL;
	i_id = botinterface;
	recorder = NULL;
	record = playback = false;
	battery = NULL;
	slam_state = false;

	estimated_compensation = 0.0f;
	max_vel = 0.0f;
	compensation_dir = -1;

	altitude = 0.0f;

	control_reset();

	/* INTERFACE */
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

	/* SLAM */
	slamcontroller = new slam();
	if (SLAM_ENABLED)
		slamcontroller->run();
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
			control.state = BOT_STATE_FLY;
			break;

		default:
			break;
	}
}


float bot_ardrone::control_get(int type, int opt)
{
	switch(type)
	{
		case BOT_ARDRONE_Velocity:
			return control.velocity[opt];

		default:
			return 0.0f;
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

	if (control.state == BOT_STATE_FLY)
		control.state = BOT_STATE_HOVER;
}


void bot_ardrone::control_compensate(bot_ardrone_measurement *m)
{
	altitude -= m->vel[2] * 0.05f;

	printf("Altitude: %f\n", altitude);

	return;

	double diffticks = ((double)clock() - compensation_timeout) / CLOCKS_PER_SEC;

	//if ((compensation_dir == 1 && m->vel[2] > 0.0f) || (compensation_dir == 2 && m->vel[2] < 0.0f))
		//max_vel += abs(m->vel[2]) * 0.05f;


	if (abs(m->vel[2]) > 40.0f)
	{
		if (m->vel[2] < 0.0f)
			control.velocity_compensate[BOT_ARDRONE_AltitudeVelocity] = (m->vel[2] / BOT_ARDRONE_CONTROL_VZ_MAX) * 1.5f;
		else
			control.velocity_compensate[BOT_ARDRONE_AltitudeVelocity] = (m->vel[2] / BOT_ARDRONE_CONTROL_VZ_MAX) * 4.0f;
		control_update();
	}
	else if (control.velocity_compensate[BOT_ARDRONE_AltitudeVelocity] != 0.0f)
	{
		control.velocity_compensate[BOT_ARDRONE_AltitudeVelocity] = 0.0f;
		control_update();
	}

	return;


	// only when BOT_STATE_FLY and no vertical speed set by the user
	if ((compensation_dir > -1 || (compensation_dir == -1 && abs(m->vel[2]) > 120.0f))
		&& control.state == BOT_STATE_FLY && control.velocity[BOT_ARDRONE_AltitudeVelocity] == 0.0f
		&& (estimated_compensation == 0.0f || estimated_compensation < max_vel)
		&& diffticks > 1.5)
	{

		printf("%f/%f\n", estimated_compensation, max_vel);

		if (compensation_dir == -1)
		{
			compensation_dir = m->vel[2] > 0.0f ? 1 : 2;
			max_vel += abs(m->vel[2]) * 0.05f;
		}

		float to_compensate = max_vel - estimated_compensation;

		if (compensation_dir == 1)
		{
			//control.velocity_compensate[BOT_ARDRONE_AltitudeVelocity] = min(1.0f, to_compensate / (BOT_ARDRONE_CONTROL_VZ_MAX * 0.05f));
			control.velocity_compensate[BOT_ARDRONE_AltitudeVelocity] = 0.5f;
			estimated_compensation += BOT_ARDRONE_CONTROL_VZ_MAX * control.velocity_compensate[BOT_ARDRONE_AltitudeVelocity] * 0.05f * 4.0f;
			printf("compensating down\n");
		}
		else
		{
			//control.velocity_compensate[BOT_ARDRONE_AltitudeVelocity] = min(-1.0f, -to_compensate / (BOT_ARDRONE_CONTROL_VZ_MAX * 0.05f));
			control.velocity_compensate[BOT_ARDRONE_AltitudeVelocity] = -1.0f;
			estimated_compensation += BOT_ARDRONE_CONTROL_VZ_MAX * abs(control.velocity_compensate[BOT_ARDRONE_AltitudeVelocity]) * 0.05f;
			printf("compensating up\n");
		}

		control_update();
	}

	// compensated
	else if (estimated_compensation >= max_vel && ((compensation_dir == 1 && m->vel[2] <= 0.0f) || (compensation_dir == 2 &&  m->vel[2] >= 0.0f)))
	{
		printf("compensation done\n");
		control.velocity_compensate[BOT_ARDRONE_AltitudeVelocity] = 0.0f;
		control_update();
		estimated_compensation = 0.0f;
		max_vel = 0.0f;
		compensation_timeout = clock();
		compensation_dir = -1;
	}
}


void bot_ardrone::take_off()
{
	i->take_off();
	control.state = BOT_STATE_HOVER;
}


void bot_ardrone::land()
{
	i->land();
	control.state = BOT_STATE_LANDED;
}


void bot_ardrone::recover(bool send)
{
	if (control.state == BOT_STATE_LANDED)
		i->recover(send);
}


void bot_ardrone::measurement_received(bot_ardrone_measurement *m)
{
	if (exit_dataset_collector)
		return;

	if (PRINT_DEBUG)
		printf("%f - ARDRONE: measurement received!\n", m->time);

	// time since last frame
	double diffticks = ((double)clock() - lastframe_time) / CLOCKS_PER_SEC;
	if (diffticks < BOT_ARDRONE_MIN_FRAME_INTERVAL)
		return;

	lastframe_time = clock();


	/* compensate the ARDrone's fixed altitude */
	//if (i_id == BOT_ARDRONE_INTERFACE_ARDRONELIB)
	//	control_compensate(m);

	if (record)
		recorder->record_measurement(m);

	if (slam_state)
	{
		if (SLAM_USE_QUEUE)
		{
			slam_queue_item queue_item = {MEASUREMENT, m};
			slamcontroller->slam_queue.push(queue_item);
			SetEvent(slamcontroller->slam_queue_pushed);
		}
		else
		{
			slamcontroller->process_measurement(m);
		}
	}
	else
	{
		printf("Altitude: %i\n", m->altitude);
	}
}


void bot_ardrone::frame_received(bot_ardrone_frame *f)
{
	if (exit_dataset_collector)
		return;

	if (PRINT_DEBUG)
		printf("%f - ARDRONE: frame received: %s!\n", f->time, f->filename);

	// time since last frame
	/*double diffticks = ((double)clock() - lastframe_time) / CLOCKS_PER_SEC;
	if (diffticks < BOT_ARDRONE_MIN_FRAME_INTERVAL)
	{
		printf("Skipping frame\n");
		return;
	}*/

	//lastframe_time = clock();

	if (record && BOT_ARDRONE_RECORD_FRAMES)
		recorder->record_frame(f);


	if (slam_state && /*slamcontroller->slam_queue.empty()*/ slamcontroller->slam_queue_frames == 0)
	{
		if (SLAM_USE_QUEUE)
		{
			slam_queue_item queue_item = {FRAME, f};
			slamcontroller->slam_queue.push(queue_item);
			slamcontroller->slam_queue_frames++;
			SetEvent(slamcontroller->slam_queue_pushed);
		}
		else
		{
			slamcontroller->process_frame(f);
		}
	}
}


double bot_ardrone::get_clock()
{
	return ((double)clock() - start_clock) / CLOCKS_PER_SEC;
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


void bot_ardrone::set_slam(bool state)
{
	if (!SLAM_ENABLED)
	{
		printf("Unable to set SLAM state, because SLAM_ENABLED is FALSE\n");
		return;
	}

	slam_state = state;
}