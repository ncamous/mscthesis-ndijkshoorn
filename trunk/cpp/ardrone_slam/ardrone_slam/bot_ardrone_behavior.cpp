#include "bot_ardrone_behavior.h"
#include "bot_ardrone.h"
#include "global.h"
#include "reinflearn.h"
#include "opencv_helpers.h"

using namespace std;


bot_ardrone_behavior::bot_ardrone_behavior(bot_ardrone *b)
{
	this->bot = b;

	thread	= CreateThread(NULL, 0, start_behavior_thread, (void*) this, 0, NULL);
}


bot_ardrone_behavior::~bot_ardrone_behavior()
{
}


void bot_ardrone_behavior::map()
{
	//bot->recover(true);

	bot->get_slam()->off(SLAM_MODE_VISUALLOC);
	bot->get_slam()->on(SLAM_MODE_MAP);
	bot->set_slam(true);
	Sleep(500);

	bot->take_off();

	Sleep(3500); // safe?


	if(stop_behavior || !heightto(-1000.0f))
		return;

	if(stop_behavior || !flyto(800.0, 0.0f))
		return;

	if(stop_behavior || !flyto(800.0f, -1600.0f))
		return;

	if(stop_behavior || !flyto(-800.0f, -1600.0f))
		return;

	if(stop_behavior || !flyto(-800.0f, 0.0f))
		return;

	if(stop_behavior || !flyto(800.0, 0.0f))
		return;


	bot->get_slam()->off(SLAM_MODE_MAP);
	//bot->get_slam()->on(SLAM_MODE_VISUALLOC);


	while (1)
	{
		if(stop_behavior || !flyto(800.0f, -1600.0f))
			return;

		if(stop_behavior || !flyto(-800.0f, -1600.0f))
			return;

		if(stop_behavior || !flyto(-800.0f, 0.0f))
			return;

		if(stop_behavior || !flyto(800.0, 0.0f))
			return;
	}

	bot->control_reset();
	bot->control_update();
}


static DWORD WINAPI start_behavior_thread(void* Param)
{
	bot_ardrone_behavior *instance = (bot_ardrone_behavior*) Param; 
	
	while (1)
	{
		if (stop_behavior)
		{
			Sleep(100); // 100 ms
			continue;
		}

		// map
		instance->map();

		if (stop_behavior)
			instance->stop();
	}

	return 1;
}


void bot_ardrone_behavior::stop()
{
	bot->control_reset();
	bot->control_update();

	bot->set_slam(false);
}


bool bot_ardrone_behavior::flyto(float x, float y, float speed)
{
	bool reached = false;
	float *state;

	float d, dx, dy;
	float v, out_vx, out_vy;
	float cruise_vx, cruise_vy;
	float *out_v;
	float *cruise_v;
	bot_behavior mode_x = BOT_BEHAVIOR_NONE;
	bot_behavior mode_y = BOT_BEHAVIOR_NONE;
	bot_behavior *mode;

	Mat Mvel(3, 1, CV_32F);
	Mat Mor(3, 1, CV_32F);
	Mat Mrot(3, 3, CV_32F);
	Mor = 0.0f;
	out_vy = 0.0f;

	state = bot->slamcontroller->get_state();


	while (1)
	{

		dx = x - state[0];
		dy = y - state[1];

		for (int i = 0; i < 2; i++)
		{

			if (i == 0)
			{
				mode = &mode_x;
				d = x - state[0];
				v = state[3];
				out_v = &out_vx;
				cruise_v = &cruise_vx;
			}
			else
			{
				mode = &mode_y;
				d = y - state[1];
				v = state[4];
				out_v = &out_vy;
				cruise_v = &cruise_vy;
			}



			if (*mode == BOT_BEHAVIOR_NONE)
			{
				printf("M: NONE\n");

				if (abs(d) < 700.0f)
					*mode = BOT_BEHAVIOR_APPROACH;
				else
					*mode = BOT_BEHAVIOR_ACCEL;
			}


			if (*mode == BOT_BEHAVIOR_ACCEL)
			{
				printf("M: ACCEL\n");

				if (abs(d) < 500.0f)
				{
					*mode = BOT_BEHAVIOR_CRUISE;
				}
				else if ((d <= 0.0f && v <= -speed) || (d > 0.0f && v <= speed))
				{
					*mode = BOT_BEHAVIOR_CRUISE;
				}
				else
				{
					*out_v = (d < 0.0) ? -5000.0f : 5000.0f;
				}
			}


			if (*mode == BOT_BEHAVIOR_CRUISE)
			{
				printf("M: CRUISE (%f)\n", d);

				if (abs(d) < log(speed) * 60.0f)
				{
					*mode = BOT_BEHAVIOR_DEACCEL;
					*cruise_v = v;
				}
				else
				{
					*out_v = (d < 0.0) ? -speed : speed;
				}
			}


			if (*mode == BOT_BEHAVIOR_DEACCEL)
			{
				printf("M: DEACCEL (%f)\n", d);

				if (abs(v) < 30.0f || (*cruise_v >= 0.0f && v < 0.0f) || (*cruise_v < 0.0f && v > 0.0f))
				{
					printf("LOW VELOCITY (%f)!\n", v);
					*mode = BOT_BEHAVIOR_APPROACH;
				}
				else
				{
					*out_v = (*cruise_v > 0.0f) ? -(v*3.0f) : -(v*3.0f);
				}
			}


			if (*mode == BOT_BEHAVIOR_APPROACH)
			{
				printf("M: APPROACH (%f)\n", d);

				if (abs(dx) < 100.0f && abs(dy) < 100.0f)
				{
					printf("REACHED\n");
					bot->control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_LinearVelocity, 0.0f);
					bot->control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_LateralVelocity, 0.0f);
					bot->control_update();
					

					for (int k = 0; k < 30; k++)
					{
						dx = x - state[0];
						dy = y - state[1];
						printf("%f, %f\n", dx, dy);
						Sleep(100);
					}

					return true;
				}

				*out_v = d * 3.0f;
				//*out_v = 0.0f;
			}
		}



		if (out_vx < -5000.0f)
			out_vx = -5000.0f;
		if (out_vx > 5000.0f)
			out_vx = 5000.0f;
		if (out_vy < -5000.0f)
			out_vy = -5000.0f;
		if (out_vy > 5000.0f)
			out_vy = 5000.0f;


		Mvel.at<float>(0) = out_vx;
		Mvel.at<float>(1) = out_vy;
		Mvel.at<float>(2) = 0.0f;

		Mor.at<float>(2) = -state[11];
		cv::RotationMatrix3D(Mor, Mrot);

		Mvel = Mrot * Mvel;

		bot->control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_LinearVelocity, Mvel.at<float>(0) / 5000.0f);
		bot->control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_LateralVelocity, Mvel.at<float>(1) / 5000.0f);
		bot->control_update();

		if (stop_behavior)
			return false;

		Sleep(20);
	}

	return true;
}


bool bot_ardrone_behavior::heightto(float z)
{
	float pos[3];
	float dz;
	bool reached = false;

	Mat Mpos(3, 1, CV_32F);

	while (1)
	{
		bot->get_slam_pos(pos);

		dz = z - pos[2];

		if (abs(dz) < 200.0f)
		{
			bot->control_reset();
			bot->control_update();
			printf("reached alt %f!\n", z);
			break; 
		}

		float velZ = min(0.7f, (dz*dz) / 1000000.0f);

		if (dz > 0.0f)
			velZ = -velZ;

		bot->control_set(BOT_ARDRONE_Velocity, BOT_ARDRONE_AltitudeVelocity, velZ);
		bot->control_update();

		if (stop_behavior)
			return false;

		Sleep(50);
	}

	return true;
}