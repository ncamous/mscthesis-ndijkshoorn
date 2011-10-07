#include "bot_ardrone_behavior.h"
#include "bot_ardrone.h"
#include "global.h"
#include "reinflearn.h"

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
	printf("Standby\n");
	Sleep(3000);

	bot->get_slam()->off(SLAM_MODE_VISUALLOC);
	bot->get_slam()->on(SLAM_MODE_MAP);
	bot->set_slam(true);
	Sleep(500);

	bot->take_off();

	Sleep(5000); // safe?

	bot->flyto_vel = 7000.0f;

	/*
	if(stop_behavior || !bot->heightto(-1500.0f))
		return;
	*/


	if(stop_behavior || !bot->heightto(-1000.0f))
		return;

	if(stop_behavior || !bot->flyto(800.0, 0.0f))
		return;

	if(stop_behavior || !bot->flyto(800.0f, -1600.0f))
		return;

	if(stop_behavior || !bot->flyto(-800.0f, -1600.0f))
		return;

	if(stop_behavior || !bot->flyto(-800.0f, 0.0f))
		return;

	if(stop_behavior || !bot->flyto(800.0, 0.0f))
		return;


	bot->get_slam()->off(SLAM_MODE_MAP);
	bot->get_slam()->on(SLAM_MODE_VISUALLOC);

	bot->flyto_vel = 5000.0f;

	while (1)
	{
		if(stop_behavior || !bot->flyto(800.0f, -1600.0f))
			return;

		if(stop_behavior || !bot->flyto(-800.0f, -1600.0f))
			return;

		if(stop_behavior || !bot->flyto(-800.0f, 0.0f))
			return;

		if(stop_behavior || !bot->flyto(800.0, 0.0f))
			return;
	}

	bot->control_reset();
	bot->control_update();
}


void bot_ardrone_behavior::forcefield()
{
	/*
	bot->get_slam()->off(SLAM_MODE_MAP);
	//bot->get_slam()->on(SLAM_MODE_VISUALLOC);
	reinflearn *rf = new reinflearn(bot);
	*/
	//delete[] rf;
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

		// forcefield
		if (!stop_behavior)
			instance->forcefield();

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