#include "global.h"
#include "bot_ardrone.h"
#include "bot_ardrone_keyboard.h"
#include "bot_ardrone_3dmouse.h"
#include "bot_ardrone_behavior.h"


/* global variables */
bool exit_application	= false;
bool stop_behavior		= false;


int main(int argc, char *argv[])
{
	HWND consoleWindow = GetConsoleWindow(); 
	MoveWindow(consoleWindow, 0, 0, 600, 400, false);


	int nr_bots = 0;
	bot_ardrone *bots[1];


	/* Real AR.Drone */
	bot_ardrone ardrone(0x01, BOT_ARDRONE_INTERFACE_ARDRONELIB, SLAM_MODE_VEL | SLAM_MODE_MAP);

	/* USARSim AR.Drone  */
	// deployment location set in bot_ardrone_usarsim.cpp (top)
	//bot_ardrone ardrone(0x00, BOT_ARDRONE_INTERFACE_USARSIM, SLAM_MODE_MAP | SLAM_MODE_VEL);

	/* No interface: playback dataset */
	//bot_ardrone ardrone(0x01, BOT_ARDRONE_INTERFACE_NONE, SLAM_MODE_VISUALMOTION | SLAM_MODE_MAP);


	bots[nr_bots++] = &ardrone;
 

	//ardrone.set_record();

	// when playing back, enable SLAM here
	//ardrone.set_slam(true);

	// playback databset folder
	//ardrone.set_playback("thesis-exp1-run13");

	/* USED FOR EXPERIMENTS
	printf("saving map\n");
	ardrone.get_slam()->map.visual_map.save_canvas();
	ardrone.get_slam()->map.elevation_map.save_map();
	ardrone.get_slam()->m_frame->descriptor_map_quality();
	*/

	/* Methods to control AR.Drone */
	//bot_ardrone_behavior autonomous(&ardrone);
	//bot_ardrone_3dmouse mouse(bots, nr_bots);
	bot_ardrone_keyboard kb(bots, nr_bots);

	return 0;
}