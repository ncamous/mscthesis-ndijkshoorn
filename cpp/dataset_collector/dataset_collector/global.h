#pragma once

#include <stdio.h>

#define PRINT_DEBUG false

/* SLAM */
#define SLAM_ENABLED true
#define SLAM_USE_QUEUE true // use a queue to store the controldata and sensor data


/* USARSIM */
#define USARSIM_IP "127.0.0.1"
#define USARSIM_PORT 3000
#define UPIS_PORT 5003


/* ARDRONE */
#define BOT_ARDRONE_FRAME_BUFSIZE 80000		// at least: 176*144*3 + 4 bytes
#define USARIM_FRAME_USERAW true
#define USARSIM_FRAME_EXT "raw"
#define BOT_ARDRONE_BATTERYLIFE 720 // 720s, 12 minutes
#define BOT_ARDRONE_RECORD_FRAMES false
#define BOT_ARDRONE_MIN_FRAME_INTERVAL 0.1

	/* USARSim */
	#define BOT_ARDRONE_USARSIM_FRAME_BLOCKSIZE 8000
	#define BOT_ARDONE_USARSIM_CONTROL_BUFSIZE 400
	#define BOT_ARDRONE_USARSIM_FRAME_REQDELAY 200 // at least 20 ms

	/* keyboard */
	#define BOT_ARDRONE_KEYBOARD_VEL 0.5f


extern bool exit_dataset_collector;