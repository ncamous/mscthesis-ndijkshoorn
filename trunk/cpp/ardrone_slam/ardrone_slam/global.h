#pragma once

#include <stdio.h>

#define PRINT_DEBUG false


/* SLAM */
#define SLAM_USE_QUEUE true // use a queue to store the controldata and sensor data
#define SLAM_SURF_HESSIANTHRESHOLD 70.0
#define SLAM_ELEVATION_MAP_DEFAULT_SIZE 200 // 10m * 10m in each direction + 1 for center

#define SLAM_MODE_VISUALMOTION 0x01
#define SLAM_MODE_ACCEL 0x02
#define SLAM_MODE_VEL 0x04
#define SLAM_MODE_VISUALLOC 0x08
#define SLAM_MODE_MAP 0x10


/* USARSIM */
#define USARSIM_IP "127.0.0.1"
#define USARSIM_PORT 3000
#define UPIS_PORT 5003


/* ARDRONE */
#define BOT_ARDRONBOT_EVENT_FRAME_BUFSIZE 80000		// at least: 176*144*3 + 4 bytes
#define USARIM_FRAME_USERAW true
#define BOT_ARDRONE_RECORD_EXT "raw"
#define BOT_ARDRONE_BATTERYLIFE 720 // 720s, 12 minutes
#define BOT_ARDRONE_RECORD_FRAMES false
#define BOT_ARDRONE_MIN_MEASUREMENT_INTERVAL 0.01 // max 100 measurement/s
#define BOT_ARDRONE_CAM_RESOLUTION_W 176
#define BOT_ARDRONE_CAM_RESOLUTION_H 144
#define BOT_ARDRONE_CAM_FOV 32.0f // camera FOV / 2
#define BOT_ARDRONE_SONAR_FOV 10.0f // sonar angle FOV
#define BOT_ARDRONE_CONTROL_VZ_MAX 1000.0f // mm/s

	/* USARSim */
	#define BOT_ARDRONE_USARSIM_FRAME_BLOCKSIZE 4096
	#define BOT_ARDRONE_USARSIM_CONTROL_BUFSIZE 400
	#define BOT_ARDRONE_USARSIM_FRAME_REQDELAY 250 // at least 20 ms
	#define BOT_ARDRONE_USARSIM_FRAME_MODE 1 // 1: request new frame when SLAM queue empty, 2: fixed framerate

	/* keyboard */
	#define BOT_ARDRONE_KEYBOARD_VEL 0.06f


extern bool exit_application;
extern bool stop_behavior;


/* conversion */
#define MG_TO_MM2 9.80665003f
#define MD_TO_RAD 1.745329252e-05f

#ifndef M_PI
	#define M_PI 3.14159265358979323846
#endif