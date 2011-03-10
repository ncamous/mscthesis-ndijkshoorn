#pragma once

#include <stdio.h>

#define PRINT_DEBUG false


/* USARSIM */
#define USARSIM_IP "127.0.0.1"
#define USARSIM_PORT 3000
#define UPIS_PORT 5003


/* ARDRONE */
#define BOT_ARDRONE_FRAME_BUFSIZE 80000		// at least: 176*144*3 + 4 bytes
#define USARIM_FRAME_USERAW true
#define USARSIM_FRAME_EXT "raw"

	/* USARSim */
	#define BOT_ARDRONE_USARSIM_FRAME_BLOCKSIZE 8000
	#define BOT_ARDONE_USARSIM_CONTROL_BUFSIZE 300
	#define BOT_ARDRONE_USARSIM_FRAME_REQDELAY 100

	/* keyboard */
	#define BOT_ARDRONE_KEYBOARD_VEL_MAX 1.0f
	#define BOT_ARDRONE_KEYBOARD_VEL_MIN -1.0f	