#pragma once

#define PRINT_DEBUG false

/* USARSIM SETTINGS */
#define USARSIM_IP "127.0.0.1"
#define USARSIM_PORT 3000
#define UPIS_PORT 5003
#define USARSIM_CONTROL_BUFSIZE 300
#define USARSIM_FRAME_BUFSIZE 80000 /* large enoug for RAW: 176*144*3 */

/* ARDRONE SETTINGS */
#define BOT_ARDRONE_RECORDER_FRAME_EXT ".raw"
#define BOT_ARDRONE_USARSIM_USERAW true
#define BOT_ARDRONE_USARSIM_FRAME_REQDELAY 100

#include <stdio.h>