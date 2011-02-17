#include <Windows.h>
#include "global.h"
#include "bot_ardrone.h"

int main(int argc, char *argv[])
{
	bot_ardrone ardrone(BOT_ARDRONE_INTERFACE_USARSIM);

	ardrone.control_set(BOT_ARDRONE_AltitudeVelocity, (float) 0.3);
	ardrone.control_update();

	Sleep(20000);
}