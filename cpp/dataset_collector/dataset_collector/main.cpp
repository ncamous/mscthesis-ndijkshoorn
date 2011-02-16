#include <stdlib.h>
#include <Windows.h>
#include "global.h"
#include "bot_ardrone.h"
#include "botinterface_usarsim.h"

int main(int argc, char *argv[])
{
	botinterface_usarsim i;
	bot_ardrone ardrone(&i);

	Sleep(3);

	ardrone.set(BOT_ARDRONE_AltitudeVelocity, (float) 0.3);

	Sleep(9999);
}