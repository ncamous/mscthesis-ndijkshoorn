#pragma once
#include "bot.h"

#define BOT_ARDRONE_AltitudeVelocity 0
#define BOT_ARDRONE_LinearVelocity 1
#define BOT_ARDRONE_LateralVelocity 2
#define BOT_ARDRONE_RotationalVelocity 3


class bot_ardrone :
	public bot
{
public:
	bot_ardrone(botinterface *i);
	~bot_ardrone(void);
	void bot_ardrone::set(int opt, float val);
	void bot_ardrone::update();

private:
	float controls[4];
};

