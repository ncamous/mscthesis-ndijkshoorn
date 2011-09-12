#pragma once
#include <Windows.h>

#define BOT_ARDRONE_BEHAVIOR_SLEEP 100

class bot_ardrone;

static DWORD WINAPI start_behavior_thread(void* Param);


class bot_ardrone_behavior
{
public:
	bot_ardrone_behavior(bot_ardrone *b);
	~bot_ardrone_behavior();
	void map();
	void forcefield();
	void stop();

private:
	bot_ardrone *bot;
	HANDLE thread;
};