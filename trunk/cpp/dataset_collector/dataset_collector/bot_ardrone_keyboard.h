#pragma once

#define VK_Q 81
#define VK_A 65

class bot_ardrone;

static bot_ardrone *keyboard_bot;

class bot_ardrone_keyboard
{
public:
	bot_ardrone_keyboard(bot_ardrone *b);
	~bot_ardrone_keyboard();
};