#pragma once

#define VK_Q 81
#define VK_A 65

class bot_ardrone;

static bot_ardrone **keyboard_bot;
static int keyboard_nr_bots = 0;
static float keyboard_vel;

static void keyboard_set(int action, int type, bool increment);

class bot_ardrone_keyboard
{
public:
	bot_ardrone_keyboard(bot_ardrone **b, int nr_bots);
	~bot_ardrone_keyboard();
};