#pragma once

#define VK_L 76
#define VK_J 74
#define VK_K 75
#define VK_I 73
#define VK_NL_MINUS 219
#define VK_NL_PLUS 191
#define VK_R 82
#define VK_M 77
#define VK_T 84
#define VK_Q 81
#define VK_A 65
#define VK_W 87
#define VK_S 83
#define VK_D 68

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