#pragma once

#include <time.h>

class slam;

class terrain3d;


class slam_module_ui
{
public:
	slam_module_ui(slam *controller);
	~slam_module_ui(void);
	void update();
	void display_canvas();

private:
	void init();

	slam *controller;

	bool initialized;

	clock_t prev_update;
	clock_t prev_render;

	terrain3d *terrain;

	float pos[6];
};

