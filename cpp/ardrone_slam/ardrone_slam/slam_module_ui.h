#pragma once

#include "engine.h"

class slam;

class terrain3d;


class slam_module_ui
{
public:
	slam_module_ui(slam *controller);
	~slam_module_ui(void);
	void update();

private:
	void init();

	slam *controller;

	bool initialized;

	Engine *matlab;

	terrain3d *terrain;
};

