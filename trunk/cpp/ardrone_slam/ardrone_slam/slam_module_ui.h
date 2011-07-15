#pragma once



#include "engine.h"

class slam;


class slam_module_ui
{
public:
	slam_module_ui(slam *controller);
	~slam_module_ui(void);
	void update();
	void display_elevation_map();

private:
	void init_CV();

	slam *controller;

	bool CV_ready;

	Engine *matlab;
};

