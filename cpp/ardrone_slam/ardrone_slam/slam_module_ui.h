#pragma once

class slam;


class slam_module_ui
{
public:
	slam_module_ui(slam *controller);
	~slam_module_ui(void);
	void update();

private:
	void init_CV();

	slam *controller;

	bool CV_ready;
};

