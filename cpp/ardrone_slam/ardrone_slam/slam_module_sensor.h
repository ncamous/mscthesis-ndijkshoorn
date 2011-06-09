#pragma once

#include "opencv2/core/types_c.h"
#include "opencv2/features2d/features2d.hpp"

struct bot_ardrone_measurement;

class slam;


class slam_module_sensor
{
public:
	slam_module_sensor(slam *controller);
	~slam_module_sensor(void);
	void process(bot_ardrone_measurement *m);

private:
	slam *controller;
};

