#pragma once

#include "opencv2/core/types_c.h"
#include "opencv2/video/tracking.hpp"
#include "opencv2/features2d/features2d.hpp"

#define MG_TO_MM2 9.80665003f
#define MD_TO_RAD 1.745329252e-05f

struct bot_ardrone_measurement;

class slam;


class slam_module_sensor
{
public:
	slam_module_sensor(slam *controller);
	~slam_module_sensor(void);
	void process(bot_ardrone_measurement *m);

private:
	void accel_compensate_gravity(cv::Mat& accel, cv::Mat& m_or);
	void calculate_scale(bot_ardrone_measurement *m);

	slam *controller;

	clock_t prev_update;

	cv::KalmanFilter *KF;
	cv::Mat processNoise;
	cv::Mat measurement;
	cv::Mat *state;

	int counter;
};

