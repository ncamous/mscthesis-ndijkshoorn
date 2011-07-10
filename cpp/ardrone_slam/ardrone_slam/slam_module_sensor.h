#pragma once

#include "opencv2/core/types_c.h"
#include "opencv2/video/tracking.hpp"
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
	void accel_compensate_gravity(cv::Mat& accel, cv::Mat& m_or);
	void calculate_scale(bot_ardrone_measurement *m);

	slam *controller;

	clock_t prev_update;

	int counter;


	/* KF */
	cv::KalmanFilter *KF;
	cv::Mat *state;

	cv::Mat measurement;
	cv::Mat measurementMatrix;
	cv::Mat measurementNoiseCov;
};

