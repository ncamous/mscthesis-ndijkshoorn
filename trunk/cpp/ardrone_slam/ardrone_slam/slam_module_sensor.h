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
	float accel_avg[3];

	void accel_compensate_gravity(cv::Mat& accel, cv::Mat& m_or);
	void update_elevation_map(int sonar_height);
	void get_sonar_state(cv::Mat& pos, cv::Mat& or);

	slam *controller;

	int counter;


	/* KF */
	cv::KalmanFilter *KF;
	cv::Mat *state;
	cv::Mat prev_state;

	cv::Mat measurement;
	cv::Mat measurementMatrix;
	cv::Mat measurementNoiseCov;

	cv::Mat measurement_or;
	cv::Mat measurement_accel;
	cv::Mat measurement_vel;

	float difftime;
	double prev_update;
	
	FILE *error_log;
};

