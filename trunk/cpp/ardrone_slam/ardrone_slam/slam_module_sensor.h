#pragma once

#include "opencv2/core/types_c.h"
#include "opencv_ekf.h"
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
	void update_elevation_map(int sonar_height);
	void get_sonar_state(cv::Mat& pos, cv::Mat& or);
	void calibrate(bot_ardrone_measurement *m);
	float sqrt_s(float f);

	slam *controller;

	int counter;


	/* KF */
	cv::ExtendedKalmanFilter *EKF;
	cv::Mat *state;
	cv::Mat prev_state;

	cv::Mat measurement;

	cv::Mat measurement_or;
	cv::Mat measurement_accel;
	cv::Mat measurement_vel;

	float difftime;
	double prev_update;
	
	FILE *error_log;

	int calib_measurements;
	float accel_gravity;
	double accel_sum[3];
	float accel_avg[3];

	double yaw_sum;
	bool calibrated;
};

