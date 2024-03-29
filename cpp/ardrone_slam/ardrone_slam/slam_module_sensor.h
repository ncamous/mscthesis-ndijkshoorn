#pragma once

#include "opencv2/core/types_c.h"
#include "opencv_ekf.h"
#include "opencv2/features2d/features2d.hpp"
#include <time.h>

struct bot_ardrone_measurement;

class slam;
class slam_map;

enum elevation_state { ELEVATION_STATE_NONE, ELEVATION_STATE_UP, ELEVATION_STATE_DOWN, ELEVATION_STATE_DISABLE };


class slam_module_sensor
{
public:
	slam_module_sensor(slam *controller);
	~slam_module_sensor(void);
	void process(bot_ardrone_measurement *m);

private:
	float process_altitude_elevation(float sonar_raw, float vel, float accel);
	void update_elevation_map(float elevation, float sonar_distance);
	void get_sonar_state(cv::Mat& pos, cv::Mat& or);
	void calibrate(bot_ardrone_measurement *m);
	float sqrt_s(float f);

	slam *controller;
	slam_map *map;

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

	cv::Mat world_plane;
	cv::Mat world_plane_normal;

	double yaw_sum;
	bool calibrated;
	elevation_state elevation_mode;
	float elevation;
	float elevation_start;
	float prev_altitude;
	float stable_altitude;
	clock_t elevation_time;
};

