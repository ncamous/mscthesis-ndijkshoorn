﻿#include "global.h"
#include "slam_module_sensor.h"
#include "bot_ardrone.h"
#include "opencv_helpers.h"
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;


#define ELEV_ACCEL_THRESHOLD 2400000.0f


slam_module_sensor::slam_module_sensor(slam *controller):
	measurement(15, 1, CV_32F),

	measurement_or(3, 1, CV_32F),
	measurement_accel(3, 1, CV_32F),
	measurement_vel(3, 1, CV_32F),

	prev_state(15, 1, CV_32F),

	world_plane(3, 1, CV_32F),
	world_plane_normal(3, 1, CV_32F)
{
	this->controller = controller;
	this->map = &controller->map;

	counter	= 0;


	/* KF */
	EKF = &controller->EKF;
	state = &EKF->statePost;

	prev_state = 0.0f;


	fopen_s (&error_log, "dataset/error_log.txt" , "w");

	calibrated = false;
	calib_measurements = 0;

	accel_avg[0] = accel_avg[1] = accel_avg[2] = 0.0f;
	accel_sum[0] = accel_sum[1] = accel_sum[2] = 0.0;

	yaw_sum = 0.0;

	/* world plane */
	world_plane = 0.0f;
	world_plane_normal = 0.0f;
	world_plane_normal.at<float>(2) = -1.0f;
	/**/

	elevation_mode	= ELEVATION_STATE_NONE;
	elevation		= 0.0f;
	stable_altitude = 0.0f;
	prev_altitude	= 0.0f;
}


slam_module_sensor::~slam_module_sensor(void)
{
}


void slam_module_sensor::process(bot_ardrone_measurement *m)
{
	bool use_accel	= controller->mode(SLAM_MODE_ACCEL);
	bool use_vel	= controller->mode(SLAM_MODE_VEL);
	bool use_vel_or	= controller->mode(SLAM_MODE_VEL_OR);
	bool use_elev	= controller->mode(SLAM_MODE_ELEVMAP);



	/* calibrate accel sensor */
	/*
	if (use_accel && !calibrated)
	{
		calibrate(m);
		return;
	}
	*/

	//if (m->altitude > 1300)
	//	m->altitude = 200;



	/* set initial position: module_frame is not used before position is known */
	if (!EKF->running)
	{
		state->at<float>(2) = (float) -m->altitude; // write initial height directly into state vector
		EKF->start(m->time - 0.00001, m->or[2]);
	}
	else if (m->time <= EKF->last_update)
	{
		return;
	}



	/* lock KF */
	EKF->lock();



	/* update transition matrix */
	difftime = (float) EKF->difftime(m->time);
	controller->update_transition_matrix(difftime);



	/* predict */
	EKF->predict();



	// H vector
	setIdentity(EKF->measurementMatrix);



	// Rk vector
	float MNC[15] = {
		300.0f, 300.0f, 200.0f,	// p: mm
		50.0f, 50.0f, 50.0f,	// v (mm/s)
		50.0f, 50.0f, 50.0f,	// a (mm/s2)
		0.02f, 0.02f, 0.02f,	// or (rad)
		1.0f, 1.0f, 0.3f		// ω (rad/s)
	};
	MatSetDiag(EKF->measurementNoiseCov, MNC);




	/* default measurements */
	memcpy_s(measurement.data, 60, EKF->statePre.data, 60);


	measurement.at<float>(12) = (float) m->altitude; // alt, also used for behavior
	/* altitude measurement */
	if (use_elev)
	{
		measurement.at<float>(13) = (measurement.at<float>(12) - state->at<float>(12)) / difftime; // vel
		measurement.at<float>(14) = (measurement.at<float>(13) - state->at<float>(13)) / difftime; // accel
	}

	if (use_elev && m->state == 3)
	{
		// use filtered sonar distance
		measurement.at<float>(2) = process_altitude_elevation(state->at<float>(12), state->at<float>(13), state->at<float>(14));
	}
	else
	{
		// do not use elevation detection
		measurement.at<float>(2) = (float) -m->altitude + elevation;
	}



	/* orientation measurement */
	measurement_or.at<float>(0) = m->or[0] * MD_TO_RAD;
	measurement_or.at<float>(1) = m->or[1] * MD_TO_RAD;
	measurement_or.at<float>(2) = (m->or[2] - EKF->yaw_offset) * MD_TO_RAD;



	/* velocity measurement */
	if (use_vel_or)
	{
		measurement_vel = 0.0f;

		double coefficients[5];
		coefficients[4] = 3435866.69565783;
		coefficients[3] = -2057491.4266002;
		coefficients[2] = 295518.169045423;
		coefficients[1] = 5593.84438482671;
		coefficients[0] = 16.1915553499124;

		for (int i = 0; i < 2; i++)
		{
			for (int j = 4; j >= 0; j--)
				measurement_vel.at<float>(1-i) += (float) (coefficients[j] * pow((double) abs(measurement_or.at<float>(i)), j));
		}


		if (measurement_or.at<float>(1) > 0.0f)
			measurement_vel.at<float>(0) = -measurement_vel.at<float>(0);

		if (measurement_or.at<float>(0) < 0.0f)
			measurement_vel.at<float>(1) = -measurement_vel.at<float>(1);

		Mat Rw(3, 3, CV_32F);
		Mat measurement_or_yaw = measurement_or.clone();
		measurement_or_yaw.at<float>(0) = 0.0f;
		measurement_or_yaw.at<float>(1) = 0.0f;
		cv::RotationMatrix3D(measurement_or_yaw, Rw);
		measurement_vel		= Rw * measurement_vel;

		memcpy_s(&measurement.data[12], 8, measurement_vel.data, 8);
		EKF->measurementNoiseCov.at<float>(3, 3) = 7000.0f;
		EKF->measurementNoiseCov.at<float>(4, 4) = 7000.0f;
		//EKF->measurementNoiseCov.at<float>(5, 5) = 50000.0f;
	}



	if (use_vel)
	{
		memcpy_s(measurement_vel.data, 12, m->vel, 12);

		Mat Rw(3, 3, CV_32F);
		Mat measurement_or_yaw = measurement_or.clone();
		measurement_or_yaw.at<float>(0) = 0.0f;
		measurement_or_yaw.at<float>(1) = 0.0f;
		cv::RotationMatrix3D(measurement_or_yaw, Rw);
		measurement_vel		= Rw * measurement_vel;

		memcpy_s(&measurement.data[12], 12, measurement_vel.data, 12);
		EKF->measurementNoiseCov.at<float>(3, 3) = 10.0f;
		EKF->measurementNoiseCov.at<float>(4, 4) = 10.0f;
		EKF->measurementNoiseCov.at<float>(5, 5) = 10.0f;
	}



	/* acceleration measurement */
	if (use_accel)
	{
		measurement_accel.at<float>(0) = (m->accel[0] - accel_avg[0]);
		measurement_accel.at<float>(1) = (m->accel[1] - accel_avg[1]);
		measurement_accel.at<float>(2) = (m->accel[2] - accel_avg[2]);

		Mat Rw(3, 3, CV_32F);
		cv::RotationMatrix3D(measurement_or, Rw);
		measurement_accel	= Rw * measurement_accel;

		// gravity
		if (m->bot_id != 0x00)
			measurement_accel.at<float>(2) += 1000.0f;

		measurement_accel *= MG_TO_MM2;
		//dumpMatrix(measurement_accel);

		// be aware: measurement.data is uchar, so index 12 is float matrix index 12/4 = 3
		memcpy_s(&measurement.data[6 * 4], 12, measurement_accel.data, 12); // accel: this is the fastest method
		EKF->measurementNoiseCov.at<float>(6, 6) = 1.0f;
		EKF->measurementNoiseCov.at<float>(7, 7) = 1.0f;
		EKF->measurementNoiseCov.at<float>(8, 8) = 1.0f;
	}



	/* correct */
	EKF->correct(measurement);



	/* directly inject attitude into state vector */
#ifdef BOT_ARDRONE_USE_ONBOARD_OR
	memcpy_s(&state ->data[36], 12, measurement_or.data, 12);
#endif


	/* release KF */
	EKF->release();


	/* store current state (position) as previous state */
	memcpy_s(prev_state.data, 60, state->data, 60); // this is the fastest method



	// check is mapping mode is on
	if (use_elev)
		update_elevation_map(elevation, state->at<float>(12)); // return sonar measured distance


	//if (counter++ % 50 == 0)
		//printf("Elevation: %f\n", elevation);

	if (counter++ % 5 == 0)
	{
		//printf("Elevation: %f\n", elevation);

		/*
		printf("error: %f, %f, %f\n", 
			abs(state->at<float>(0) - m->gt_loc[0]),
			abs(state->at<float>(1) - (m->gt_loc[1] - 1000.0f)),
			abs(state->at<float>(2) - (m->gt_loc[2] - 2496.0f))
			);
		*/

		fprintf(error_log, "%f,%f,%f,%f,%f\n",
			(float) m->time_pc,
			//GetTickCount(),
			state->at<float>(0),
			state->at<float>(1),
			/*
			(float) m->altitude,
			(float) state->at<float>(12),
			(float) state->at<float>(13),
			(float) state->at<float>(14),
			-elevation
			*/
			(float) m->gt_loc[0],
			(float) m->gt_loc[1]
			/*
			(float) m->altitude,
			(float) m->accel[0]
			*/
			/*
			state->at<float>(0),
			state->at<float>(1),
			*/
			/*
			state->at<float>(12),
			state->at<float>(13),
			state->at<float>(14)
			*/
		);

		//if (counter % 5 == 0)
			fflush(error_log);
			//printf("flush\n");
	}
}


float slam_module_sensor::process_altitude_elevation(float sonar_raw, float vel, float accel)
{
	float delta = sonar_raw - prev_altitude;
	double dt;
	prev_altitude = sonar_raw;



	switch (elevation_mode)
	{
	case ELEVATION_STATE_NONE:
		//if (abs(accel) < 8000.0f)
		if (abs(vel) < 500.0f)
			stable_altitude = sonar_raw;

		if (accel < -ELEV_ACCEL_THRESHOLD)
		//if (vel < -1000.0f)
		{
			printf("ELEVATION MODE: UP\n");
			elevation_mode	= ELEVATION_STATE_UP;
			elevation_start	= stable_altitude;
			elevation_time	= clock();
			return stable_altitude;
		}
		else if (accel > ELEV_ACCEL_THRESHOLD)
		//else if (vel > 1000.0f)
		{
			printf("ELEVATION MODE: DOWN (%f)\n", delta);
			elevation_mode	= ELEVATION_STATE_DOWN;
			elevation_start	= stable_altitude;
			elevation_time	= clock();
			return stable_altitude;
		}
		else
		{
			return -sonar_raw + elevation;
		}
		break;

	case ELEVATION_STATE_UP:
		dt = ((double) (clock() - elevation_time)) / CLOCKS_PER_SEC;

		if (accel > 0.0f || dt >= 0.6)
		//if (vel >= 0.0f || dt >= 0.3)
		{
			elevation_mode	= ELEVATION_STATE_NONE;

			float delta = elevation_start - prev_altitude;
			if (delta < 0.0f)
				delta = 0.0f;

			//elevation		-= elevation_start - prev_altitude;
			elevation		-= delta;
			if (elevation > 0.0f)
				elevation = 0.0f;
			stable_altitude	= sonar_raw;
			printf("ELEVATION MODE: NONE (%f)\n", delta);
			return -sonar_raw + elevation;
		}
		break;

	case ELEVATION_STATE_DOWN:
		dt = ((double) (clock() - elevation_time)) / CLOCKS_PER_SEC;

		if (accel < 0.0f || dt >= 0.6)
		//if (vel <= 0.0f || dt >= 0.3)
		{
			elevation_mode	= ELEVATION_STATE_NONE;

			float delta = elevation_start - prev_altitude;
			if (delta > 0.0f)
				delta = 0.0f;

			//elevation		-= elevation_start - prev_altitude;
			elevation		-= delta;
			if (elevation > 0.0f)
				elevation = 0.0f;
			stable_altitude	= sonar_raw;
			printf("ELEVATION MODE: NONE (%f)\n", -sonar_raw, elevation, -sonar_raw + elevation);
			return -sonar_raw + elevation;
		}
		break;

	}

	return state->at<float>(2); // -sonar_raw + elevation; //
}


void slam_module_sensor::update_elevation_map(float elevation, float sonar_distance)
{
	/* cast sonar beam on the 'world/object' based on the current position & attitude */
	Mat sonar_pos(3, 1, CV_32F);
	Mat sonar_or(3, 1, CV_32F);

	sonar_or.at<float>(0) = 0.0f;
	sonar_or.at<float>(1) = 0.0f;
	sonar_or.at<float>(2) = 0.0f;

	Mat sonar_normal(3, 1, CV_32F);
	Mat sonar_rot(3, 3, CV_32F);
	Mat hit(3, 1, CV_32F);

	get_sonar_state(sonar_pos, sonar_or);

	sonar_normal.at<float>(0) = 0.0f;
	sonar_normal.at<float>(1) = 0.0f;
	sonar_normal.at<float>(2) = 1.0f; // pointing down

	/* more efficient method to extract line normal from vehicle attitide? */
	cv::RotationMatrix3D(sonar_or, sonar_rot);
	sonar_normal = sonar_rot * sonar_normal;
	cv::normalize(sonar_normal, sonar_normal); // to normal vector

	//CalcLinePositionAtDistance(sonar_pos, sonar_normal, (double) sonar_distance, hit);
	CalcLinePlaneIntersection(world_plane, world_plane_normal, sonar_pos, sonar_normal, hit);

	hit = sonar_pos;

	if (abs(elevation) >= 50.0f)
	{
		
		map->elevation_map.update(hit.at<float>(0), hit.at<float>(1), elevation, 1, 70);
	}
	else
	{
		float r_mm = (float) (tan((BOT_ARDRONE_SONAR_FOV / 180.0f) * M_PI) * sonar_distance);
		//float r_mm = (float) (tan((BOT_ARDRONE_SONAR_FOV / 180.0f) * M_PI) * state->at<float>(2));
		//r_mm *= 1.8f;
		//printf("flatten: %f\n", r_mm);
		map->elevation_map.update(hit.at<float>(0), hit.at<float>(1), 0.0, 2, r_mm);
	}
}


void slam_module_sensor::calibrate(bot_ardrone_measurement *m)
{
	if (++calib_measurements > BOT_ARDRONE_CALIB_MEASUREMENTS)
	{
		printf("Calibration complete: %f, %f, %f\n", accel_avg[0], accel_avg[1], accel_avg[2]);

		accel_sum[0] = accel_sum[1] = accel_sum[2] = 0.0f;
		calib_measurements = 0;

		calibrated = true;
		return;
	}

	measurement_or.at<float>(0) = m->or[0] * MD_TO_RAD;
	measurement_or.at<float>(1) = m->or[1] * MD_TO_RAD;
	//measurement_or.at<float>(2) = m->or[2] * MD_TO_RAD;
	measurement_or.at<float>(2) = 0.0f;

	//measurement_or = 0.0f;

	measurement_accel.at<float>(0) = m->accel[0];
	measurement_accel.at<float>(1) = m->accel[1];
	measurement_accel.at<float>(2) = m->accel[2];

	Mat Rw(3, 3, CV_32F);
	cv::RotationMatrix3D(measurement_or, Rw);
	measurement_accel	= Rw * measurement_accel;
	measurement_accel.at<float>(2) += 1000.0f;

	if (calib_measurements <= BOT_ARDRONE_CALIB_MEASUREMENTS)
	{
		accel_sum[0] += measurement_accel.at<float>(0);
		accel_sum[1] += measurement_accel.at<float>(1);
		accel_sum[2] += measurement_accel.at<float>(2);

		yaw_sum += m->or[2];
	}

	if (calib_measurements == BOT_ARDRONE_CALIB_MEASUREMENTS)
	{
		accel_avg[0] = (float) (accel_sum[0] / (double) BOT_ARDRONE_CALIB_MEASUREMENTS);
		accel_avg[1] = (float) (accel_sum[1] / (double) BOT_ARDRONE_CALIB_MEASUREMENTS);
		accel_avg[2] = (float) (accel_sum[2] / (double) BOT_ARDRONE_CALIB_MEASUREMENTS);

		EKF->yaw_offset = (float) (yaw_sum / (double) BOT_ARDRONE_CALIB_MEASUREMENTS); // AR.Drone's start Z orientation is always 0.0
	}
}


void slam_module_sensor::get_sonar_state(Mat& pos, Mat& or)
{
	pos.at<float>(0) = state->at<float>(0) + 0.0f;
	pos.at<float>(1) = state->at<float>(1);
	pos.at<float>(2) = state->at<float>(2);

	or.at<float>(0) = state->at<float>(9);
	or.at<float>(1) = state->at<float>(10);
	or.at<float>(2) = state->at<float>(11);
}


float slam_module_sensor::sqrt_s(float f)
{
	if (f < 0.0f)
		return sqrt(-f);
	else
		return sqrt(f);
}