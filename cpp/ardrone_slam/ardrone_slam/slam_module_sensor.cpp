#include "global.h"
#include "slam_module_sensor.h"
#include "bot_ardrone.h"
#include "opencv_helpers.h"
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;


slam_module_sensor::slam_module_sensor(slam *controller):
	measurement(15, 1, CV_32F),

	measurement_or(3, 1, CV_32F),
	measurement_accel(3, 1, CV_32F),
	measurement_vel(3, 1, CV_32F),

	prev_state(15, 1, CV_32F)
{
	this->controller = controller;


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

	elevation_mode	= 0;
	elevation		= 0.0f;
}


slam_module_sensor::~slam_module_sensor(void)
{
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


void slam_module_sensor::process(bot_ardrone_measurement *m)
{
	bool use_accel	= controller->mode(SLAM_MODE_ACCEL);
	bool use_vel	= controller->mode(SLAM_MODE_VEL);



	/* calibrate accel sensor */
	if (use_accel && !calibrated)
	{
		calibrate(m);
		return;
	}



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
	//EKF->measurementMatrix = 0.0f;
	setIdentity(EKF->measurementMatrix);



	// Rk vector
	float MNC[15] = {
		300.0f, 300.0f, 300.0f,	// p: mm
		50.0f, 50.0f, 50.0f,	// v (mm/s)
		50.0f, 50.0f, 50.0f,	// a (mm/s2)
		0.02f, 0.02f, 0.02f,	// or (rad)
		0.01f, 0.01f, 0.01f		// ω (rad/s)
	};
	MatSetDiag(EKF->measurementNoiseCov, MNC);




	/* default measurements */
	memcpy_s(measurement.data, 60, EKF->statePre.data, 60);



	/* altitude measurement */
	/*
	if (m->state == 3)
	{
		if (elevation_mode == 0)
		{
			if (m->vel[2] > 50.0f)
			{
				elevation_mode		= 1;
				elevation_start		= (float) -m->altitude;
				//printf("going in elevation mode 1\n");
			}
			else if (elevation_mode == 0 && m->vel[2] < -50.0f)
			{
				elevation_mode		= 2;
				elevation_start		= (float) -m->altitude;
				//printf("going in elevation mode 1\n");
			}
		} else {

			if (elevation_mode == 1 && m->vel[2] < 0.0f)
			{
				elevation_mode		= 0;
				elevation			= elevation_start + (float) m->altitude;
				//printf("ending elevation 1!\n");
			}
			else if (elevation_mode == 2 && m->vel[2] > 0.0f)
			{
				elevation_mode		= 0;
				elevation			= elevation_start + (float) m->altitude;
				//printf("ending elevation 2!\n");
			}
		}
	}
	*/
		


	EKF->measurementNoiseCov.at<float>(2, 2) = 100.0f;
	measurement.at<float>(2) = (float) -m->altitude + elevation;
	//printf("alt: %f (%f)\n", (float) -m->altitude, elevation);



	/* orientation measurement */
	measurement_or.at<float>(0) = m->or[0] * MD_TO_RAD;
	measurement_or.at<float>(1) = m->or[1] * MD_TO_RAD;
	measurement_or.at<float>(2) = (m->or[2] - EKF->yaw_offset) * MD_TO_RAD;



	/* velocity measurement */
	if (use_vel)
	{
		memcpy_s(measurement_vel.data, 12, m->vel, 12);

		/*
		measurement_vel.at<float>(0) = -(18163.0f * (measurement_or.at<float>(1) * measurement_or.at<float>(1)) - 86.0f * measurement_or.at<float>(1));
		measurement_vel.at<float>(1) = (18163.0f * (measurement_or.at<float>(0) * measurement_or.at<float>(0)) - 86.0f * measurement_or.at<float>(0));
		measurement_vel.at<float>(2) = 0.0f;

		if (measurement_or.at<float>(1) < 0.0f)
			measurement_vel.at<float>(0) = -measurement_vel.at<float>(0);

		if (measurement_or.at<float>(0) < 0.0f)
			measurement_vel.at<float>(1) = -measurement_vel.at<float>(1);
		*/

		Mat Rw(3, 3, CV_32F);
		Mat measurement_or_yaw = measurement_or.clone();
		measurement_or_yaw.at<float>(0) = 0.0f;
		measurement_or_yaw.at<float>(1) = 0.0f;
		cv::RotationMatrix3D(measurement_or_yaw, Rw);
		measurement_vel		= Rw * measurement_vel;

		memcpy_s(&measurement.data[12], 12, measurement_vel.data, 12);
		EKF->measurementNoiseCov.at<float>(3, 3) = 100.0f;
		EKF->measurementNoiseCov.at<float>(4, 4) = 100.0f;
		EKF->measurementNoiseCov.at<float>(5, 5) = 100.0f;
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
		measurement_accel.at<float>(2) += 1000.0f;

		/*
		if (abs(measurement_accel.at<float>(2)) > 50.0f)
			printf("Large accel\n");
		else
			printf("No accel\n");
		*/

		measurement_accel *= MG_TO_MM2;

		//printf("Z-accel: %f\n", m->accel[2]);



		// be aware: measurement.data is uchar, so index 12 is float matrix index 12/4 = 3
		memcpy_s(&measurement.data[6 * 4], 12, measurement_accel.data, 12); // accel: this is the fastest method
		EKF->measurementNoiseCov.at<float>(6, 6) = 100.0f;
		EKF->measurementNoiseCov.at<float>(7, 7) = 100.0f;
		EKF->measurementNoiseCov.at<float>(8, 8) = 100.0f;


		/*
		calib_measurements++;
		accel_sum[0] += measurement_accel.at<float>(0);
		accel_sum[1] += measurement_accel.at<float>(1);
		accel_sum[2] += measurement_accel.at<float>(2);

		printf("avg: %f, %f, %f\n", accel_sum[0] / calib_measurements, accel_sum[1] / calib_measurements, accel_sum[2] / calib_measurements);
		*/
	}



	/* angular velocity */
#ifndef BOT_ARDRONE_USE_ONBOARD_OR
	memcpy_s(&measurement.data[48], 12, m->phys_gyros, 12); // accel: this is the fastest method
	EKF->measurementNoiseCov.at<float>(12, 12) = 0.00001f;
	EKF->measurementNoiseCov.at<float>(13, 13) = 0.00001f;
	EKF->measurementNoiseCov.at<float>(14, 14) = 0.00001f;
#endif




	/* correct */
	EKF->correct(measurement);



	/* directly inject attitude into state vector */
#ifdef BOT_ARDRONE_USE_ONBOARD_OR
	memcpy_s(&state->data[36], 12, measurement_or.data, 12);
#endif


	/* release KF */
	EKF->release();


	/* store current state (position) as previous state */
	memcpy_s(prev_state.data, 60, state->data, 60); // this is the fastest method
	/**/



	// check is mapping mode is on
	if (controller->mode(SLAM_MODE_MAP))
		update_elevation_map((int) elevation);


	//if (counter++ % 2 == 0)
	//{
		/*
		printf("reset state\n");

		state->at<float>(0) = 0.0f;
		state->at<float>(1) = 0.0f;
		//state->at<float>(2) = 0.0f;
		state->at<float>(3) = 0.0f;
		state->at<float>(4) = 0.0f;
		state->at<float>(5) = 0.0f;
		state->at<float>(6) = 0.0f;
		state->at<float>(7) = 0.0f;
		state->at<float>(8) = 0.0f;
		*/

		/*
		printf("error: %f, %f, %f\n", 
			abs(state->at<float>(0) - m->gt_loc[0]),
			abs(state->at<float>(1) - (m->gt_loc[1] - 1000.0f)),
			abs(state->at<float>(2) - (m->gt_loc[2] - 2496.0f))
			);
		*/

		fprintf(error_log, "%f,%f,%f,%f,%f\n",
			(float) m->time,
			measurement_accel.at<float>(2),
			state->at<float>(8),
			EKF->errorCovPost.at<float>(8, 8),
			(float) m->altitude
		);

		fflush(error_log);
	//}
}


void slam_module_sensor::accel_compensate_gravity(Mat& accel, cv::Mat& m_or)
{
	Mat Rg(3, 3, CV_32F);
	cv::RotationMatrix3D(m_or, Rg);
	Mat gravity = (Mat_<float>(3,1) << 0.0f, 0.0f, 1000.0f); // 1000mg in z-direction
	gravity = Rg * gravity;

	//dumpMatrix(gravity);
	//printf("\n");

	accel -= gravity;
}


void slam_module_sensor::update_elevation_map(int sonar_height)
{
	/* cast sonar beam on the 'world/object' based on the current position & attitude */
	float h;

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

	CalcLinePositionAtDistance(sonar_pos, sonar_normal, (double) sonar_height, hit);
	hit.at<float>(2) =  (float) sonar_height;
	h = hit.at<float>(2);

	if (abs(h) >= 50.0f)
	{
		controller->elevation_map.update(hit.at<float>(0), hit.at<float>(1), hit.at<float>(2), 10, 200);
	}
	else
	{
		float r_mm = (float) (tan((BOT_ARDRONE_SONAR_FOV / 180.0f) * M_PI) * sonar_height);

		controller->elevation_map.update(hit.at<float>(0), hit.at<float>(1), hit.at<float>(2), 20, r_mm);
	}
}


void slam_module_sensor::get_sonar_state(Mat& pos, Mat& or)
{
	pos.at<float>(0) = state->at<float>(0);
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