#include "global.h"
#include "slam_module_sensor.h"
#include "bot_ardrone.h"
#include "opencv_helpers.h"
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;


slam_module_sensor::slam_module_sensor(slam *controller):
	measurement(9, 1, CV_32F),

	measurement_or(3, 1, CV_32F),
	measurement_accel(3, 1, CV_32F),
	measurement_vel(3, 1, CV_32F),

	prev_state(12, 1, CV_32F)
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
}


slam_module_sensor::~slam_module_sensor(void)
{
}


void slam_module_sensor::calibrate(bot_ardrone_measurement *m)
{
	if (++calib_measurements > BOT_ARDRONE_CALIB_MEASUREMENTS)
	{
		printf("Calibration complete: %f, %f, %f\n", accel_avg[0], accel_avg[1], accel_avg[2]);
		calibrated = true;
		return;
	}

	measurement_or.at<float>(0) = m->or[0] * MD_TO_RAD;
	measurement_or.at<float>(1) = m->or[1] * MD_TO_RAD;
	measurement_or.at<float>(2) = m->or[2] * MD_TO_RAD;
	measurement_or.at<float>(2) = 0.0f;

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
		EKF->last_measurement_time = m->time - 0.0001;
		EKF->running = true; // KF is initialized. Now that the initial height of the vehicle is known, the frame module can start working
		EKF->yaw_offset = m->or[2]; // TMP
	}
	else if (m->time <= EKF->last_measurement_time)
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
	/*
	float MM[9] = {
		1.0f, 1.0f, 1.0f,		// p
		1.0f, 1.0f, 1.0f,		// v
		1.0f, 1.0f, 1.0f		// a
	};
	MatSetDiag(EKF->measurementMatrix, MM);
	*/
	setIdentity(EKF->measurementMatrix);

	// Rk vector
	float MNC[9] = {
		300.0f, 300.0f, 300.0f,	// p: mm
		50.0f, 50.0f, 50.0f,	// v (mm/s)
		50.0f, 50.0f, 50.0f		// a (mm/s2)
	};
	MatSetDiag(EKF->measurementNoiseCov, MNC);




	/* default measurements */
	memcpy_s(measurement.data, 36, EKF->statePre.data, 36);



	/* altitude measurement */
	EKF->measurementNoiseCov.at<float>(2, 2) = 100.0f;
	measurement.at<float>(2) = (float) -m->altitude;



	/* orientation measurement */
	measurement_or.at<float>(0) = m->or[0] * MD_TO_RAD;
	measurement_or.at<float>(1) = m->or[1] * MD_TO_RAD;
	measurement_or.at<float>(2) = (m->or[2] - EKF->yaw_offset) * MD_TO_RAD;



	/* velocity measurement */
	if (use_vel)
	{
		//for(int i = 3; i < 6; i++)
		//	measurementMatrix.at<float>(i, i) = 1.0f; // measured v

		measurement_vel.at<float>(0) = -(18163.0f * (measurement_or.at<float>(1) * measurement_or.at<float>(1)) - 86.0f * measurement_or.at<float>(1));
		measurement_vel.at<float>(1) = (18163.0f * (measurement_or.at<float>(0) * measurement_or.at<float>(0)) - 86.0f * measurement_or.at<float>(0));
		measurement_vel.at<float>(2) = 0.0f;

		if (measurement_or.at<float>(1) < 0.0f)
			measurement_vel.at<float>(0) = -measurement_vel.at<float>(0);

		if (measurement_or.at<float>(0) < 0.0f)
			measurement_vel.at<float>(1) = -measurement_vel.at<float>(1);

		Mat Rw(3, 3, CV_32F);
		Mat measurement_or_yaw = measurement_or.clone();
		measurement_or_yaw.at<float>(0) = 0.0f;
		measurement_or_yaw.at<float>(1) = 0.0f;
		cv::RotationMatrix3D(measurement_or_yaw, Rw);
		measurement_vel		= Rw * measurement_vel;

		memcpy_s(&measurement.data[3 * 4], 12, measurement_vel.data, 12); // vel: this is the fastest method

		measurement_accel = 0.0f;
		memcpy_s(&measurement.data[6 * 4], 12, measurement_accel.data, 12); // accel: this is the fastest method
	}



	/* acceleration measurement */
	if (use_accel)
	{
		EKF->measurementNoiseCov.at<float>(6, 6) = 30.0f;
		EKF->measurementNoiseCov.at<float>(7, 7) = 30.0f;
		EKF->measurementNoiseCov.at<float>(8, 8) = 30.0f;

		measurement_accel.at<float>(0) = (m->accel[0] - accel_avg[0]) * MG_TO_MM2;
		measurement_accel.at<float>(1) = (m->accel[1] - accel_avg[1]) * MG_TO_MM2;
		measurement_accel.at<float>(2) = (m->accel[2] - accel_avg[2]) * MG_TO_MM2;

		Mat Rw(3, 3, CV_32F);
		cv::RotationMatrix3D(measurement_or, Rw);
		measurement_accel	= Rw * measurement_accel;

		measurement_accel.at<float>(2) += 9806.65f;

		dumpMatrix(measurement_accel);


		// be aware: measurement.data is uchar, so index 12 is float matrix index 12/4 = 3
		memcpy_s(&measurement.data[6 * 4], 12, measurement_accel.data, 12); // accel: this is the fastest method
		EKF->measurementNoiseCov.at<float>(6, 6) = 10.0f;
		EKF->measurementNoiseCov.at<float>(7, 7) = 10.0f;
		EKF->measurementNoiseCov.at<float>(8, 8) = 10.0f;
	}




	/* correct */
	EKF->correct(measurement, m->time);


	/* directly inject attitude into state vector */
	memcpy_s(&state->data[9 * 4], 12, measurement_or.data, 12);


	/* release KF */
	EKF->release();


	/* store current state (position) as previous state */
	memcpy_s(prev_state.data, 48, state->data, 48); // this is the fastest method
	/**/


	/* elevation map */
	// check is mapping mode is on
	//if (!controller->mode(SLAM_MODE_MAP))
		//update_elevation_map(m->altitude/* - alt_correct*/);


	if (counter++ % 500 == 0)
	{
		/*
		printf("reset state\n");

		state->at<float>(0) = 0.0f;
		state->at<float>(1) = 0.0f;
		state->at<float>(0) = 0.0f;
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
		/*
		fprintf(error_log, "%f,%f,%f,%f\n",
			(float) m->time,
			abs(state->at<float>(0) - m->gt_loc[0]),
			abs(state->at<float>(1) - (m->gt_loc[1] - 1000.0f)),
			abs(state->at<float>(2) - (m->gt_loc[2] - 2496.0f))
			);

		fflush(error_log);
		*/

		//dumpMatrix(controller->EKF.errorCovPost);
		//Sleep(1500);
		//printf("\n\n");
	}
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

	h = hit.at<float>(2);

	if (abs(h) >= 70.0f)
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