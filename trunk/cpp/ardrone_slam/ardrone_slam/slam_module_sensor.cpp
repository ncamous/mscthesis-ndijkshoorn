#include "global.h"
#include "slam_module_sensor.h"
#include "bot_ardrone.h"
#include "opencv_helpers.h"
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;


slam_module_sensor::slam_module_sensor(slam *controller):
	measurement(9, 1, CV_32F),
	measurementMatrix(9, 12, CV_32F),
	measurementNoiseCov(9, 9, CV_32F),

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


	// H vector
	measurementMatrix = 0.0f;
	measurementMatrix.at<float>(2, 2) = 1.0f; // measured altitude

	if (controller->mode(SLAM_MODE_VEL))
	{
		for(int i = 3; i < 9; i++)
			measurementMatrix.at<float>(i, i) = 1.0f; // measured v+a
	}

	if (controller->mode(SLAM_MODE_ACCEL))
	{
		for(int i = 6; i < 9; i++)
			measurementMatrix.at<float>(i, i) = 1.0f; // measured a
	}


	measurementNoiseCov = 0.0f;
	float MNC[9] = {
		0.0f, 0.0f, 300.0f,		// p: mm
		100.0f, 100.0f, 100.0f,	// v (mm/s)
		//1.0f, 1.0f, 1.0f		// a (mm/s2)
		4.0f, 4.0f, 4.0f		// a (mm/s2)
	};
	MatSetDiag(measurementNoiseCov, MNC);


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
	if (++calib_measurements > 600)
	{
		printf("Calibration complete: %f, %f, %f\n", accel_avg[0], accel_avg[1], accel_avg[2]);
		calibrated = true;
		return;
	}

	measurement_or.at<float>(0) = m->or[0] * MD_TO_RAD;
	measurement_or.at<float>(1) = m->or[1] * MD_TO_RAD;
	measurement_or.at<float>(2) = 0.0f;

	measurement_accel.at<float>(0) = m->accel[0];
	measurement_accel.at<float>(1) = m->accel[1];
	measurement_accel.at<float>(2) = m->accel[2];

	Mat Rw(3, 3, CV_32F);
	cv::RotationMatrix3D(measurement_or, Rw);
	measurement_accel	= Rw * measurement_accel;
	measurement_accel.at<float>(2) += 1000.0f;

	if (calib_measurements <= 600)
	{
		accel_sum[0] += measurement_accel.at<float>(0);
		accel_sum[1] += measurement_accel.at<float>(1);
		accel_sum[2] += measurement_accel.at<float>(2);

		yaw_sum += m->or[2];
	}

	if (calib_measurements == 600)
	{
		accel_avg[0] = (float) (accel_sum[0] / 600.0);
		accel_avg[1] = (float) (accel_sum[1] / 600.0);
		accel_avg[2] = (float) (accel_sum[2] / 600.0);

		EKF->yaw_offset = (float) (yaw_sum / 600.0); // AR.Drone's start Z orientation is always 0.0
	}
}


void slam_module_sensor::process(bot_ardrone_measurement *m)
{
	/* Time different between current measurement and previous measurement.
	 * Used to calculate the transition matrix.
	 * I assume the vehicle is hovering when starting SLAM. So the first couple measurements have a small impact.
	 */
	if (!calibrated)
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
	}
	else if (m->time <= EKF->last_measurement_time)
	{
		return;
	}

	//printf("sonar: %i\n", m->altitude);


	bool use_accel	= controller->mode(SLAM_MODE_ACCEL);
	bool use_vel	= controller->mode(SLAM_MODE_VEL);



	/* orientation measurement */
	measurement_or.at<float>(0) = m->or[0] * MD_TO_RAD;
	measurement_or.at<float>(1) = m->or[1] * MD_TO_RAD;
	measurement_or.at<float>(2) = (m->or[2] - EKF->yaw_offset) * MD_TO_RAD;



	/* only use orientation sensor */
	if (!use_accel && !use_vel)
	{
		memcpy_s(&state->data[9 * 4], 12, measurement_or.data, 12);
		return;
	}



	difftime = (float) EKF->difftime(m->time);



	/* velocity measurement */
	if (use_vel)
	{
		memcpy_s(measurement_vel.data, 12, m->vel, 12);

		Mat Rw(3, 3, CV_32F);
		Mat measurement_or_yaw = measurement_or.clone();
		measurement_or_yaw.at<float>(0) = 0.0f;
		measurement_or_yaw.at<float>(1) = 0.0f;
		cv::RotationMatrix3D(measurement_or_yaw, Rw);
		measurement_vel		= Rw * measurement_vel;

		memcpy_s(&measurement.data[3 * 4], 12, measurement_vel.data, 12); // vel: this is the fastest method

		// compute virtual acceleration measurement
		measurement.at<float>(6) = (measurement.at<float>(3) - prev_state.at<float>(3)) / difftime;
		measurement.at<float>(7) = (measurement.at<float>(4) - prev_state.at<float>(4)) / difftime;
		measurement.at<float>(8) = (measurement.at<float>(5) - prev_state.at<float>(5)) / difftime;
	}


	/* acceleration measurement */
	if (use_accel)
	{
		measurement_accel.at<float>(0) = (m->accel[0] - accel_avg[0]) * MG_TO_MM2;
		measurement_accel.at<float>(1) = (m->accel[1] - accel_avg[1]) * MG_TO_MM2;
		measurement_accel.at<float>(2) = (m->accel[2] - accel_avg[2]) * MG_TO_MM2;

		Mat Rw(3, 3, CV_32F);
		cv::RotationMatrix3D(measurement_or, Rw);
		measurement_accel	= Rw * measurement_accel;

		measurement_accel.at<float>(2) += 9806.65f;

		// be aware: measurement.data is uchar, so index 12 is float matrix index 12/4 = 3
		memcpy_s(&measurement.data[6 * 4], 12, measurement_accel.data, 12); // accel: this is the fastest method
	}


	/* altitude measurement */
	measurement.at<float>(2) = (float) -m->altitude;




	/* lock KF */
	EKF->lock();


	/* switch KF matrices */
	update_measurement_matrix();
	EKF->measurementMatrix		= measurementMatrix;
	EKF->measurementNoiseCov	= measurementNoiseCov;


	/* update transition matrix */
	controller->update_transition_matrix(difftime);


	/* predict */
	EKF->predict();


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


	if (counter++ % 50 == 0)
	{
		/*
		printf("error: %f, %f, %f\n", 
			abs(state->at<float>(0) - m->gt_loc[0]),
			abs(state->at<float>(1) - (m->gt_loc[1] - 1000.0f)),
			abs(state->at<float>(2) - (m->gt_loc[2] - 2496.0f))
			);

		fprintf(error_log, "%f,%f,%f,%f\n",
			(float) m->time,
			abs(state->at<float>(0) - m->gt_loc[0]),
			abs(state->at<float>(1) - (m->gt_loc[1] - 1000.0f)),
			abs(state->at<float>(2) - (m->gt_loc[2] - 2496.0f))
			);

		fflush(error_log);
		*/

		//dumpMatrix(controller->EKF.errorCovPost);

		//printf("\n\n");
	}
}


void slam_module_sensor::update_measurement_matrix()
{
	for (int i = 6; i < 9; i++)
		measurementMatrix.at<float>(i, i) = 1.0f;

	//measurementMatrix = 0.0f;
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