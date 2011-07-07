#include "global.h"
#include "slam_module_sensor.h"
#include "bot_ardrone.h"
#include "opencv_helpers.h"
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;


slam_module_sensor::slam_module_sensor(slam *controller):
	measurementMatrix(3, 12, CV_32F),
	measurementNoiseCov(3, 3, CV_32F)
{
	this->controller = controller;


	prev_update = clock();
	counter = 0;
	//scale_set = false;



	/* KF */
	KF = &controller->KF;
	state = &KF->statePost;


	// H vector
	measurementMatrix = 0.0f;
	for(int i = 0; i < 3; i++)
	{
		measurementMatrix.at<float>(i, 6+i) = 1.0f; // measured a
		//KF.measurementMatrix.at<float>(3+i, 9+i) = 1.0f; // measured q (attitude/orientation)
	}

	measurementNoiseCov = 0.0f;
	setIdentity(measurementNoiseCov, Scalar::all(1e-5));
}


slam_module_sensor::~slam_module_sensor(void)
{
}


void slam_module_sensor::process(bot_ardrone_measurement *m)
{
	double difftime = ((double)clock() - prev_update) / CLOCKS_PER_SEC;
	prev_update = clock();


	/* set scale */
	/*
	if (!scale_set)
		calculate_scale(m);
	*/


	/* set initial position: module_frame is not used before position is known */
	if (!controller->KF_running)
	{
		KF->statePost.at<float>(2) = (float) -m->altitude; // write initial height directly into state vector
		controller->KF_running = true;
	}

	return;


	/* switch KF matrices */
	KF->measurementMatrix = measurementMatrix;
	KF->measurementNoiseCov = measurementNoiseCov;


	/* measurement */
	Mat m_or = (Mat_<float>(3,1) << m->or[0] * MD_TO_RAD, m->or[1] * MD_TO_RAD, m->or[2] * MD_TO_RAD);
	Mat m_accel = (Mat_<float>(3,1) << m->accel[0], m->accel[1], m->accel[2]);



	/* update transition matrix */
	for (int i = 0; i < 3; i++)
	{
		// position (p)
		KF->transitionMatrix.at<float>(i, 3+i) = float(difftime);
		KF->transitionMatrix.at<float>(i, 6+i) = float(0.5 * difftime*difftime);
		// velocity (v)
		KF->transitionMatrix.at<float>(3+i, 6+i) = float(difftime);
	}


	/* predict */
	Mat prediction = KF->predict();


	/* correct */
	measurement = m_accel;


	// transform angular acceleration to world accelerations
	Mat Rw(3, 3, CV_32F);
	cv::RotationMatrix3D(m_or, Rw);
	measurement = Rw * measurement;


	/* compensate for gravity */
	if (!m->usarsim)
		measurement.at<float>(2) += 1000.0f;


	/* convert MG to MS2 */
	measurement = measurement * MG_TO_MM2;

	/*
	dumpMatrix(measurement);
	printf("(%f)\n", m->accel[2]);
	printf("\n\n");
	return;
	*/

	KF->correct(measurement);


	/* directly inject attitude into state vector */
	KF->statePost.at<float>(9) = m_or.at<float>(0);
	KF->statePost.at<float>(10) = m_or.at<float>(1);
	KF->statePost.at<float>(11) = m_or.at<float>(2);
	/**/



	/* state */
	//randn( processNoise, Scalar(0), Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));
	//state = KF.statePost /* + processNoise*/;

	//dumpMatrix(KF->statePost);

	if (counter++ % 20 == 0)
	{
		//-52.0,5.68,-4.0

		m->gt_loc[0] += 52.0f;
		m->gt_loc[1] -= 5.68f;
		m->gt_loc[2] += 3.63f;

		//printf("state: [%f, %f, %f]\n", state->at<float>(0), state->at<float>(1), state->at<float>(2));
		//printf("gt:    [%f, %f, %f]\n", m->gt_loc[0] * 1000.f, m->gt_loc[1] * 1000.f, m->gt_loc[2] * 1000.f);
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


void slam_module_sensor::calculate_scale(bot_ardrone_measurement *m)
{
	//scale_set = true;

	double altitude = (double) m->altitude;
	double scale = 2.0f * tan(((BOT_ARDRONE_CAM_FOV)/180.0f)*PI) * altitude;
	scale /= (double) BOT_ARDRONE_CAM_RESOLUTION_W;
	//scale = 1.0 / scale; // mm -> px

	controller->set_scale(scale);
}





	//if (frame == NULL) // we need the frame size in order to calculate the canvas scale
	//	return;

	/*
	// initial height
	if (initial_height == -1)
	{
		initial_height = m->altitude;
		canvas_scale = 2.0f * tan(((BOT_ARDRONE_CAM_FOV)/180.0f)*PI) * (float)initial_height;
		//canvas_scale /= (float)frame->width;
		printf("MAP SCALE: 1px is %f mm\n", canvas_scale);
		printf("MAP SIZE: %f x %f m\n", canvas_scale * 0.8f, canvas_scale * 0.8f);
	}

	elevation = initial_height - m->altitude;
	double rel_elevation = (double)elevation / (double)initial_height;

	if (abs(rel_elevation) > 0.1)
	{
		float a = (2.0f * tan(((BOT_ARDRONE_SONAR_FOV)/180.0f)*PI) * (float) m->altitude);
		// size (mm) to pixels
		a *= 1.0f / canvas_scale;

		if (SLAM_BUILD_OBSTACLE_MAP)
		{
			printf("obstacle found. Distance: %i\n", m->altitude);

			int d = max(1, int(a * 0.5));
			int x, y, w, h;
			x = max(0, last_loc[0] - d);
			y = max(0, last_loc[1] - d);
			w = min(2*d, obstacle_map.rows - x);
			h = min(2*d, obstacle_map.cols - y);

			y = max(0, y - 40); // tmp
			x = max(0, x - 20);
			//y -= 90;

			printf("ROI: %i, %i, %i, %i\n", x, y, w, h);

			Rect r(x, y, w, h);
			Mat roi(obstacle_map, r);
			roi = 0;
		}
	}
	*/

	//int t[3] = {last_loc[0], last_loc[1], elevation};
	//matlab->add_elevation_map_tuple((int*) t);