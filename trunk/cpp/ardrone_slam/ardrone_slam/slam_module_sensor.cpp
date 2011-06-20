#include "global.h"
#include "slam_module_sensor.h"
#include "bot_ardrone.h"
#include "opencv_helpers.h"
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;


slam_module_sensor::slam_module_sensor(slam *controller)
{
	this->controller = controller;
	KF = &controller->KF;
	state = &KF->statePost;

	//processNoise(9, 1, CV_32F)

	prev_update = clock();

	measurement = Mat::zeros(3, 1, CV_32F);
}


slam_module_sensor::~slam_module_sensor(void)
{
}


void slam_module_sensor::process(bot_ardrone_measurement *m)
{
	double difftime = ((double)clock() - prev_update) / CLOCKS_PER_SEC;
	prev_update = clock();

	//printf("%f %f %f\n", m->or[0], m->or[1], m->or[2]);

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
	// a
	measurement.at<float>(0) = m->accel[0] * MG_TO_MS2;
	measurement.at<float>(1) = m->accel[1] * MG_TO_MS2;
	measurement.at<float>(2) = m->accel[2] * MG_TO_MS2;

	// w (attitude)
	//measurement.at<float>(3) = m->or[0];
	//measurement.at<float>(4) = m->or[1];
	//measurement.at<float>(5) = m->or[2];

	//measurement.at<float>(0) = 1.0f;
	//measurement.at<float>(1) = 0.0f;
	//measurement.at<float>(2) = 0.0f;

	//Mat angles(3, 1, CV_32F);
	Mat angles = (Mat_<float>(3,1) << m->or[0] * MD_TO_RAD, m->or[1] * MD_TO_RAD, m->or[2] * MD_TO_RAD);
	//Mat angles = (Mat_<float>(3,1) << 0.0f, 0.0174532925f, 0.0f); // 1 deg
	Mat R(3, 3, CV_32F);
	cv::Rodrigues(angles, R);

	measurement = R * measurement;

	//dumpMatrix(R);
	//printf("\n");
	//dumpMatrix(measurement);
	//printf("\n\n");

	KF->correct(measurement);


	/* state */
	//randn( processNoise, Scalar(0), Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));
	//state = KF.statePost /* + processNoise*/;

	//dumpMatrix(KF->statePost);

	printf("state: [%f, %f, %f]\n", state->at<float>(0), state->at<float>(1), state->at<float>(2));
	printf("gt:    [%f, %f, %f]\n", m->gt_loc[0], m->gt_loc[1] - 10.0f, m->gt_loc[2] + 10.0f);
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