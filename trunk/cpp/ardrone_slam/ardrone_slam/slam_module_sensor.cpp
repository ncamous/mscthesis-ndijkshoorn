#include "global.h"
#include "slam_module_sensor.h"
#include "bot_ardrone.h"

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

	/* update transition matrix */
	for (int i = 0; i < 9; i+=3)
	{
		// position
		KF->transitionMatrix.at<float>(i, i+1) = float(difftime);
		KF->transitionMatrix.at<float>(i, i+2) = float(0.5 * difftime*difftime);
		// velocity
		KF->transitionMatrix.at<float>(i+1, i+2) = float(difftime);
	}


	/* predict */
	Mat prediction = KF->predict();


	/* correct */
	measurement.at<float>(0) = m->accel[0] * MG_TO_MS2;
	measurement.at<float>(1) = m->accel[1] * MG_TO_MS2;
	measurement.at<float>(2) = m->accel[2] * MG_TO_MS2;
	//measurement = KF.statePost.clone();
	//measurement.at<float>(2) = m->accel[0] * MG_TO_MS2;
	//measurement = KF.transitionMatrix*measurement;

	KF->correct(measurement);


	/* state */
	//randn( processNoise, Scalar(0), Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));
	//state = KF.statePost /* + processNoise*/;


	printf("state: [%f, %f, %f]\n", state->at<float>(0), state->at<float>(3), state->at<float>(6));
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