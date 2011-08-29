#include "global.h"
#include "slam_module_sensor.h"
#include "bot_ardrone.h"
#include "opencv_helpers.h"
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;


slam_module_sensor::slam_module_sensor(slam *controller):
	measurement(9, 1, CV_32F),
	measurementMatrix(9, 12, CV_32F),
	measurementNoiseCov(9, 9, CV_32F)
{
	this->controller = controller;


	counter = 0;


	/* KF */
	KF = &controller->KF;
	state = &KF->statePost;


	// H vector
	measurementMatrix = 0.0f;
	for(int i = 0; i < 3; i++)
	{
		measurementMatrix.at<float>(i, 6+i) = 1.0f; // measured a
	}

	measurementNoiseCov = 0.0f;
	//setIdentity(measurementNoiseCov, Scalar::all(1e-5));
	float MNC[12] = {
		0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f,
		0.5f, 0.5f, 0.5f,
		0.0f, 0.0f, 0.0f
	};
	MatSetDiag(measurementNoiseCov, MNC);
}


slam_module_sensor::~slam_module_sensor(void)
{
}


void slam_module_sensor::process(bot_ardrone_measurement *m)
{
	/* Time different between current measurement and previous measurement.
	 * Used to calculate the transition matrix.
	 * I assume the vehicle is hovering when starting SLAM. So the first couple measurements have a small impact.
	 */
	if (counter == 0)
		difftime = 0.0001; // it may take some time between class initialization and processing the first measurement.
	else
		difftime = m->time - controller->KF_prev_update;



	/* set initial position: module_frame is not used before position is known */
	if (!controller->KF_running)
	{
		KF->statePost.at<float>(2) = (float) -m->altitude; // write initial height directly into state vector
		controller->KF_running = true; // KF is initialized. Now that the initial height of the vehicle is known, the frame module can start working
	}

	return;


	/* measurement */
	Mat m_or = (Mat_<float>(3,1) << m->or[0] * MD_TO_RAD, m->or[1] * MD_TO_RAD, m->or[2] * MD_TO_RAD);
	Mat m_accel = (Mat_<float>(3,1) << m->accel[0], m->accel[1], m->accel[2]);


	// transform angular acceleration to world accelerations
	Mat Rw(3, 3, CV_32F);
	cv::RotationMatrix3D(m_or, Rw);
	m_accel = Rw * m_accel;


	// compensate for gravity
	if (!m->usarsim)
		m_accel.at<float>(2) += 1000.0f;

	// convert MG to MS2
	m_accel = m_accel * MG_TO_MM2;

	memcpy_s(measurement.data, 12, m_accel.data, 12); // accel: this is the fastest method



	/* lock KF */
	WaitForSingleObject(controller->KFSemaphore, 0L);


	/* switch KF matrices */
	KF->measurementMatrix	= measurementMatrix;
	KF->measurementNoiseCov	= measurementNoiseCov;


	/* update transition matrix */
	controller->update_transition_matrix((float) difftime);


	/* predict */
	Mat prediction = KF->predict();


	/* correct */
	KF->correct(measurement);


	/* directly inject attitude into state vector */
	KF->statePost.at<float>(9) = m_or.at<float>(0);
	KF->statePost.at<float>(10) = m_or.at<float>(1);
	KF->statePost.at<float>(11) = m_or.at<float>(2);


	/* release KF */
	controller->KF_prev_update = m->time;
	ReleaseSemaphore(controller->KFSemaphore, 1, NULL);


	//printf("GT OR: %f, %f, %f\n", m_or.at<float>(0), m_or.at<float>(1), m_or.at<float>(2));
	/**/



	/* elevation map */
	//update_elevation_map(m->altitude);


	if (counter++ % 1000 == 0)
	{
		//-52.0,5.68,-4.0

		m->gt_loc[0] += 52.0f;
		m->gt_loc[1] -= 5.68f;
		m->gt_loc[2] += 3.63f;

		//printf("state: [%f, %f, %f]\n", state->at<float>(0), state->at<float>(1), state->at<float>(2));
		//printf("gt:    [%f, %f, %f]\n", m->gt_loc[0] * 1000.f, m->gt_loc[1] * 1000.f, m->gt_loc[2] * 1000.f);
		//printf("gt:    [%f, %f, %f]\n", m_or.at<float>(0), m_or.at<float>(1), m_or.at<float>(2));
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

	//controller->set_scale(scale);
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
		controller->elevation_map.update(hit.at<float>(0), hit.at<float>(1), hit.at<float>(2), 10);
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