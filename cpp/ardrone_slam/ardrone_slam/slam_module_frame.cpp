﻿#include "global.h"
#include "slam_module_frame.h"
#include "bot_ardrone.h"

#include "opencv_helpers.h"
#include <cv.hpp>
#include <cxcore.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;


slam_module_frame::slam_module_frame(slam *controller):
	camera_matrix(3, 3, CV_32F),
	world_plane(3, 1, CV_32F),
	world_plane_normal(3, 1, CV_32F),

	T(4, 4, CV_32F),
	originH(4, 1, CV_32F),

	frame(BOT_ARDRONE_FRAME_H, BOT_ARDRONE_FRAME_W, CV_8UC4, NULL, 0),
	frame_gray(BOT_ARDRONE_FRAME_H, BOT_ARDRONE_FRAME_W, CV_8U),

	obj_pos(3, 1, CV_64F),
	obj_or(3, 1, CV_64F),
	new_pos(3, 1, CV_32F),
	new_or(3, 1, CV_32F),

	measurement(15, 1, CV_32F),

	prev_state(15, 1, CV_32F)
{
	this->controller = controller;
	this->map = &controller->map;
	prev_frame_exists = false;

	set_camera();

	de = new SurfDescriptorExtractor();


	/* image corners */
	image_corners.push_back(Point2f(0.0f, 0.0f));
	image_corners.push_back(Point2f(0.0f, (float) (BOT_ARDRONE_FRAME_H - 1)));
	image_corners.push_back(Point2f((float) (BOT_ARDRONE_FRAME_W - 1), (float) (BOT_ARDRONE_FRAME_H - 1)));
	image_corners.push_back(Point2f((float) (BOT_ARDRONE_FRAME_W - 1), 0.0f));
	image_corners.push_back(image_corners[2] * 0.5f); // center


	/* world plane */
	world_plane = 0.0f;
	world_plane_normal = 0.0f;
	world_plane_normal.at<float>(2) = -1.0f;
	/**/


	T = 0.0f;
	T.at<float>(3, 3) = 1.0f; // stays always 1

	originH = 0.0f;
	originH.at<float>(3) = 1.0f;


	prev_frame_descriptors = NULL;



	/* KF */
	EKF = &controller->EKF;
	state = &EKF->statePost;
	cov = &EKF->errorCovPost;



	// tmp
	last_loc = clock();

	fopen_s (&loc_log, "dataset/loc_log.txt" , "w");
	nr_visual_motion = nr_visual_motion_success = 0;
}


slam_module_frame::~slam_module_frame(void)
{
}


void slam_module_frame::process(bot_ardrone_frame *f)
{
	/* NOTE: the world (position) is solvePnP's object position
	 * The position of the AR.Drone is retrieved by inverting the camera -> object transformation
	 * that is returned by solvePnP
	 */

	// scale not kwown: no use of processing the frame
	if (!EKF->running)
		return;


	controller->sensor_pause(f->time); // freeze sensor measurements, because we update the state


	this->f = f;
	frame.data = (unsigned char*) f->data;
	//blur(frame, frame, cv::Size(3,3));
	//add_noise(frame); // TMP
	cvtColor(frame, frame_gray, CV_BGRA2GRAY);



	/* reset */
	bool loc			= false;
	features_extracted	= false;
	keypoints.clear();
	imagepoints.clear();



	/* process */
	if (controller->mode(SLAM_MODE_VISUALLOC))
	{
		clock_t start = clock();
		loc = process_visual_loc();
		//printf("loc time: %f\n", (clock() - start) / (double)CLOCKS_PER_SEC);
	}


	if (controller->mode(SLAM_MODE_VISUALMOTION) && !loc)
	{
		printf("getting motion\n");
		process_visual_motion();
	}

	Mat frame_state = state->clone (); // store updated state
	controller->sensor_resume(); // state not updated anymore


	if (controller->mode(SLAM_MODE_MAP))
		process_map(frame_state);
	else
		Sleep(10); // some time before processing next frame. In order to process all sensor measurements
}


bool slam_module_frame::process_visual_motion()
{
	SLAM_VISUALMOTION_START

	if (state->at<float>(2) > -300.0f)
		return false;

	nr_visual_motion++;


	printf("nr_visual_motion: %i/%i\n", nr_visual_motion_success, nr_visual_motion);


	/* find features */
	if (!features_extracted)
	{
		get_features(frame_gray, keypoints);
		get_descriptors(frame_gray, keypoints, descriptors);
	}


	if (keypoints.size() < 4)
	{
		//printf("Not enough features found: dropping frame\n");
		//prev_frame_exists = false; // do not use this frame for visual motion
		SLAM_VISUALMOTION_END
		return false;
	}


	/* convert frame (pixel) coordinates to world coordinates (2D) */
	vector<Point3f> imagepoints_wc;
	imagepoints_to_local3d(imagepoints, imagepoints_wc, NULL, true);


	/* match with previous frame */
	if (prev_frame_exists)
	{

		/* match */
		vector<DMatch> matches;
		get_matches(descriptors, prev_frame_descriptors, matches, true, 0.18);

		//if (matches.size() < 4)
		if (matches.size() < 3)
		{
			prev_frame_exists = false; // do not use this frame for visual motion
			SLAM_VISUALMOTION_END
			return false;
		}





		vector<short> inliers;
		Mat T(1, 1, CV_32FC3);
		float R = -1.0f;


		//double confidence = find_robust_translation_rotation(imagepoints_wc, prev_frame_wc, Mat(), matches, inliers, T, R, 50.0);

		int nr_inliers = find_robust_translation_rotation_inliers(imagepoints_wc, prev_frame_wc, Mat(), matches, inliers, T, R, 20.0);
		if (nr_inliers < 6)
		{
			printf("Not enough inliers found (%i): dropping frame\n", nr_inliers);
			//return;
		}
		else
		{
			printf("Inliers: %i/%i\n", nr_inliers, matches.size());
		}

		double confidence = 0.0;
		Mat tmp;
		drawMatches(frame_gray, keypoints, prev_frame_gray, prev_keypoints, matches, tmp);
		imshow("Image:", tmp);
        cvWaitKey(6);
		Sleep(2000);
		
		/*
		vector<short> mask;
		Mat H;
		int nr_inliers = find_robust_perspective_transformation(imagepoints_wc, prev_frame_wc, matches, mask, 20, H, 10.0);


		if (nr_inliers < 6)
		{
			printf("Not enough inliers found (%i): dropping frame\n", nr_inliers);
			//return;
		}

		Mat p1m = Mat::zeros(1, 1, CV_32FC2);
		//printf("# %f,%f\n", p1m.at<Vec2f>(0)[0], p1m.at<Vec2f>(0)[1]);
		//Mat p1mt;
		//perspectiveTransform(p1m, T, H);
		T.at<Vec2f>(0)[0] = (float) H.at<double>(0, 2);
		T.at<Vec2f>(0)[1] = (float) H.at<double>(1, 2);
		//printf("## %f,%f\n", p1mt.at<Vec2f>(0)[0], p1mt.at<Vec2f>(0)[1]);
		*/
		/*
		vector<short> mask;
		Mat H;
		int nr_inliers = find_robust_affine_transformation(imagepoints_wc, prev_frame_wc, matches, mask, 20, H, 10.0);
		if (nr_inliers < 4)
		{
			printf("Not enough inliers found (%i): dropping frame\n", nr_inliers);
			//return;
		}

		T.at<Vec2f>(0)[0] = (float) H.at<double>(0, 2);
		T.at<Vec2f>(0)[1] = (float) H.at<double>(1, 2);

	


		double confidence = 0.0;
		*/

		//if (confidence >= 0.7)
		if (((double)nr_inliers / (double)matches.size()) >= 0.5 && abs(T.at<Vec3f>(0)[0]) < 500.0f && abs(T.at<Vec3f>(0)[1]) < 500.0f)
		//if (((double)nr_inliers / (double)matches.size()) >= 0.5 && abs(T.at<Vec2f>(0)[0]) < 500.0f && abs(T.at<Vec2f>(0)[1]) < 500.0f)
		{
			vector<DMatch> matches_f;

			/*
			for (size_t i = 0; i < inliers.size(); i++)
			{
				matches_f.push_back(matches[inliers[i]]);
			}

			 Mat tmp;
             //printf("%i %i %i\n", keypoints.size(), prev_keypoints.size(), matches_f.size());
             drawMatches(frame_gray, keypoints, prev_frame_gray, prev_keypoints, matches_f, tmp);
             imshow("Image:", tmp);
             cvWaitKey(6);
			 */


			/* lock KF */
			EKF->lock();

			printf("nr_visual_motion: %i/%i\n", ++nr_visual_motion_success, nr_visual_motion);


#ifdef SLAM_VISUALMOTION_WRITE_STATE_DIRECTLY

			Mat cam_pos(3, 1, CV_32F);
			Mat cam_or(3, 1, CV_32F);
			get_localcam(cam_pos, cam_or, &prev_state);

			cam_pos.at<float>(0) += T.at<Vec3f>(0)[0];
			cam_pos.at<float>(1) += T.at<Vec3f>(0)[1];

			localcam_to_world(cam_pos, cam_or);

			state->at<float>(0) = cam_pos.at<float>(0);
			state->at<float>(1) = cam_pos.at<float>(1);

			/* unlock KF */
			EKF->release();

			//printf("SLAM VISUALMOTION (%f, %f)   [%f]\n", cam_pos.at<float>(0), cam_pos.at<float>(1), confidence);

#else

			/* update transition matrix */
			difftime = EKF->difftime(f->time);
			controller->update_transition_matrix((float) difftime);



			/* predict */
			EKF->predict();



			// H vector
			//EKF->measurementMatrix = 0.0f;
			setIdentity(EKF->measurementMatrix);



			// Rk vector
			float MNC[15] = {
				300.0f, 300.0f, 200.0f,	// p: mm
				2.1f, 2.1f, 10.0f,	// v (mm/s)
				50.0f, 50.0f, 50.0f,	// a (mm/s2)
				0.02f, 0.02f, 0.02f,	// or (rad)
				0.01f, 0.01f, 0.01f		// ω (rad/s)
			};
			MatSetDiag(EKF->measurementNoiseCov, MNC);



			/* default measurements */
			memcpy_s(measurement.data, 60, EKF->statePre.data, 60);



			/* velocity measurement */
			float dt = (float) (f->time - prev_frame_time);

			Mat measurement_vel(3, 1, CV_32F);
			measurement_vel = 0.0f;
			memcpy_s(measurement_vel.data, 8, T.data, 8);
			measurement_vel /= dt;
			localvelocity_to_world(measurement_vel);

			memcpy_s(measurement.data + 12, 8, measurement_vel.data, 8);
			EKF->measurementNoiseCov.at<float>(3, 3) = 1.0f;
			EKF->measurementNoiseCov.at<float>(4, 4) = 1.0f;
			EKF->measurementNoiseCov.at<float>(5, 5) = 1.0f;



			/* correct */
			EKF->correct(measurement);



			/* unlock KF */
			EKF->release();
			printf("VISUAL VEL (%f, %f) [%f]\n", measurement_vel.at<float>(0), measurement_vel.at<float>(1), confidence);
#endif
		}
	}


#ifdef SLAM_VISUALMOTION_WRITE_STATE_DIRECTLY
	/* store current state (position) as previous state. Used to calculate vel/accel */
	memcpy_s(prev_state.data, 48, state->data, 48); // this is the fastest method
	// do this after localization is done?
#endif


	/* store current frame as previous frame */
	store_prev_frame();

	return true;
}


bool slam_module_frame::process_visual_loc()
{
	double dt = ((double)clock() - last_loc) / CLOCKS_PER_SEC;
	if (dt < 0.75)
		return false;


	SLAM_LOC_START


	/* get local world coordinates of the image corners */
	vector<Point3f> image_corners_wc;
	imagepoints_to_local3d(image_corners, image_corners_wc);


	/* get descriptors from map */
	Mat map_descriptors;
	Mat map_keypoints_wc;
	Mat map_keypoints_t;

	float radius = 2500.0f;
	//float radius = max(cov->at<float>(0, 0), cov->at<float>(1, 1));
	//radius += RectRadius(image_corners_wc); // should be dynamic and basic on the state cov

	Mat cam_pos(3, 1, CV_32F);
	Mat cam_or(3, 1, CV_32F);
	get_localcam(cam_pos, cam_or);
	Point3f wc(cam_pos);

	map->get_local_descriptors(map_descriptors, map_keypoints_wc, map_keypoints_t, wc, radius);


	/* not enough descriptors from map (unexplored area?) */
	if (map_descriptors.rows < 3)
	{
		SLAM_LOC_END
		return false;
	}

	
	// transform frame to canvas
	//Mat frameT;
	//controller->visual_map.frame_to_canvas(frame, frameT, image_corners, image_corners_wc);


	/* frame features */
	if (!features_extracted)
	{
		get_features(frame_gray, keypoints); // also stores image points in "imagepoints"
		get_descriptors(frame_gray, keypoints, descriptors);
	}


	/* convert frame (pixel) coordinates to world coordinates (2D) */
	vector<Point3f> imagepoints_wc;
	imagepoints_to_local3d(imagepoints, imagepoints_wc);



	/* match */
	vector<DMatch> matches;
	get_matches(descriptors, map_descriptors, matches, true);

	//printf("nr matches: %i\n", matches.size());

	if (matches.size() < 3)
	{
		SLAM_LOC_END
		return false;
	}


	vector<short> inliers;
	Mat T(1, 1, CV_32FC3);
	float R = -1.0f;

	double confidence = find_robust_translation_rotation(imagepoints_wc, map_keypoints_wc, map_keypoints_t, matches, inliers, T, R, 100.0);

	//printf("conf: %f\n", confidence);

	/*
	vector<DMatch> matches3;
	for (size_t i = 0; i < mask.size(); i++)
	{
		matches3.push_back(matches[mask[i]]);
	}

	Mat out;
	drawMatches( frame_gray, keypoints, first_frame, controller->visual_map.keypoints, matches3, out);
	imshow("Image:", out);
	cvWaitKey(4);
	*/


	if (confidence >= 0.7)
	{
		Mat cam_pos(3, 1, CV_32F);
		Mat cam_or(3, 1, CV_32F);
		get_localcam(cam_pos, cam_or);

		cam_pos.at<float>(0) += T.at<Vec3f>(0)[0];
		cam_pos.at<float>(1) += T.at<Vec3f>(0)[1];


		/*
		fprintf(loc_log, "%f,%f,%f\n",
			(float) f->time,
			cam_pos.at<float>(0),
			cam_pos.at<float>(1)
		);

		fflush(loc_log);
		*/


		localcam_to_world(cam_pos, cam_or);

		// rotation
		if (SLAM_LOC_UPDATE_YAW && confidence > 0.9)
		{
			EKF->yaw_offset -= R; // write to state here?
			printf("ROT: %f\n", R);
		}


		/* lock KF */
		EKF->lock();


#ifdef SLAM_LOC_WRITE_STATE_DIRECTLY

		state->at<float>(0) = cam_pos.at<float>(0);
		state->at<float>(1) = cam_pos.at<float>(1);

		/* unlock KF */
		EKF->release();
		last_loc = clock();

		printf("SLAM LOC (%f, %f) [%f]\n", cam_pos.at<float>(0), cam_pos.at<float>(1), confidence);

#else

		/* update transition matrix */
		difftime = EKF->difftime(f->time);
		controller->update_transition_matrix((float) difftime);



		/* predict */
		EKF->predict();



		// H vector
		//EKF->measurementMatrix = 0.0f;
		setIdentity(EKF->measurementMatrix);



		// Rk vector
		float MNC[15] = {
			10.0f, 10.0f, 200.0f,	// p: mm
			50.0f, 50.0f, 50.0f,	// v (mm/s)
			50.0f, 50.0f, 50.0f,	// a (mm/s2)
			0.02f, 0.02f, 0.02f,	// or (rad)
			0.01f, 0.01f, 0.01f		// ω (rad/s)
		};
		MatSetDiag(EKF->measurementNoiseCov, MNC);


		/* default measurements */
		memcpy_s(measurement.data, 60, EKF->statePre.data, 60);
		//memset(measurement.data + 12, 0, 12); // zero velocity



		/* position measurement */
		memcpy_s(measurement.data, 8, cam_pos.data, 8);
		/*
		EKF->measurementNoiseCov.at<float>(0, 0) = 10.0f;
		EKF->measurementNoiseCov.at<float>(1, 1) = 10.0f;

		for (int i = 3; i < 9; i++)
		{
			measurement.at<float>(i) = 0.0;
			EKF->measurementNoiseCov.at<float>(0, 0) = 0.0001f;
		}
		*/


		/* correct */
		EKF->correct(measurement);


		/* unlock KF */
		EKF->release();
		last_loc = clock();

		printf("VISUAL LOC (%f, %f) [%f]\n", cam_pos.at<float>(0), cam_pos.at<float>(1), confidence);
#endif

		return true;
	}


	SLAM_LOC_END

	return false;
}


bool slam_module_frame::process_map(Mat& frame_state)
{
	// flying too low, dont add to map (dark image)
	if (state->at<float>(12) < 350.0f)
		return false;


	vector<Point3f> image_corners_wc;
	imagepoints_to_local3d(image_corners, image_corners_wc, &frame_state);


	map->visual_map.update(frame, image_corners, image_corners_wc);


	/* features map (not-transformed image) */
	if (!features_extracted)
	{
		get_features(frame_gray, keypoints);
		get_descriptors(frame_gray, keypoints, descriptors);
	}

	vector<Point3f> imagepoints_wc;
	imagepoints_to_local3d(imagepoints, imagepoints_wc, &frame_state);

	map->update(keypoints, descriptors, imagepoints_wc);

	return true;
}


bool slam_module_frame::measurementSeemsOk()
{
	//if (measurement.at<float>(2) > 0.0f)
	//	return false;

	for (int i = 3; i < 5; i++) // check v
	{
		if (abs(measurement.at<float>(i) - state->at<float>(i)) > 500.0f)
			return false;
	}

	return true;
}


void slam_module_frame::store_prev_frame()
{
	prev_frame_exists		= true;
	prev_frame_time			= f->time;
	prev_frame_descriptors	= descriptors.clone();
	//prev_frame_ip			= imagepoints;
	imagepoints_to_local3d(imagepoints, prev_frame_wc, NULL, true);

	prev_keypoints = keypoints;
	prev_frame_gray = frame_gray.clone();
}


double slam_module_frame::find_robust_translation_rotation(InputArray p1, InputArray p2 /*map*/, InputArray t, vector<DMatch>& matches, vector<short>& inliers, Mat& T, float& R, double maxInlierDist)
{
	size_t nr_matches = matches.size();

	bool use_time = !t.empty();

	Mat _p1 = p1.getMat();
	Mat _p2 = p2.getMat();
	Mat _p2time = t.getMat();

	Mat p1m(nr_matches, 1, CV_32FC3);
	Mat p2m(nr_matches, 1, CV_32FC3);
	Mat p2time(nr_matches, 1, CV_32S); // time

	for (size_t i = 0; i < matches.size(); i++)
	{
		p1m.at<Vec3f>(i) = _p1.at<Vec3f>(matches[i].queryIdx);
		p2m.at<Vec3f>(i) = _p2.at<Vec3f>(matches[i].trainIdx);

		if (use_time)
			p2time.at<int>(i) = _p2time.at<int>(matches[i].trainIdx);
	}

	Mat p1s(3, 1, CV_32FC3);
	Mat p2s(3, 1, CV_32FC3);

	Mat p1s_best(3, 1, CV_32FC3);
	Mat p2s_best(3, 1, CV_32FC3);


	/** RANSAC **/
	Mat T_temp = Mat(1, 1, CV_32FC3);
	Scalar mean, stddev;
	double confidence, best_confidence;
	double time, best_time;
	CvRNG rng = cvRNG(-1); // not very nice place here

	best_confidence = 0.0;
	best_time = FLT_MAX;
	int subset_idx[3];

	for (int i = 0; i < 700; i++)
	{
		if (!getMatSubset(p1m, p2m, p1s, p2s, 300, rng, subset_idx))
			continue;

		meanStdDev(p2s - p1s, mean, stddev);

		// get 2D translation
		T_temp.at<Vec3f>(0)[0] = (float) mean[0];
		T_temp.at<Vec3f>(0)[1] = (float) mean[1];

		confidence = 1.0 - (stddev[0] / 200.0) - (stddev[1] / 200.0);

		if (use_time)
		{
			// average time of 3 features -> better: use covariance between the three times to determine quality
			time = p2time.at<int>(subset_idx[0]) + p2time.at<int>(subset_idx[1]) + p2time.at<int>(subset_idx[2]);

			if (confidence >= 0.7 && time < best_time)
			{
				best_confidence = confidence;
				best_time = time;
				memcpy_s(T.data, sizeof(float) * 2, T_temp.data, sizeof(float) * 2); // currently, only x and y used
				memcpy_s(p1s_best.data, sizeof(float) * 9, p1s.data, sizeof(float) * 9); // 3 points * 3 floats
				memcpy_s(p2s_best.data, sizeof(float) * 9, p2s.data, sizeof(float) * 9); // 3 points * 3 floats
			}

		}
		else
		{
			if (confidence > best_confidence)
			{
				best_confidence = confidence;
				memcpy_s(T.data, sizeof(float) * 2, T_temp.data, sizeof(float) * 2); // currently, only x and y used
				memcpy_s(p1s_best.data, sizeof(float) * 9, p1s.data, sizeof(float) * 9); // 3 points * 3 floats
				memcpy_s(p2s_best.data, sizeof(float) * 9, p2s.data, sizeof(float) * 9); // 3 points * 3 floats
			}
		}
	}





	// Use Scalar for to translate points. Using a Mat results in 1 point, when there are 3 points to translate
	Scalar Ts(T.at<Vec3f>(0)[0], T.at<Vec3f>(0)[1], 0.0f);


	/* Inliers mask */
	//Mat p1mt(nr_matches, 1, CV_32FC3);
	/*
	p1mt = p1m + Ts;
	for (size_t i = 0; i < nr_matches; i++)
	{
		if (norm(p2m.at<Point3f>(i) - p1mt.at<Point3f>(i)) <= maxInlierDist)
			inliers.push_back((short) i);
	}
	*/


	/* Rotation */
	if (best_confidence > 0.7 && R >= 0.0)
	{
		p1s_best += Ts; // transform query points to match training points
		R = Kabsch(p1s_best, p2s_best, Mat());
	}

	return best_confidence;
}



int slam_module_frame::find_robust_translation_rotation_inliers(InputArray p1, InputArray p2 /*map*/, InputArray t, vector<DMatch>& matches, vector<short>& inliers, Mat& T, float& R, double maxInlierDist)
{
	size_t nr_matches = matches.size();

	bool use_time = !t.empty();

	Mat _p1 = p1.getMat();
	Mat _p2 = p2.getMat();
	Mat _p2time = t.getMat();

	Mat p1m(nr_matches, 1, CV_32FC3);
	Mat p2m(nr_matches, 1, CV_32FC3);
	Mat p2time(nr_matches, 1, CV_32S); // time

	for (size_t i = 0; i < matches.size(); i++)
	{
		p1m.at<Vec3f>(i) = _p1.at<Vec3f>(matches[i].queryIdx);
		p2m.at<Vec3f>(i) = _p2.at<Vec3f>(matches[i].trainIdx);
	}

	Mat p1s(3, 1, CV_32FC3);
	Mat p2s(3, 1, CV_32FC3);

	Mat p1s_best(3, 1, CV_32FC3);
	Mat p2s_best(3, 1, CV_32FC3);


	/** RANSAC **/
	Mat T_temp = Mat(1, 1, CV_32FC3);
	Scalar mean, stddev;
	double confidence, best_confidence;
	double time, best_time;
	CvRNG rng = cvRNG(-1); // not very nice place here

	best_confidence = 0.0;
	best_time = FLT_MAX;
	int subset_idx[3];
	int best_inliers = 0;

	for (int i = 0; i < 700; i++)
	{
		if (!getMatSubset(p1m, p2m, p1s, p2s, 300, rng, subset_idx))
			continue;

		meanStdDev(p2s - p1s, mean, stddev);

		// get 2D translation
		T_temp.at<Vec3f>(0)[0] = (float) mean[0];
		T_temp.at<Vec3f>(0)[1] = (float) mean[1];
		T_temp.at<Vec3f>(0)[2] = 0.0f;

		Point3f Tpoint(mean[0], mean[1], 0.0f);

		Mat p1mt;
		p1mt = p1m + T_temp;


		int inliers = 0;
		for (size_t i = 0; i < nr_matches; i++)
		{
				Point3f tmp1 = p2m.at<Point3f>(i);
				Point3f tmp2 = p1m.at<Point3f>(i) + Tpoint;

				//printf("z: %f, %f\n", tmp1.z, tmp2.z);

				if (norm(tmp1 - tmp2) <= maxInlierDist)
				{
					inliers++;
				}
		}

		if (inliers > best_inliers)
		{
			memcpy_s(T.data, sizeof(float) * 2, T_temp.data, sizeof(float) * 2); // currently, only x and y used
			best_inliers = inliers;
		}
	}

	return best_inliers;
}



int slam_module_frame::find_robust_perspective_transformation(InputArray p1, InputArray p2, vector<DMatch>& matches, vector<short>& mask, int max, Mat& H, double maxInlierDist)
{
	size_t nr_matches = matches.size();

	Mat _p1 = p1.getMat();
	Mat _p2 = p2.getMat();

	Mat p1m(nr_matches, 1, CV_32FC2);
	Mat p2m(nr_matches, 1, CV_32FC2);

    for (size_t i = 0; i < matches.size(); i++)
    {
                p1m.at<Vec2f>(i)[0] = _p1.at<Vec3f>( matches[i].queryIdx )[0]; //p1[matches[i].queryIdx].x;
                p1m.at<Vec2f>(i)[1] = _p1.at<Vec3f>( matches[i].queryIdx )[1]; //p1[matches[i].queryIdx].y;

                p2m.at<Vec2f>(i)[0] = _p2.at<Vec3f>( matches[i].trainIdx )[0]; //p2[matches[i].trainIdx].x;
                p2m.at<Vec2f>(i)[1] = _p2.at<Vec3f>( matches[i].trainIdx )[1]; //p2[matches[i].trainIdx].y;
    }

        H = findHomography(p1m, p2m, CV_RANSAC, maxInlierDist);
        //memcpy_s(H.data, sizeof(double) * 9, homography.data, sizeof(double) * 9);

        Mat p1mt;
        perspectiveTransform(p1m, p1mt, H);

        int nr_inliers = 0;
        for (size_t i = 0; i < nr_matches; i++)
        {
                if (norm(p2m.at<Point2f>(i) - p1mt.at<Point2f>(i)) <= maxInlierDist)
                {
                        mask.push_back((short) i);
                        ++nr_inliers;
                }
        }

	return nr_inliers;
}


int slam_module_frame::find_robust_affine_transformation(InputArray p1, InputArray p2, vector<DMatch>& matches, vector<short>& mask, int max, Mat& H, double maxInlierDist)
{
	size_t nr_matches = matches.size();

	Mat _p1 = p1.getMat();
	Mat _p2 = p2.getMat();

	Mat p1m(1, nr_matches, CV_32FC2);
	Mat p2m(1, nr_matches, CV_32FC2);

	//Mat tmp_outliers;
	vector<uchar> tmp_inliers; 

    for (size_t i = 0; i < matches.size(); i++)
    {
                p1m.at<Vec2f>(i)[0] = _p1.at<Vec3f>( matches[i].queryIdx )[0]; //p1[matches[i].queryIdx].x;
                p1m.at<Vec2f>(i)[1] = _p1.at<Vec3f>( matches[i].queryIdx )[1]; //p1[matches[i].queryIdx].y;
				//p1m.at<Vec3f>(i)[2] = 0.0f;

                p2m.at<Vec2f>(i)[0] = _p2.at<Vec3f>( matches[i].trainIdx )[0]; //p2[matches[i].trainIdx].x;
                p2m.at<Vec2f>(i)[1] = _p2.at<Vec3f>( matches[i].trainIdx )[1]; //p2[matches[i].trainIdx].y;
				//p2m.at<Vec3f>(i)[2] = 0.0f;
    }

	H = estimateRigidTransform(p1m, p2m, false);


        Mat p1mt;
        //perspectiveTransform(p1m, p1mt, H);
		transform(p1m, p1mt, H);

        int nr_inliers = 0;
        for (size_t i = 0; i < nr_matches; i++)
        {
                if (norm(p2m.at<Point2f>(i) - p1mt.at<Point2f>(i)) <= maxInlierDist)
                {
                        mask.push_back((short) i);
                        ++nr_inliers;
                }
        }

	//int nr_inliers = countNonZero(tmp_inliers);

	return nr_inliers;
}


void slam_module_frame::imagepoints_to_local3d(vector<Point2f>& src, vector<Point3f>& dst, Mat* state, bool fixed_xy)
{
	Mat cam_pos(3, 1, CV_32F);
	Mat cam_or(3, 1, CV_32F);
	Mat cam_rot(3, 3, CV_32F);

	get_localcam(cam_pos, cam_or, state);
	cam_pos.at<float>(2) *= -1.0f;

	if (fixed_xy)
		cam_pos.at<float>(0) = cam_pos.at<float>(1) = 0.0f;

	cv::RotationMatrix3D(cam_or, cam_rot);

	Mat point(3, 1, CV_32F);
	Mat intersection(3, 1, CV_32F);
	Point3f point3d;

	dst.clear();

	for( size_t i = 0; i < src.size(); i++ )
	{
		point.at<float>(0) = src[i].x;
		point.at<float>(1) = src[i].y;
		point.at<float>(2) = 1.0f;

		point = camera_matrix_inv * point;
		point = cam_rot * point;

		cv::normalize(point, point);

		CalcLinePlaneIntersection(world_plane, world_plane_normal, cam_pos, point, intersection);

		point3d.x = intersection.at<float>(0);
		point3d.y = intersection.at<float>(1);
		//point3d.z = intersection.at<float>(2);
		point3d.z = 0.0f;

		dst.push_back(point3d);
	}
}


void slam_module_frame::get_state(Mat& pos, Mat& or, Mat* statestate_or)
{
	if (state == NULL)
		state = this->state;

	pos.at<float>(0) = state->at<float>(0);
	pos.at<float>(1) = state->at<float>(1);
	pos.at<float>(2) = state->at<float>(2);

	or.at<float>(0) = state->at<float>(9);
	or.at<float>(1) = state->at<float>(10);
	or.at<float>(2) = state->at<float>(11);
}


void slam_module_frame::get_localcam(Mat& pos, Mat& or, Mat* state)
{
	get_state(pos, or, state);

	world_to_localcam(pos, or);
}


void slam_module_frame::localcam_to_world(Mat& pos, Mat& or)
{
	Mat tmp = (Mat_<float>(3,1) << M_PI, M_PI, -0.5f * M_PI);
	Mat rot(3, 3, CV_32F);
	cv::RotationMatrix3D(tmp, rot, false);

	// position
	pos = rot * pos;
	pos.at<float>(2) *= -1.0f;

	// orientation
	or = rot * or;
}


void slam_module_frame::world_to_localcam(Mat& pos, Mat& or)
{
	Mat tmp = (Mat_<float>(3,1) << M_PI, M_PI, 0.5f * M_PI);
	Mat rot(3, 3, CV_32F);
	cv::RotationMatrix3D(tmp, rot);

	// position
	pos = rot * pos;
	pos.at<float>(2) *= -1.0f;
	 
	// orientation
	or = rot * or;
}


void slam_module_frame::localvelocity_to_world(Mat& v)
{
	Mat tmp = (Mat_<float>(3,1) << M_PI, M_PI, -0.5f * M_PI);
	Mat rot(3, 3, CV_32F);
	cv::RotationMatrix3D(tmp, rot, false);

	v = rot * v;
}


void slam_module_frame::get_features(Mat& frame, vector<cv::KeyPoint> &v)
{
	vector<cv::KeyPoint> tmp;

	SURF surf_extractor(SLAM_SURF_HESSIANTHRESHOLD);
	surf_extractor(frame, Mat(), tmp);

	for(size_t i = 0; i < tmp.size(); i++)
	{
		if (tmp[i].response > SLAM_SURF_MIN_RESPONSE)
			v.push_back(tmp[i]);
	}

	// auto convert to imagepoints. Used for other algorithms
	KeyPoint::convert(v, imagepoints);

	features_extracted = true;
}


void slam_module_frame::get_descriptors(Mat& frame, vector<KeyPoint> &v, Mat& descriptors)
{
	de->compute(frame, v, descriptors);

	features_extracted = true;
}


void slam_module_frame::get_matches(Mat& q_descriptors, Mat& t_descriptors, vector<DMatch>& matches, bool use_unique, double max_distance)
{
	if (!use_unique)
	{
		dm.match(q_descriptors, t_descriptors, matches);
	}
	else
	{
		vector<DMatch> matches_tmp;
		std::map<int, int> tIdx_best;
		std::map<int, int>::iterator it;

		dm.match(q_descriptors, t_descriptors, matches_tmp);

		for (size_t i = 0; i < matches_tmp.size(); i++)
		{
			if (matches_tmp[i].distance > 0.12)
				continue;

			it = tIdx_best.find(matches_tmp[i].trainIdx);

			if (it == tIdx_best.end() || matches_tmp[i].distance < matches_tmp[it->second].distance)
			{
				tIdx_best[matches_tmp[i].trainIdx] = i;
			}
		}


		for(it = tIdx_best.begin(); it != tIdx_best.end(); it++)
		{
			matches.push_back(matches_tmp[it->second]);
		}
	}
}


void slam_module_frame::set_camera()
{
	switch (controller->bot_id)
	{
		// USARSim
		case 0x00:
			camera_matrix = 0.0f;
			camera_matrix.at<float>(0, 0) = 141.76401f; //1.60035f;
			camera_matrix.at<float>(1, 1) = 141.689265f; //1.60035f;
			camera_matrix.at<float>(2, 2) = 1.f;
			camera_matrix.at<float>(0, 2) = 88.0f;
			camera_matrix.at<float>(1, 2) = 72.0f;
			break;

		// UvA AR.Drone
		/*
		case 0x01: // 66% size, 1.500000000000150000000000015
			camera_matrix = 0.0f;
			camera_matrix.at<float>(0, 0) = 161.0499991006940328f;
			camera_matrix.at<float>(1, 1) = 196.184482080725745f;
			camera_matrix.at<float>(2, 2) = 1.f;
			camera_matrix.at<float>(0, 2) = 76.3469783848513134f;
			camera_matrix.at<float>(1, 2) = 65.5339900608514809f;
			break;
		*/
		case 0x01:
			camera_matrix = 0.0f;
			camera_matrix.at<float>(0, 0) = 178.944443445215592f;
			camera_matrix.at<float>(1, 1) = 217.98275786747305f;
			camera_matrix.at<float>(2, 2) = 1.f;
			camera_matrix.at<float>(0, 2) = 84.829975983168126f;
			camera_matrix.at<float>(1, 2) = 72.815544512057201f;
			break;

		default:
			printf("ERROR: Bot id not found!\n");

	}

	//camera_matrix *= 0.333333;
	//camera_matrix.at<float>(2, 2 ) = 1.0f;

	camera_matrix_inv = camera_matrix.inv();
}


void slam_module_frame::add_noise(Mat &img)
{

	unsigned char* imagedata = (unsigned char*) img.data;
	unsigned char tmp;
	double value;
	unsigned int sum;

	double brightness = 0.0;
	double contrast = 0.0;

	double contrast_random;

	contrast_random = 0.25 + (0.4 * rand() / RAND_MAX);

	if (((double)rand() / (double)RAND_MAX) > 0.5)
		brightness = 0.1 * ((double)rand() / (double)RAND_MAX);
	else
		brightness = -0.1 * ((double)rand() / (double)RAND_MAX);


	for (int i = 0; i < img.cols * img.rows; i++)
	{
		sum = 0;

		for (int j = 0; j < 3; j++)
		{
			sum += imagedata[i * 4 + j];
		}


		for (int j = 0; j < 3; j++)
		{
			tmp = (unsigned char)imagedata[i * 4 + j];
			value = (double)tmp / 255.0;

			if (brightness < 0.0)
				value = value * ( 1.0 + brightness);
            else
				value = value + ((1 - value) * brightness);

			value = (value - 0.5) * (tan ((contrast + 1) * M_PI/4) ) + 0.5;

			if (sum > 250)
				value = value + (1.0 - value) * contrast_random;

			imagedata[i * 4 + j] = unsigned char(value * 255.0);
		}
	}



	return;
}

/*
void slam_module_frame::descriptor_map_quality()
{
	Mat local;
	int nr_similar = 0;

	Mat *grid = &controller->visual_map.descriptors_grid;

	for (int x = 0; x < grid->cols; x++)
	{
		for (int y = 0; y < grid->rows; y++)
		{
			if (grid->at<unsigned short>(y, x) > 0)
			{
				int A = grid->at<unsigned short>(y, x);

				get_local_descriptors(x, y, local, 5);

				vector<DMatch> matches;

				Mat cur = controller->visual_map.descriptors.row(A);

				dm.match(local, cur, matches);
				
				for (int i = 0; i < (int) matches.size(); i++)
				{
					if (matches[i].distance < 0.15f)
					{
						nr_similar++;
						printf("Found feature that has a duplicate close (%i, %i)\n", x, y);
					}
				}
			}
		}
	}

	printf("Descriptor errors: %i / %i\n", nr_similar, controller->visual_map.descriptors_count - 1);
}


void slam_module_frame::get_local_descriptors(int x, int y, cv::Mat& map_descriptors, int r)
{
	unsigned short *indices;
	unsigned short index;
	int i = 0;

	int x2, y2, w, h;
	x2 = max(0, x - r);
	y2 = max(0, y - r);
	w = min(controller->visual_map.descriptors_grid.cols - x2, r * 2);
	h = min(controller->visual_map.descriptors_grid.rows - y2, r * 2);

	indices = new unsigned short[w * h];

	Mat grid(controller->visual_map.descriptors_grid, Rect(x2, y2, w, h));
	int offsetX = x2;
	int offsetY = y2;

	for (x2 = 0; x2 < grid.cols; x2++)
	{
		for (y2 = 0; y2 < grid.rows; y2++)
		{
			index = grid.at<unsigned short>(y2, x2);
			if (index > 0 && offsetX + x2 != x && offsetY + y2 != y)
			{
				indices[i++] = index;
			}
		}
	}

	map_descriptors	= Mat(i, controller->visual_map.descriptors.cols, controller->visual_map.descriptors.type());

	int descriptors_rowsize	= SLAM_DESCRIPTOR_SIZE;

	for (int j = 0; j < i; j++)
	{
		memcpy_s(map_descriptors.data + j * descriptors_rowsize, descriptors_rowsize,
			controller->visual_map.descriptors.data + indices[j] * descriptors_rowsize, descriptors_rowsize);
	}

	delete [] indices;
}
*/