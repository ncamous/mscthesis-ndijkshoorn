#include "global.h"
#include "slam_module_frame.h"
#include "bot_ardrone.h"

#include "opencv_helpers.h"
#include <cv.hpp>
#include <cxcore.hpp>
#include <opencv2/calib3d/calib3d.hpp>

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

	measurement(9, 1, CV_32F),
	measurementMatrix(9, 12, CV_32F),
	measurementNoiseCov(9, 9, CV_32F),

	prev_state(12, 1, CV_32F),
	cur_state(12, 1, CV_32F)
{
	this->controller = controller;
	prev_frame_exists = false;

	set_camera();

	//fd = new SurfFeatureDetector(SLAM_SURF_HESSIANTHRESHOLD, 3, 4);
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

	// H vector
	measurementMatrix = 0.0f;
	/*
	for(int i = 0; i < 9; i++)
		measurementMatrix.at<float>(i, i) = 1.0f; // measured pos, velocity and acceleration
	*/
	for(int i = 0; i < 3; i++)
		measurementMatrix.at<float>(i, i) = 1.0f; // measured pos only

	measurementNoiseCov = 0.0f;
	float MNC[9] = {
		200.0f, 200.0f, 200.0f, // pos
		100.0f, 100.0f, 300.0f, // vel
		5.0f, 5.0f, 15.0f // accel
	};
	MatSetDiag(measurementNoiseCov, MNC);


	// tmp
	last_loc = clock();

	first_frame2 = true;

	fopen_s (&loc_log, "dataset/loc_log.txt" , "w");
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


	this->f = f;
	frame.data = (unsigned char*) f->data;



	// reset
	keypoints.clear();
	imagepoints.clear();



	// TODO: attach current state to frame struct, when frame is received
	save_cur_state();



	if (controller->mode(SLAM_MODE_VISUALMOTION))
		process_visual_state();


	if (controller->mode(SLAM_MODE_VISUALLOC))
		process_visual_loc();


	if (controller->mode(SLAM_MODE_MAP))
		process_map();
}


void slam_module_frame::process_visual_state()
{
	// too large velocity
	/*
	if (state->at<float>(3) > 500.0f || state->at<float>(4) > 500.0f)
	{
		printf("Too large velocity: dropping frame\n");
		prev_frame_exists = false; // needs some testing?
		return;
	}
	*/


	// pause sensor module
	controller->sensor_pause(f->time);


	// convert to gray
	cvtColor(frame, frame_gray, CV_BGRA2GRAY);


	get_objectpos(obj_pos, obj_or);


	/* find features */
	if (keypoints.empty())
		get_features(frame_gray, keypoints);

	if (keypoints.size() < 30)
	{
		printf("Not enough features found: dropping frame\n");

		controller->sensor_resume();
		return;
	}


	/* calculate descriptors (on greyscale image) */
	get_descriptors(frame_gray, keypoints, descriptors);


	/* match with previous frame */
	if (prev_frame_exists)
	{
		vector<DMatch> matches2;
		dm.match(descriptors, prev_frame_descriptors, matches2);

		vector<DMatch> matches;
		for (int i = 0; i < (int) matches2.size(); i++)
		{
			if (matches2[i].distance < 0.4f)
				matches.push_back(matches2[i]);
		}

		if (matches.size() < 20)
		{
			printf("Not enough features matched (%i): dropping frame\n", matches.size());
			prev_frame_exists = false;
			controller->sensor_resume();
			store_prev_frame();
			return;
		}


		/* find robust matched descriptors (RANSAC) */
		vector<short> mask;
		Mat H;
		int nr_inliers = find_robust_matches(imagepoints, prev_frame_ip, matches, mask, 20, H, 3.0);
		if (nr_inliers < 6)
		{
			printf("Not enough inliers found (%i): dropping frame\n", nr_inliers);
			controller->sensor_resume();
			store_prev_frame();
			return;
		}


		/* retrieve camera motion from two frames */
		find_object_position(obj_pos, obj_or, matches, mask);

		/*
		int nr_inliers = find_object_position(obj_pos, obj_or, matches, mask);
		if (nr_inliers < 6)
		{
			printf("Not enough inliers found (%i): dropping frame\n", nr_inliers);
			return;
		}
		*/


		object_to_worldpos(obj_pos, obj_or, new_pos, new_or);


		/* KF measurement */
		memcpy_s(measurement.data, 12, new_pos.data, 12); // pos: this is the fastest method
		difftime = f->time - prev_frame_time; // time between consecutive frames
		calculate_measurement(); // vel & accel: calculated from new pos and previous state


		if (measurementSeemsOk())
		{

			/* lock KF */
			EKF->lock();


			/* switch KF matrices */
			EKF->measurementMatrix		= measurementMatrix;
			EKF->measurementNoiseCov	= measurementNoiseCov;


			/* update transition matrix */
			//difftime = f->time - controller->KF_prev_update;
			difftime = EKF->difftime(f->time);


			controller->update_transition_matrix((float) difftime);


			/* predict */
			EKF->predict();


			/* correct */
			EKF->correct(measurement, f->time);

	
			/* save current state. This state is used to project IP and add frame to map */
			save_cur_state();


			/* release KF */
			EKF->release();

		}
		else
		{
			//prev_frame_exists = false;
			printf("Incorrect measurement filtered!\n");
			// current state is not updated
			// drop frame ?
		}

	}


	/* store current state (position) as previous state. Used to calculate vel/accel */
	memcpy_s(prev_state.data, 48, state->data, 48); // this is the fastest method


	/* process queued sensor data, before new frame is received */
	controller->sensor_resume();


	/* store current frame as previous frame */
	store_prev_frame();
}


void slam_module_frame::process_visual_loc()
{
	double dt = ((double)clock() - last_loc) / CLOCKS_PER_SEC;
	if (dt < 0.4)
		return;


	SLAM_LOC_START


	/* get local world coordinates of the image corners */
	vector<Point3f> image_corners_wc;
	imagepoints_to_local3d(image_corners, image_corners_wc);


	/* get descriptors from map */
	Mat map_descriptors;
	Mat map_keypoints_wc;
	float radius = max(cov->at<float>(0, 0), cov->at<float>(1, 1));
	radius += 2500.0f; // just in case. Needs to be removed when KF cov is working fine
	radius += RectRadius(image_corners_wc); // should be dynamic and basic on the state cov

	Mat cam_pos(3, 1, CV_32F);
	Mat cam_or(3, 1, CV_32F);
	get_localcam(cam_pos, cam_or);
	Point3f wc(cam_pos);

	controller->visual_map.get_local_descriptors(map_descriptors, map_keypoints_wc, wc, radius);

	/* not enough descriptors from map (unexplored area?) */
	if (map_descriptors.rows < 3)
	{
		SLAM_LOC_END
		return;
	}



	//Mat frameT;


	// transform frame to canvas
	//controller->visual_map.frame_to_canvas(frame, frameT, image_corners, image_corners_wc);


	/* frame features */
	if (keypoints.empty())
	{
		cvtColor(frame, frame_gray, CV_BGRA2GRAY); // only once!!

		get_features(frame_gray, keypoints); // also stores image points in "imagepoints"
		get_descriptors(frame_gray, keypoints, descriptors);
	}


	/* convert frame (pixel) coordinates to world coordinates (2D) */
	vector<Point2f> imagepoints_wc;
	imagepoints_to_local3d(imagepoints, imagepoints_wc);



	/* match */
	vector<DMatch> matches;
	get_matches(descriptors, map_descriptors, matches, true);

	if (matches.size() < 3)
	{
		SLAM_LOC_END
		return;
	}


	vector<short> inliers;
	Mat T(1, 1, CV_32FC2);
	float R = 0.0f;

	double confidence = find_robust_translation_rotation(imagepoints_wc, map_keypoints_wc, matches, inliers, T, R, 100.0);

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

		cam_pos.at<float>(0) += T.at<Vec2f>(0)[0];
		cam_pos.at<float>(1) += T.at<Vec2f>(0)[1];

		// TMP
		fprintf(loc_log, "%f,%f,%f\n",
			(float) f->time,
			cam_pos.at<float>(0),
			cam_pos.at<float>(1)
		);

		fflush(loc_log);


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
		ReleaseMutex(controller->hMutex);
		last_loc = clock();

		printf("SLAM LOC (%f, %f)   [%f]\n", cam_pos.at<float>(0), cam_pos.at<float>(1), confidence);

#else

		memcpy_s(measurement.data, 12, cam_pos.data, 12); // pos: this is the fastest method
		//difftime = f->time - prev_frame_time; // time between consecutive frames
		//calculate_measurement(); // vel & accel: calculated from new pos and previous state


		//if (measurementSeemsOk())
		//{

		//difftime = f->time - controller->KF_prev_update;
		difftime = EKF->difftime(f->time);


		/* switch KF matrices */
		EKF->measurementMatrix	= measurementMatrix;
		EKF->measurementNoiseCov	= measurementNoiseCov;


		/* update transition matrix */
		controller->update_transition_matrix((float) difftime);

		printf("DIFFTIME: %f\n", (float) difftime);


		/* predict */
		EKF->predict();


		/* correct */
		EKF->correct(measurement, f->time);

	
		/* save current state. This state is used to project IP and add frame to map */
		save_cur_state();


		/* unlock KF */
		EKF->release();
		last_loc = clock();

		printf("SLAM LOC (%f, %f)   [%f]\n", cam_pos.at<float>(0), cam_pos.at<float>(1), confidence);
		//}

#endif

	}


	SLAM_LOC_END
}


void slam_module_frame::process_map()
{
	// flying too low, dont add to map (dark image)
	if (cur_state.at<float>(2) > -200.0f)
		return;

	vector<Point3f> image_corners_wc;
	imagepoints_to_local3d(image_corners, image_corners_wc);

	controller->visual_map.update(frame, image_corners, image_corners_wc);

	/* features map (not-transformed image) */
	if (keypoints.empty())
	{
		cvtColor(frame, frame_gray, CV_BGRA2GRAY); // only once!!

		get_features(frame_gray, keypoints);
		get_descriptors(frame_gray, keypoints, descriptors);

		vector<Point3f> imagepoints_wc;
		imagepoints_to_local3d(imagepoints, imagepoints_wc);

		controller->visual_map.update(keypoints, descriptors, imagepoints_wc);
	}

	first_frame2 = false;
	//first_frame = frame_gray.clone();
}


bool slam_module_frame::measurementSeemsOk()
{
	if (measurement.at<float>(2) > 0.0f)
		return false;

	for (int i = 0; i <= 3; i++)
	{
		if (abs(measurement.at<float>(i) - cur_state.at<float>(i)) > 200.0f)
			return false;
	}

	return true;
}


void slam_module_frame::store_prev_frame()
{
	prev_frame_exists		= true;
	prev_frame_time			= f->time;
	prev_frame_descriptors	= descriptors.clone();
	prev_frame_ip			= imagepoints;
	imagepoints_to_local3d(imagepoints, prev_frame_wc);
}


double slam_module_frame::find_robust_translation_rotation(InputArray p1, InputArray p2, vector<DMatch>& matches, vector<short>& inliers, Mat& T, float& R, double maxInlierDist)
{
	bool res;
	size_t nr_matches = matches.size();

	Mat _p1 = p1.getMat();
	Mat _p2 = p2.getMat();

	Mat p1m(nr_matches, 1, CV_32FC2);
	Mat p2m(nr_matches, 1, CV_32FC2);
	Mat p1mt(nr_matches, 1, CV_32FC2);

    for (size_t i = 0; i < matches.size(); i++)
    {
		p1m.at<Vec2f>(i)[0] = _p1.at<Vec2f>( matches[i].queryIdx )[0];
		p1m.at<Vec2f>(i)[1] = _p1.at<Vec2f>( matches[i].queryIdx )[1];

		p2m.at<Vec2f>(i)[0] = _p2.at<Vec2f>( matches[i].trainIdx )[0];
		p2m.at<Vec2f>(i)[1] = _p2.at<Vec2f>( matches[i].trainIdx )[1];
    }

	Mat p1s(3, 1, CV_32FC2);
	Mat p2s(3, 1, CV_32FC2);
	Point2f *pf1s = (Point2f*) p1s.data;
	Point2f *pf2s = (Point2f*) p2s.data;

	Mat p1s_best(3, 1, CV_32FC2);
	Mat p2s_best(3, 1, CV_32FC2);


	/** RANSAC **/
	Mat T_temp = Mat(1, 1, CV_32FC2);
	Scalar mean, stddev;
	double confidence, best_confidence;
	CvRNG rng = cvRNG(-1);

	best_confidence = 0.0;

	for (int i = 0; i < 800; i++)
	{
		res = getMatSubset(p1m, p2m, p1s, p2s, 300, rng);

		meanStdDev(p2s - p1s, mean, stddev);

		T_temp.at<Vec2f>(0)[0] = (float) mean[0];
		T_temp.at<Vec2f>(0)[1] = (float) mean[1];

		p1mt = p1m + T_temp;

		confidence = 1.0 - (stddev[0] / 200.0) - (stddev[1] / 200.0);

		if (confidence > best_confidence)
		{
			best_confidence = confidence;
			memcpy_s(T.data, sizeof(float) * 2, T_temp.data, sizeof(float) * 2);
			memcpy_s(p1s_best.data, sizeof(float) * 6, p1s.data, sizeof(float) * 6);
			memcpy_s(p2s_best.data, sizeof(float) * 6, p2s.data, sizeof(float) * 6);
		}
	}


	/* Inliers mask */
	p1mt = p1m + T;
	for (size_t i = 0; i < nr_matches; i++)
	{
		if (norm(p2m.at<Point2f>(i) - p1mt.at<Point2f>(i)) <= maxInlierDist)
			inliers.push_back((short) i);
	}


	/* Rotation */
	if (best_confidence > 0.7)
	{
		p1s_best += T; // transform query points to match training points
		R = Kabsch(p1s_best, p2s_best, Mat());
	}

	return best_confidence;
}


int slam_module_frame::find_robust_matches(InputArray p1, InputArray p2, vector<DMatch>& matches, vector<short>& mask, int max, Mat& H, double maxInlierDist)
{
	size_t nr_matches = matches.size();

	Mat _p1 = p1.getMat();
	Mat _p2 = p2.getMat();

	Mat p1m(nr_matches, 1, CV_32FC2);
	Mat p2m(nr_matches, 1, CV_32FC2);

    for (size_t i = 0; i < matches.size(); i++)
    {
		p1m.at<Vec2f>(i)[0] = _p1.at<Vec2f>( matches[i].queryIdx )[0]; //p1[matches[i].queryIdx].x;
		p1m.at<Vec2f>(i)[1] = _p1.at<Vec2f>( matches[i].queryIdx )[1]; //p1[matches[i].queryIdx].y;

		p2m.at<Vec2f>(i)[0] = _p2.at<Vec2f>( matches[i].trainIdx )[0]; //p2[matches[i].trainIdx].x;
		p2m.at<Vec2f>(i)[1] = _p2.at<Vec2f>( matches[i].trainIdx )[1]; //p2[matches[i].trainIdx].y;
    }

	Mat homography = findHomography(p1m, p2m, CV_RANSAC, maxInlierDist);
	memcpy_s(H.data, sizeof(double) * 9, homography.data, sizeof(double) * 9);

	Mat p1mt;
	perspectiveTransform(p1m, p1mt, H);

	int nr_inliers = 0;
	for (size_t i = 0; i < nr_matches; i++)
	{
		if (norm(p2m.at<Point2f>(i) - p1mt.at<Point2f>(i)) <= maxInlierDist)
		{
			mask.push_back((short) i);
			if (++nr_inliers >= max)
				break;
		}
	}

	return nr_inliers;
}



int slam_module_frame::find_object_position(Mat& cam_pos, Mat& cam_or, vector<DMatch>& matches, vector<short>& mask)
{
	Mat points3d(mask.size(), 1, CV_32FC3);
	Mat imagePoints(mask.size(), 1, CV_32FC2);

	//Mat points3d(matches.size(), 1, CV_32FC3);
	//Mat imagePoints(matches.size(), 1, CV_32FC2);

	int src_i;
	for (size_t i = 0; i < mask.size(); i++)
	//for (size_t i = 0; i < matches.size(); i++)
	{
		src_i = matches[mask[i]].queryIdx;
		//src_i = matches[i].queryIdx;
		imagePoints.at<Vec2f>(i)[0] = imagepoints[src_i].x;
		imagePoints.at<Vec2f>(i)[1] = imagepoints[src_i].y;

		src_i = matches[mask[i]].trainIdx;
		//src_i = matches[i].trainIdx;
		points3d.at<Vec3f>(i)[0] = prev_frame_wc[src_i].x;
		points3d.at<Vec3f>(i)[1] = prev_frame_wc[src_i].y;
		points3d.at<Vec3f>(i)[2] = prev_frame_wc[src_i].z;
	}

	Mat dist_coef(5, 1, CV_32F);
	dist_coef = 0.0f;
	vector<int> inliers;

	solvePnP(points3d, imagePoints, camera_matrix, dist_coef, cam_or, cam_pos, true);

	//solvePnPRansac(points3d, imagePoints, camera_matrix, dist_coef, cam_or, cam_pos, true, 150, 3.0, 20 /* TODO */, inliers);

	return inliers.size();
}


void slam_module_frame::calculate_measurement()
{
	float dt = (float) difftime;

	// vel
	measurement.at<float>(3) = (measurement.at<float>(0) - prev_state.at<float>(0)) / dt;
	measurement.at<float>(4) = (measurement.at<float>(1) - prev_state.at<float>(1)) / dt;
	measurement.at<float>(5) = (measurement.at<float>(2) - prev_state.at<float>(2)) / dt;

	// accel
	measurement.at<float>(6) = (measurement.at<float>(3) - prev_state.at<float>(3)) / dt;
	measurement.at<float>(7) = (measurement.at<float>(4) - prev_state.at<float>(4)) / dt;
	measurement.at<float>(8) = (measurement.at<float>(5) - prev_state.at<float>(5)) / dt;
}


void slam_module_frame::save_cur_state()
{
	memcpy_s(cur_state.data, 12 * 4, state->data, 12 * 4);
}


void slam_module_frame::imagepoints_to_local3d(vector<Point2f>& src, vector<Point3f>& dst)
{
	Mat cam_pos(3, 1, CV_32F);
	Mat cam_or(3, 1, CV_32F);
	Mat cam_rot(3, 3, CV_32F);

	get_localcam(cam_pos, cam_or);
	cam_pos.at<float>(2) *= -1.0f;

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
		point3d.z = intersection.at<float>(2);

		dst.push_back(point3d);
	}
}


void slam_module_frame::imagepoints_to_local3d(vector<Point2f>& src, vector<Point2f>& dst)
{
	Mat cam_pos(3, 1, CV_32F);
	Mat cam_or(3, 1, CV_32F);
	Mat cam_rot(3, 3, CV_32F);

	get_localcam(cam_pos, cam_or);
	cam_pos.at<float>(2) *= -1.0f;

	cv::RotationMatrix3D(cam_or, cam_rot);

	Mat point(3, 1, CV_32F);
	Mat intersection(3, 1, CV_32F);
	Point2f point2d;

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

		point2d.x = intersection.at<float>(0);
		point2d.y = intersection.at<float>(1);

		dst.push_back(point2d);
	}
}



void slam_module_frame::get_state(Mat& pos, Mat& or)
{
	pos.at<float>(0) = cur_state.at<float>(0);
	pos.at<float>(1) = cur_state.at<float>(1);
	pos.at<float>(2) = cur_state.at<float>(2);

	or.at<float>(0) = cur_state.at<float>(9);
	or.at<float>(1) = cur_state.at<float>(10);
	or.at<float>(2) = cur_state.at<float>(11);
}


void slam_module_frame::get_localcam(Mat& pos, Mat& or)
{
	get_state(pos, or);

	world_to_localcam(pos, or);
}


void slam_module_frame::object_to_worldpos(Mat& obj_pos, Mat& obj_or, Mat& pos, Mat& or)
{
	Mat rot(3, 3, CV_32F);

	MatDoubleToFloat(obj_pos, pos);
	MatDoubleToFloat(obj_or, or);

	object_to_localcam(pos, or);
	localcam_to_world(pos, or);
}


void slam_module_frame::get_objectpos(Mat& pos, Mat& or)
{
	Mat tmp_pos(3, 1, CV_32F);
	Mat tmp_or(3, 1, CV_32F);

	get_state(tmp_pos, tmp_or);

	world_to_localcam(tmp_pos, tmp_or);
	localcam_to_object(tmp_pos, tmp_or);

	MatFloatToDouble(tmp_pos, pos);
	MatFloatToDouble(tmp_or, or);
}



/** HELPERS **/
void slam_module_frame::object_to_localcam(Mat& pos, Mat& or)
{
	//dumpMatrix(or);

	/*
	or.at<float>(0) = 0.0f;
	or.at<float>(1) = 0.0f;
	or.at<float>(2) = 0.0f;
	*/

	/*
	or.at<float>(0) = cur_state.at<float>(9);
	or.at<float>(1) = cur_state.at<float>(10);
	or.at<float>(2) = cur_state.at<float>(11);

	Mat tmp = (Mat_<float>(3,1) << M_PI, M_PI, 0.5f * M_PI);
	Mat rot2(3, 3, CV_32F);
	cv::RotationMatrix3D(tmp, rot2);

	or = rot2 * or;
	*/

	//pos.at<float>(2) = -cur_state.at<float>(2);

	//printf("solvePNP POS: %f, %f, %f\n", pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
	//printf("OR: %f, %f, %f\n", cur_state.at<float>(9), cur_state.at<float>(10));

	TransformationMatrix(pos, or, T);
	Mat T_inv = T.inv();

	//
	//Mat rot(T_inv, Rect(0, 0, 3, 3));
	//cv::RotationMatrix3D(or, rot);
	//

	Mat posH = T_inv * originH;

	pos.at<float>(0) = posH.at<float>(0);
	pos.at<float>(1) = posH.at<float>(1);
	pos.at<float>(2) = -posH.at<float>(2);

	//float tmp = pos.at<float>(0);

	/*
	pos.at<float>(0) = -pos.at<float>(0);
	pos.at<float>(1) = -pos.at<float>(1);
	*/


	/* calculate position shift caused by cam orientation */
	/*
	Mat cam_pos(3, 1, CV_32F);
	Mat cam_or(3, 1, CV_32F);
	Mat cam_rot(3, 3, CV_32F);

	get_localcam(cam_pos, cam_or);
	cam_pos.at<float>(0) = 0.0f;
	cam_pos.at<float>(1) = 0.0f;
	cam_pos.at<float>(2) *= -1.0f;

	cv::RotationMatrix3D(cam_or, cam_rot);

	Mat point(3, 1, CV_32F);
	Mat intersection(3, 1, CV_32F);

	point.at<float>(0) = 0.0f;
	point.at<float>(1) = 0.0f;
	point.at<float>(2) = 1.0f;

	point = cam_rot * point;

	cv::normalize(point, point);

	CalcLinePlaneIntersection(world_plane, world_plane_normal, cam_pos, point, intersection);

	//dumpMatrix(intersection);

	cam_pos.at<float>(0) -= intersection.at<float>(0);
	cam_pos.at<float>(1) -= intersection.at<float>(1);
	*/

	//printf("pos: %f, %f\n", pos.at<float>(0), pos.at<float>(1));
	//printf("shift: %f, %f\n", intersection.at<float>(0), intersection.at<float>(1));

	//Mat rot(T_inv, Rect(0, 0, 3, 3));
	//Rodrigues(rot, or); // rotation matrix to rotation vector
}


void slam_module_frame::localcam_to_object(Mat& pos, Mat& or)
{
	TransformationMatrix(pos, or, T);
	Mat T_inv = T.inv();

	Mat posH = T_inv * originH;

	pos.at<float>(0) = posH.at<float>(0);
	pos.at<float>(1) = posH.at<float>(1);
	pos.at<float>(2) = -posH.at<float>(2);

	Mat rot(T_inv, Rect(0, 0, 3, 3));
	Rodrigues(rot, or); // rotation matrix to rotation vector
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
/***/



void slam_module_frame::get_features(Mat& frame, vector<cv::KeyPoint> &v)
{
	vector<cv::KeyPoint> tmp;

	SURF surf_extractor(SLAM_SURF_HESSIANTHRESHOLD);
	surf_extractor(frame, Mat(), tmp);

	for(size_t i = 0; i < tmp.size(); i++)
	{
		if (tmp[i].response > 1000.0f)
			v.push_back(tmp[i]);
	}

	// auto convert to imagepoints. Used for other algorithms
	KeyPoint::convert(v, imagepoints);
}


void slam_module_frame::get_descriptors(Mat& frame, vector<KeyPoint> &v, Mat& descriptors)
{
	de->compute(frame, v, descriptors);
}


void slam_module_frame::get_matches(Mat& q_descriptors, Mat& t_descriptors, vector<DMatch>& matches, bool use_unique)
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
		case 0x01:
			camera_matrix = 0.0f;
			camera_matrix.at<float>(0, 0) = 198.82715938357288f;
			camera_matrix.at<float>(1, 1) = 217.98275786747305f;
			camera_matrix.at<float>(2, 2) = 1.f;
			camera_matrix.at<float>(0, 2) = 84.829975983168126f;
			camera_matrix.at<float>(1, 2) = 72.815544512057201f;
			break;

		default:
			printf("ERROR: Bot id not found!\n");

	}

	camera_matrix_inv = camera_matrix.inv();
}


void slam_module_frame::add_noise(IplImage *img)
{

	unsigned char* imagedata = (unsigned char*) img->imageData;
	unsigned char tmp;
	double value;
	unsigned int sum;

	double brightness = 0.0;
	double contrast = 0.0;

	double contrast_random;

	contrast_random = 0.4 + (0.5 * rand() / RAND_MAX);

	if (((double)rand() / (double)RAND_MAX) > 0.5)
		brightness = 0.12 * ((double)rand() / (double)RAND_MAX);
	else
		brightness = -0.12 * ((double)rand() / (double)RAND_MAX);


	for (int i = 0; i < img->width * img->height; i++)
	{
		sum = 0;

		for (int j = 0; j < 3; j++)
		{
			sum += imagedata[i * 3 + j];
		}


		for (int j = 0; j < 3; j++)
		{
			tmp = (unsigned char)imagedata[i * 3 + j];
			value = (double)tmp / 255.0;

			if (brightness < 0.0)
				value = value * ( 1.0 + brightness);
            else
				value = value + ((1 - value) * brightness);

			value = (value - 0.5) * (tan ((contrast + 1) * M_PI/4) ) + 0.5;

			if (sum > 250)
				value = value + (1.0 - value) * contrast_random;

			imagedata[i * 3 + j] = unsigned char(value * 255.0);
		}
	}



	return;
}


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