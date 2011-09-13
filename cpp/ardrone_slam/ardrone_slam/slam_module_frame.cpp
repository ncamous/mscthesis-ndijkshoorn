#include "global.h"
#include "slam_module_frame.h"
#include "bot_ardrone.h"

#include "opencv_helpers.h"
#include <cv.hpp>
#include <cxcore.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;


slam_module_frame::slam_module_frame(slam *controller):
	camera_matrix(3, 3, CV_32F),
	world_plane(3, 1, CV_32F),
	world_plane_normal(3, 1, CV_32F),

	T(4, 4, CV_32F),
	originH(4, 1, CV_32F),

	frame(BOT_ARDRONE_CAM_RESOLUTION_H, BOT_ARDRONE_CAM_RESOLUTION_W, CV_8UC3, NULL, 0), // do not want to allocate data here. HOW?
	frame_rgba(BOT_ARDRONE_CAM_RESOLUTION_H, BOT_ARDRONE_CAM_RESOLUTION_W, CV_8UC4),
	frame_gray(BOT_ARDRONE_CAM_RESOLUTION_H, BOT_ARDRONE_CAM_RESOLUTION_W, CV_8U),

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
	image_corners.push_back(Point2f(0.0f, (float) (BOT_ARDRONE_CAM_RESOLUTION_H - 1)));
	image_corners.push_back(Point2f((float) (BOT_ARDRONE_CAM_RESOLUTION_W - 1), (float) (BOT_ARDRONE_CAM_RESOLUTION_H - 1)));
	image_corners.push_back(Point2f((float) (BOT_ARDRONE_CAM_RESOLUTION_W - 1), 0.0f));


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
	KF = &controller->KF;
	state = &KF->statePost;

	// H vector
	measurementMatrix = 0.0f;
	for(int i = 0; i < 9; i++)
		measurementMatrix.at<float>(i, i) = 1.0f; // measured pos, velocity and acceleration

	measurementNoiseCov = 0.0f;
	float MNC[9] = {
		50.0f, 50.0f, 500.0f, // pos
		100.0f, 100.0f, 300.0f, // vel
		5.0f, 5.0f, 15.0f // accel
	};
	MatSetDiag(measurementNoiseCov, MNC);


	// tmp
	last_loc = clock();
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
	if (!controller->KF_running)
		return;


	this->f = f;
	frame.data = (uchar*) &f->data[4];


	// convert to RGBA because Direct3D wants a 4 channel array
	if (f->usarsim)
		cvtColor(frame, frame_rgba, CV_BGR2BGRA);
	else
		cvtColor(frame, frame_rgba, CV_RGB2BGRA);


	// TODO: attach current state to frame struct, when frame is received
	save_cur_state();


	if (controller->mode(SLAM_MODE_VISUALMOTION))
		process_visual_state();


	if (controller->mode(SLAM_MODE_VISUALLOC))
		process_visual_loc();


	if (controller->mode(SLAM_MODE_MAP))
		add_frame_to_map();
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
	keypoints.clear(); // necessary?
	current_frame_ip.clear();  // necessary?

	int features_found = find_features(frame_gray, keypoints);
	//printf("Nr features: %i\n", keypoints.size());
	if (keypoints.size() < 30)
	{
		printf("Not enough features found: dropping frame\n");

		controller->sensor_resume();
		return;
	}

	KeyPoint::convert(keypoints, current_frame_ip);

	for (size_t i = 0; i < keypoints.size(); i++)
		circle(frame_rgba, keypoints[i].pt, 3, Scalar(0,0,255), 1, 8);


	/* calculate descriptors (on greyscale image) */
    de->compute(frame_gray, keypoints, descriptors);


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
		int nr_inliers = find_robust_matches(current_frame_ip, prev_frame_ip, matches, mask, 20, H, 3.0);
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
			WaitForSingleObject(controller->KFSemaphore, 0L);


			/* switch KF matrices */
			KF->measurementMatrix	= measurementMatrix;
			KF->measurementNoiseCov	= measurementNoiseCov;


			/* update transition matrix */
			difftime = f->time - controller->KF_prev_update;
			if (difftime <= 0.0)
				difftime = 0.0001;

			controller->update_transition_matrix((float) difftime);


			/* predict */
			KF->predict();


			/* correct */
			KF->correct(measurement);

	
			/* save current state. This state is used to project IP and add frame to map */
			save_cur_state();


			/* release KF */
			controller->KF_prev_update = f->time;
			ReleaseSemaphore(controller->KFSemaphore, 1, NULL);

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
	if (dt < 3.0)
		return;


	vector<CornerHist> corners;
	vector<Point2f> corners_p;
	vector<Point3f> image_corners_wc;
	Mat frameT;

	//controller->sensor_pause(f->time);

	// get local world coordinates of the image corners
	imagepoints_to_local3d(image_corners, image_corners_wc);

	// transform frame to canvas
	controller->visual_map.frame_to_canvas(frame, frameT, image_corners, image_corners_wc);


	// find corners in transformed frame
	controller->visual_map.find_corners(frameT, corners, corners_p, false);

	//printf("frame corners: %i\n", corners.size());

	// match (P) histograms + distance
	//printf("Matching\n");
	vector<DMatch> matches;
	
	Mat cam_pos(3, 1, CV_32F);
	Mat cam_or(3, 1, CV_32F);
	get_localcam(cam_pos, cam_or);

	CornerHistMatch(corners, controller->visual_map.corners, matches, (float*) cam_pos.data);

	/*
	if (matches.size() < 4)
	{
		//printf("%i matches: 1 point LOC\n", matches.size());

		size_t best_i;
		float best_distance = 99999;

		// find best match
		for (size_t i = 0; i < matches.size(); i++)
		{
			if (matches[i].distance < best_distance)
			{
				best_i			= i;
				best_distance	= matches[i].distance;
			}
		}

		if (best_distance <= 0.3)
		{

			int qIdx = matches[ best_i ].queryIdx;
			int tIdx = matches[ best_i ].trainIdx;

			Point2f translate = corners[ qIdx ].wc - controller->visual_map.corners[ tIdx ].wc;
			//translate.x = corners[ qIdx ].wc.x - controller->visual_map.corners[ tIdx ].wc.x;
			//translate.y = corners[ qIdx ].wc.y - controller->visual_map.corners[ tIdx ].wc.y;

			//printf("%f, %f\n", translate.x, translate.y);

			cam_pos.at<float>(0) -= translate.x;
			cam_pos.at<float>(1) -= translate.y;

			WaitForSingleObject(controller->KFSemaphore, 0L);

			localcam_to_world(cam_pos, cam_or);

			state->at<float>(0) = cam_pos.at<float>(0);
			state->at<float>(1) = cam_pos.at<float>(1);

			ReleaseSemaphore(controller->KFSemaphore, 1, NULL);
			last_loc = clock();

			printf("performed LOC update\n");

		}

	}
	else*/
	if (matches.size() >= 4)
	{
		/*
		for (int i = 0; i < (int) matches.size(); i++)
		{
			printf("query %i -> %i (%f)\n", matches[i].queryIdx, matches[i].trainIdx, matches[i].distance);
		}
		*/

		// ransac for inliers
		vector<short> mask;
		vector<Point2f> ip;
		Mat H(3, 3, CV_64F);

		int nr_inliers = find_robust_matches(corners_p, controller->visual_map.corners_p, matches, mask, 50, H, 8.0);
		//printf("found %i - inliers: %i\n", corners.size(), nr_inliers);
		if (nr_inliers > 3)
		{
			float translate[2];
			translate[0] = (float) H.at<double>(0, 2);
			translate[1] = (float) H.at<double>(1, 2);

			dumpMatrix(H);

			//printf("%f, %f\n", translate[0], translate[1]);

			//printf("state: %f, %f\n", state->at<float>(0), state->at<float>(1));

			cam_pos.at<float>(0) += translate[0];
			cam_pos.at<float>(1) += translate[1];

			/* lock KF */
			WaitForSingleObject(controller->KFSemaphore, 0L);

			localcam_to_world(cam_pos, cam_or);

			state->at<float>(0) = cam_pos.at<float>(0);
			state->at<float>(1) = cam_pos.at<float>(1);

			/* unlock KF */
			ReleaseSemaphore(controller->KFSemaphore, 1, NULL);
			last_loc = clock();

			printf("!!!!! performed LOC update (multiple)\n");
		}
	}

	controller->sensor_resume();
}


void slam_module_frame::CornerHistMatch(vector<CornerHist>& query_descriptors, vector<CornerHist>& train_descriptors, vector<DMatch>& matches, float *pos)
{
	matches.reserve(query_descriptors.size());
	size_t query_size = query_descriptors.size();
	size_t train_size = train_descriptors.size();
	Point2f estimated_pos(pos[0], pos[1]);
	double score;
	double min_distance;
	int best_iIdx;

    for( size_t qIdx = 0; qIdx < query_size; qIdx++ )
    {
			min_distance = 99999.0;

            for( size_t iIdx = 0; iIdx < train_size; iIdx++ )
            {
                score = 1.0 - compareHist( query_descriptors[qIdx].hist, train_descriptors[iIdx].hist, 0 );
				if (score > 0.1)
					continue;

				score += norm(estimated_pos - train_descriptors[iIdx].wc) / 10000.0f;

				//printf("[%i,%i] = %f\n", qIdx, iIdx, score);

				if (score < min_distance)
				{
					min_distance = score;
					best_iIdx = iIdx;
				}
            }

			if (min_distance < 0.5f)
			{
				matches.push_back( DMatch( qIdx, best_iIdx, (float) min_distance ) );
			}
            //std::sort( curMatches->begin(), curMatches->end() );
    }
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


void slam_module_frame::add_frame_to_map()
{
	// flying too low, dont add to map (dark image)
	if (cur_state.at<float>(2) > -250.0f)
		return;

	vector<Point3f> image_corners_wc;
	imagepoints_to_local3d(image_corners, image_corners_wc);
	controller->visual_map.update(frame_rgba, image_corners, image_corners_wc);
}


void slam_module_frame::store_prev_frame()
{
	prev_frame_exists		= true;
	prev_frame_time			= f->time;
	prev_frame_descriptors	= descriptors.clone();
	prev_frame_ip			= current_frame_ip;
	imagepoints_to_local3d(current_frame_ip, prev_frame_wc);
}


int slam_module_frame::find_robust_matches(vector<Point2f>& p1, vector<Point2f>& p2, vector<DMatch>& matches, vector<short>& mask, int max, Mat& H, double maxInlierDist)
{
	size_t nr_matches = matches.size();

	Mat p1m(nr_matches, 1, CV_32FC2);
	Mat p2m(nr_matches, 1, CV_32FC2);

    for (size_t i = 0; i < matches.size(); i++)
    {
		p1m.at<Vec2f>(i)[0] = p1[matches[i].queryIdx].x;
		p1m.at<Vec2f>(i)[1] = p1[matches[i].queryIdx].y;

		p2m.at<Vec2f>(i)[0] = p2[matches[i].trainIdx].x;
		p2m.at<Vec2f>(i)[1] = p2[matches[i].trainIdx].y;
    }

	Mat homography = findHomography(p1m, p2m, CV_RANSAC);
	memcpy_s(H.data, sizeof(double) * 9, homography.data, sizeof(double) * 9);

	Mat p1mt;
	perspectiveTransform(p1m, p1mt, H);
	
	//double maxInlierDist = 3.0;
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
		imagePoints.at<Vec2f>(i)[0] = current_frame_ip[src_i].x;
		imagePoints.at<Vec2f>(i)[1] = current_frame_ip[src_i].y;

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

	Mat tmp = (Mat_<float>(3,1) << PI, PI, 0.5f * PI);
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
	Mat tmp = (Mat_<float>(3,1) << PI, PI, -0.5f * PI);
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
	Mat tmp = (Mat_<float>(3,1) << PI, PI, 0.5f * PI);
	Mat rot(3, 3, CV_32F);
	cv::RotationMatrix3D(tmp, rot);

	// position
	pos = rot * pos;
	pos.at<float>(2) *= -1.0f;

	// orientation
	or = rot * or;
}
/***/



int slam_module_frame::find_features(Mat& frame, vector<cv::KeyPoint> &v)
{
	//if (SLAM_USE_OBSTACLE_MASK)
	//	calculate_frame_mask(img->width, img->height);

	// frame_mask is ignored when empty
	//if (SLAM_USE_OBSTACLE_MASK)
	//	fd->detect(img, v, frame_mask);
	//else

	//SurfFeatureDetector blup = new SurfFeatureDetector(SLAM_SURF_HESSIANTHRESHOLD, 3, 4);
	//fd->detect(img, v);

	SURF surf_extractor(SLAM_SURF_HESSIANTHRESHOLD);
	surf_extractor(frame, Mat(), v);

	return v.size();
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

		// Oldest UvA AR.Drone
		case 0x01:
			camera_matrix = 0.0f;
			camera_matrix.at<float>(0, 0) = 197.31999258f; //1.60035f;
			camera_matrix.at<float>(1, 1) = 215.24223662f; //1.60035f;
			camera_matrix.at<float>(2, 2) = 1.f;
			camera_matrix.at<float>(0, 2) = 85.748438497f;
			camera_matrix.at<float>(1, 2) = 69.141653496f;
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

			value = (value - 0.5) * (tan ((contrast + 1) * PI/4) ) + 0.5;

			if (sum > 250)
				value = value + (1.0 - value) * contrast_random;

			imagedata[i * 3 + j] = unsigned char(value * 255.0);
		}
	}



	return;

}