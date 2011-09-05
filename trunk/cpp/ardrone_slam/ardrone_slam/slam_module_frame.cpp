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


	use_visual = SLAM_MODE(controller->mode, SLAM_MODE_VISUAL);

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
	float MNC[12] = {
		8.0f, 8.0f, 40.0f, // pos
		5.0f, 5.0f, 20.0f, // vel
		4.0f, 4.0f, 10.0f // accel
	};
	MatSetDiag(measurementNoiseCov, MNC);
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


	// frames from the real ardrone are received in RGB order instead of BGR. MOVE TO ARDRONELIB INTERFACE?
	if (!f->usarsim)
		cvtColor( frame, frame, CV_RGB2BGR );


	/* store current estimated position before computing: otherwise timestamp of frame and position do not match! */
	save_cur_state();


	// only add to map
	if (!use_visual)
	{
		add_frame_to_map();
		return;
	}



	// pause sensor module
	controller->sensor_pause(f->time);



	// convert to gray
	cvtColor(frame, frame_gray, CV_BGR2GRAY);


	get_objectpos(obj_pos, obj_or);


	/* find features */
	keypoints.clear(); // necessary?
	current_frame_ip.clear();  // necessary?

	int features_found = find_features(frame_gray, keypoints);
	if (keypoints.size() < 30)
	{
		printf("Not enough features found: dropping frame\n");
		prev_frame_exists = false; // needs some testing?

		save_cur_state();
		controller->sensor_resume();
		add_frame_to_map();
		return;
	}

	KeyPoint::convert(keypoints, current_frame_ip);


	/* calculate descriptors (on greyscale image) */
    de->compute(frame_gray, keypoints, descriptors);


	/* match with previous frame */
	if (prev_frame_exists)
	{
		vector<DMatch> matches;
		dm.match(descriptors, prev_frame_descriptors, matches);

		if (matches.size() < 10)
		{
			//printf("Not enough features matched (%i): dropping frame\n", matches.size());
			save_cur_state();
			controller->sensor_resume();
			store_prev_frame();
			add_frame_to_map();
			return;
		}


		/* find robust matched descriptors (RANSAC) */
		vector<short> mask;
		int nr_inliers = find_robust_matches(current_frame_ip, prev_frame_ip, matches, mask, 30);

		if (nr_inliers < 4)
		{
			//printf("Not enough inliers found (%i): dropping frame\n", nr_inliers);
			save_cur_state();
			controller->sensor_resume();
			store_prev_frame();
			add_frame_to_map();
			return;
		}


		/* retrieve camera motion from two frames */
		find_object_position(obj_pos, obj_or, matches, mask);
		//printf("local pos: %f, %f, %f\n", obj_pos.at<double>(0), obj_pos.at<double>(1), obj_pos.at<double>(2));
		/*
		int nr_inliers = find_object_position(obj_pos, obj_or, matches, mask);
		if (nr_inliers < 4)
		{
			printf("Not enough inliers found (%i): dropping frame\n", nr_inliers);
			return;
		}
		*/

		object_to_worldpos(obj_pos, obj_or, new_pos, new_or);

		//printf("cam or: %f, %f, %f\n", new_or.at<float>(0), new_or.at<float>(1), new_or.at<float>(2));
		//printf("state or: %f, %f, %f\n", state->at<float>(9), state->at<float>(10), state->at<float>(11));


		/* KF measurement */
		memcpy_s(measurement.data, 12, new_pos.data, 12); // pos: this is the fastest method
		difftime = f->time - prev_frame_time; // time between consecutive frames
		calculate_measurement(); // vel & accel: calculated from new pos and previous state


		/* lock KF */
		WaitForSingleObject(controller->KFSemaphore, 0L);


		/* switch KF matrices */
		KF->measurementMatrix	= measurementMatrix;
		KF->measurementNoiseCov	= measurementNoiseCov;


		/* update transition matrix */
		difftime = f->time - controller->KF_prev_update;
		if (difftime < 0.0)
			difftime = 0.00001;

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


	/* store current state (position) as previous state. Used to calculate vel/accel */
	memcpy_s(prev_state.data, 48, state->data, 48); // this is the fastest method


	/* process queued sensor data, before new frame is received */
	controller->sensor_resume();


	/* store current frame as previous frame */
	store_prev_frame();


	/* add frame to canvas */
	add_frame_to_map();
}


void slam_module_frame::add_frame_to_map()
{
	cvtColor(frame, frame_rgba, CV_BGR2BGRA); // convert to RGBA because Direct3D wants a 4 channel array
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


int slam_module_frame::find_robust_matches(vector<Point2f>& p1, vector<Point2f>& p2, vector<DMatch>& matches, vector<short>& mask, int max)
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


	Mat p1mt;
	perspectiveTransform(p1m, p1mt, homography);
	
	double maxInlierDist = 3;
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

	//solvePnPRansac(points3d, imagePoints, camera_matrix, dist_coef, cam_or, cam_pos, true, 100, 3.0, 10 /* TODO */, inliers);

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