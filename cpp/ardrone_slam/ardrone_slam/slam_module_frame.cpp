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
	world_plane_normal(3, 1, CV_32F)
{
	this->controller = controller;

	fd = new SurfFeatureDetector(SLAM_SURF_HESSIANTHRESHOLD, 3, 4);
	de = new SurfDescriptorExtractor();


	/* camera matrix (USARSim) */
	camera_matrix = 0.0f;
	camera_matrix.at<float>(0, 0) = 141.76401f; //1.60035f;
	camera_matrix.at<float>(1, 1) = 141.689265f; //1.60035f;
	camera_matrix.at<float>(2, 2) = 1.f;
	camera_matrix.at<float>(0, 2) = 88.0f;
	camera_matrix.at<float>(1, 2) = 72.0f;

	camera_matrix_inv = camera_matrix.inv();
	/**/


	/* world plane */
	world_plane = 0.0f;
	world_plane_normal = 0.0f;
	world_plane_normal.at<float>(2) = 1.0f;


	frame = NULL;
	prev_frame_descriptors = NULL;

	dropped_frame_counter = 0;
	frame_counter = 0;
	feature_distance = 0.0;

	prev_frame_h = Mat::eye(3, 3, CV_64F);
	double* h_data = (double*) prev_frame_h.data;
	h_data[2] = 300.0;
	h_data[5] = 300.0;

	if (SLAM_BUILD_OBSTACLE_MAP)
	{
		obstacle_map = Mat(800, 800, CV_8UC1);
		obstacle_map = 255;
	}

	/* KF */
	KF = &controller->KF;
	state = &KF->statePost;
}


slam_module_frame::~slam_module_frame(void)
{
}

void slam_module_frame::process(bot_ardrone_frame *f)
{
	// scale not kwown: no use of processing the frame
	if (!controller->KF_running)
		return;


	// initialize frame
	if (frame == NULL)
	{
		unsigned short w, h;

		memcpy_s(&w, 2, &f->data[0], 2);
		memcpy_s(&h, 2, &f->data[2], 2);

		w = ntohs(w);
		h = ntohs(h);

		frame = cvCreateImageHeader(cvSize(w, h), IPL_DEPTH_8U, 3);
		gray = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);
	}

	frame->imageData = &f->data[4];


	/*
	char s[100];
	sprintf(s, "blup_%i.png\0", frame_counter++); 

	Mat img(frame);
	imwrite(string(s), img);
	//imwrite(s, frame);
	return;
	*/


	// frames from the real ardrone are received in RGB order instead of BGR
	if (!f->usarsim)
		cvCvtColor( frame, frame, CV_RGB2BGR );



	/* find features */
	vector<cv::KeyPoint> keypoints;
	int features_found = find_features(frame, keypoints);

	vector<Point2f> current_frame_ip; // current frame image points
	KeyPoint::convert(keypoints, current_frame_ip);



	/* calculate descriptors (on greyscale image) */
	Mat descriptors;
	cvCvtColor(frame, gray, CV_RGB2GRAY);
    de->compute(gray, keypoints, descriptors);




	/* match with previous frame */
	if (frame_counter > 0)
	{

		if (keypoints.size() < 20)
		{
			printf("Not enough features found: dropping frame\n");
			return;
		}

		vector<DMatch> matches;
		dm.match(descriptors, prev_frame_descriptors, matches);

		if (matches.size() < 20)
		{
			printf("Not enough features matched (%i): dropping frame\n", matches.size());
			return;
		}


		/* retrieve camera motion from two frames */
		size_t nr_matches = matches.size();

		Mat points3d(nr_matches, 1, CV_32FC3);
		Mat imagePoints(nr_matches, 1, CV_32FC2);

		int src_i;
		for (size_t i = 0; i < nr_matches; i++)
		{
			src_i = matches[i].queryIdx;

			imagePoints.at<Vec2f>(i)[0] = current_frame_ip[src_i].x;
			imagePoints.at<Vec2f>(i)[1] = current_frame_ip[src_i].y;

			src_i = matches[i].trainIdx;

			points3d.at<Vec3f>(i)[0] = prev_frame_wc[src_i].x;
			points3d.at<Vec3f>(i)[1] = prev_frame_wc[src_i].y;
			points3d.at<Vec3f>(i)[2] = prev_frame_wc[src_i].z;
		}

		Mat dist_coef(5, 1, CV_32F);
		Mat rotation_vector_calc(3, 1, CV_64F);
		Mat translation_vector_calc(3, 1, CV_64F);
		dist_coef = 0.0f;


		solvePnP(points3d, imagePoints, camera_matrix, dist_coef, rotation_vector_calc, translation_vector_calc);

		dumpMatrix(translation_vector_calc);
		printf("\n");
		//dumpMatrix(rotation_vector_calc);
		//printf("\n\n\n");

		Sleep(500);
	}



	/* store current frame as previous frame */
	//prev_frame_keypoints = keypoints;
	prev_frame_descriptors = descriptors;
	// prev_frame_wc is set below

	//Point2f tmp(88.0f, 72.0f);
	//src.push_back(tmp);

	/*
	printf("===== INPUT =====\n");
	for( size_t i = 0; i < current_frame_ip.size(); i++ )
	{
		printf("%f, %f\n", current_frame_ip[i].x, current_frame_ip[i].y);
	}
	*/

	imagepoints_to_world3d(current_frame_ip, prev_frame_wc);

	/*
	printf("\n");
	printf("===== OUTPUT (%i) =====\n", prev_frame_wc.size());
	for( size_t i = 0; i < prev_frame_wc.size(); i++ )
	{
		printf("%f, %f, %f\n", prev_frame_wc[i].x, prev_frame_wc[i].y, prev_frame_wc[i].z);
	}

	printf("\n\n");
	*/

	frame_counter++;
}


void slam_module_frame::imagepoints_to_world3d(vector<Point2f>& src, vector<Point3f>& dst)
{
	// get camera (vectors)
	Mat cam_pos(3, 1, CV_32F);
	Mat cam_rotation(3, 1, CV_32F);
	get_current_camera(cam_pos, cam_rotation);

	// 3D rotation matrix
	Mat rotation_matrix(3, 3, CV_32F);
	cv::RotationMatrix3D(cam_rotation, rotation_matrix);

	Mat point(3, 1, CV_32F);
	Mat intersection(3, 1, CV_32F);
	Point3f point3d;

	dst.clear();
	//dst.resize(src.size());

	for( size_t i = 0; i < src.size(); i++ )
	{
		point.at<float>(0) = src[i].x;
		point.at<float>(1) = src[i].y;
		point.at<float>(2) = 1.0f;

		point = camera_matrix_inv * point;
		point = rotation_matrix * point;

		cv::normalize(point, point);

		CalcLinePlaneIntersection(world_plane, world_plane_normal, cam_pos, point, intersection);

		point3d.x = intersection.at<float>(0);
		point3d.y = intersection.at<float>(1);
		point3d.z = intersection.at<float>(2);

		dst.push_back(point3d);
	}
}


void slam_module_frame::get_current_camera(Mat& pos, Mat& orientation)
{
	// test data
	/*
	float test_cam_p[3] = {0.0f, 0.0f, -1000.0f};
	float test_cam_o[3] = {0.0f-PI, 0.0f-PI, -0.5f * PI};
	*/

	pos.at<float>(0) = state->at<float>(0);
	pos.at<float>(1) = state->at<float>(1);
	pos.at<float>(2) = state->at<float>(2);

	orientation.at<float>(0) = state->at<float>(9);
	orientation.at<float>(1) = state->at<float>(10);
	orientation.at<float>(2) = state->at<float>(11);
}


void slam_module_frame::process(IplImage *i)
{
	bot_ardrone_frame *f1 = new bot_ardrone_frame;

	int datasize = i->width * i->height * 3;
	char data[999999];

	unsigned short w, h;
	w = htons(i->width);
	h = htons(i->height);

	memcpy(&data[0], &w, 2);
	memcpy(&data[2], &h, 2);
	memcpy(&data[4], i->imageData, datasize);

	f1->data = data;

	process(f1);
}


int slam_module_frame::find_features(IplImage *img, vector<cv::KeyPoint> &v)
{
	if (SLAM_USE_OBSTACLE_MASK)
		calculate_frame_mask(img->width, img->height);

	// frame_mask is ignored when empty
	if (SLAM_USE_OBSTACLE_MASK)
		fd->detect(img, v, frame_mask);
	else
		fd->detect(img, v);

	//printf("found %i features\n", v.size());

	return v.size();
}


void slam_module_frame::calculate_frame_mask(int width, int height)
{
	IplImage *mask_img = cvCreateImage(cvSize(width, height), 8, 1);
	Mat h_inverse = prev_frame_h.inv();
	CvMat invHomography = h_inverse;

	IplImage *obstacle_map_img = cvCreateImageHeader(cvSize(800, 800), 8, 1);
	obstacle_map_img->imageData = (char*)obstacle_map.data;

	cvWarpPerspective(obstacle_map_img, mask_img, &invHomography, CV_INTER_LINEAR);

	frame_mask = Mat(mask_img, true); // copy

	//imshow("Mask:", mask_img);
	cvWaitKey(4);
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