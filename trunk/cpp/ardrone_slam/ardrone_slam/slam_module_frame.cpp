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

	measurementMatrix(3, 12, CV_32F),
	measurementNoiseCov(3, 3, CV_32F)
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
	world_plane_normal.at<float>(2) = -1.0f;


	frame = NULL;
	prev_frame_descriptors = NULL;

	dropped_frame_counter = 0;
	frame_counter = 0;
	feature_distance = 0.0;

	if (SLAM_BUILD_OBSTACLE_MAP)
	{
		obstacle_map = Mat(800, 800, CV_8UC1);
		obstacle_map = 255;
	}




	/* KF */
	KF = &controller->KF;
	state = &KF->statePost;

	// H vector
	measurementMatrix = 0.0f;
	for(int i = 0; i < 3; i++)
	{
		measurementMatrix.at<float>(i, i) = 1.0f; // measured a
	}

	measurementNoiseCov = 0.0f;
	setIdentity(measurementNoiseCov, Scalar::all(1e-2));
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

		if (matches.size() < 10)
		{
			printf("Not enough features matched (%i): dropping frame\n", matches.size());
			return;
		}


		/* find robust matched descriptors (RANSAC) */
		vector<char> matchesMask(matches.size(), 0);
		int nr_inliers = find_robust_matches(current_frame_ip, prev_frame_ip, matches, matchesMask);
		int nr_matches_used = min(10, nr_inliers);


		if (nr_inliers < 4)
		{
			printf("Not enough inliers found (%i): dropping frame\n", nr_inliers);
			return;
		}


		/* retrieve camera motion from two frames */
		Mat points3d(nr_matches_used, 1, CV_32FC3);
		Mat imagePoints(nr_matches_used, 1, CV_32FC2);

		int src_i;
		int j = 0;
		for (size_t i = 0; i < matches.size(); i++)
		{
			if (matchesMask[i] == 0)
				continue;

			src_i = matches[i].queryIdx;
			imagePoints.at<Vec2f>(j)[0] = current_frame_ip[src_i].x;
			imagePoints.at<Vec2f>(j)[1] = current_frame_ip[src_i].y;

			src_i = matches[i].trainIdx;
			points3d.at<Vec3f>(j)[0] = prev_frame_wc[src_i].x;
			points3d.at<Vec3f>(j)[1] = prev_frame_wc[src_i].y;
			points3d.at<Vec3f>(j)[2] = prev_frame_wc[src_i].z;

			if (++j >= nr_matches_used)
				break;
		}

		Mat dist_coef(5, 1, CV_32F);
		Mat rotation_vector_calc(3, 1, CV_64F);
		Mat translation_vector_calc(3, 1, CV_64F);
		dist_coef = 0.0f;

		solvePnP(points3d, imagePoints, camera_matrix, dist_coef, rotation_vector_calc, translation_vector_calc);

		printf("Vstate:[%f, %f, %f]\n", translation_vector_calc.at<double>(1), -translation_vector_calc.at<double>(0), -translation_vector_calc.at<double>(2));

		//float alt_diff = (float) abs(state->at<float>(2) - (float)-translation_vector_calc.at<double>(2));

		//printf("alt diff: %f\n", alt_diff);

		/*
		state->at<float>(0) = (float) -translation_vector_calc.at<double>(1);
		state->at<float>(1) = (float) translation_vector_calc.at<double>(0);
		state->at<float>(2) = (float) -translation_vector_calc.at<double>(2);
		*/

		
		//dumpMatrix(translation_vector_calc);
		//printf("\n");
		//dumpMatrix(rotation_vector_calc);
		//printf("\n\n\n");

		//Sleep(20000);


		/*
		Mat measurement = (Mat_<float>(3,1) <<
			(float) translation_vector_calc.at<double>(1),
			(float) -translation_vector_calc.at<double>(0),
			(float) -translation_vector_calc.at<double>(2)
		);
		*/

		//KF->correct(measurement);
	}



	/* store current frame as previous frame */
	prev_frame_descriptors = descriptors;
	prev_frame_ip = current_frame_ip;
	imagepoints_to_world3d(current_frame_ip, prev_frame_wc);

	frame_counter++;

	//printf("please move drone NOW!\n");
	Sleep(500);
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

	//for( size_t i = 0; i < 1; i++ )
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

		//printf("IP (%f, %f) = (%f, %f, %d)\n", src[i].x, src[i].y, point3d.x, point3d.y, point3d.z);

		dst.push_back(point3d);
	}
}


void slam_module_frame::get_current_camera(Mat& pos, Mat& orientation)
{
	pos.at<float>(0) = 0.0f; //-state->at<float>(1); // x(cam) = y
	pos.at<float>(1) = 0.0f; //state->at<float>(0); // y(cam) = -x
	pos.at<float>(2) = state->at<float>(2); // z(cam) = z
	//printf("alt: %f\n", pos.at<float>(2));

	orientation.at<float>(0) = 0.0f; //state->at<float>(9);
	orientation.at<float>(1) = 0.0f; //state->at<float>(10);
	orientation.at<float>(2) = 0.0f; //state->at<float>(11); // camera orientation is downwards
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


int slam_module_frame::find_robust_matches(vector<Point2f>& p1, vector<Point2f>& p2, vector<DMatch>& matches, vector<char>& matchesMask)
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


	//vector<char> matchesMask(matches.size(), 0);
	Mat p1mt;
	perspectiveTransform(p1m, p1mt, homography);
	
	double maxInlierDist = 3;
	int nr_inliers = 0;
	for (size_t i1 = 0; i1 < nr_matches; i1++)
	{
		if (norm(p2m.at<Point2f>(i1) - p1mt.at<Point2f>(i1)) <= maxInlierDist)
		{
			matchesMask[i1] = 1;
			nr_inliers++;
		}
	}

	return nr_inliers;
}


void slam_module_frame::calculate_frame_mask(int width, int height)
{
	/*
	IplImage *mask_img = cvCreateImage(cvSize(width, height), 8, 1);
	Mat h_inverse = prev_frame_h.inv();
	CvMat invHomography = h_inverse;

	IplImage *obstacle_map_img = cvCreateImageHeader(cvSize(800, 800), 8, 1);
	obstacle_map_img->imageData = (char*)obstacle_map.data;

	cvWarpPerspective(obstacle_map_img, mask_img, &invHomography, CV_INTER_LINEAR);

	frame_mask = Mat(mask_img, true); // copy

	//imshow("Mask:", mask_img);
	cvWaitKey(4);
	*/
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



		/* determine distance threshold */
		/*
		vector<float> distances;
		float distance_threshold;
		size_t nr_matches = 10;

		for (size_t i = 0; i < matches.size(); i++)
			distances.push_back(matches[i].distance);

		sort(distances.begin(), distances.end(), greater<float>()); // desc

		distance_threshold = distances[ nr_matches-1 ];
		*/

		//printf("distance threshold: %f\n", distance_threshold);