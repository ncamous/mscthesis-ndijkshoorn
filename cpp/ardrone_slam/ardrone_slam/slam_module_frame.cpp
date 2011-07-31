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

	//fd = new SurfFeatureDetector(SLAM_SURF_HESSIANTHRESHOLD, 3, 4);
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
	/**/


	frame = NULL;
	prev_frame_descriptors = NULL;

	frame_counter = 0;


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
		//gray = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);
		gray = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 4);

		/* image corners */
		image_corners.push_back(Point2f(0.0f, 0.0f));
		image_corners.push_back(Point2f(0.0f, (float) (w - 1)));
		image_corners.push_back(Point2f((float) (h - 1), (float) (w - 1)));
		image_corners.push_back(Point2f((float) (h - 1), 0.0f));
	}

	frame->imageData = &f->data[4];




	// frames from the real ardrone are received in RGB order instead of BGR
	/*
	if (!f->usarsim)
		cvCvtColor( frame, frame, CV_RGB2BGR );
	*/


	// convert to gray
	//cvCvtColor(frame, gray, CV_BGR2GRAY);
	cvCvtColor(frame, gray, CV_BGR2BGRA);



	/* store current estimated position before computing: otherwise timestamp of frame and position do not match! */
	/*
	Mat obj_pos(3, 1, CV_64F);
	Mat obj_or(3, 1, CV_64F);
	get_objectpos(obj_pos, obj_or);
	*/


	/* find features */
	/*
	vector<cv::KeyPoint> keypoints;
	int features_found = find_features(gray, keypoints);

	current_frame_ip.clear();
	KeyPoint::convert(keypoints, current_frame_ip);
	*/

	/* calculate descriptors (on greyscale image) */
	/*
	Mat descriptors;
    de->compute(gray, keypoints, descriptors);
	*/


	/* match with previous frame */
	if (frame_counter > 0)
	{
		/*
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
		*/


		/* find robust matched descriptors (RANSAC) */
		/*
		vector<short> mask;
		int nr_inliers = find_robust_matches(current_frame_ip, prev_frame_ip, matches, mask, 10);

		if (nr_inliers < 4)
		{
			printf("Not enough inliers found (%i): dropping frame\n", nr_inliers);
			return;
		}
		*/


		/* retrieve camera motion from two frames */
		/*
		find_object_position(obj_pos, obj_or, matches, mask);
		*/

		/*
		int nr_inliers = find_object_position(obj_pos, obj_or, matches, mask);
		if (nr_inliers < 4)
		{
			printf("Not enough inliers found (%i): dropping frame\n", nr_inliers);
			return;
		}
		*/


		/*
		Mat new_pos(3, 1, CV_32F);
		Mat new_or(3, 1, CV_32F);
		object_to_worldpos(obj_pos, obj_or, new_pos, new_or);

		state->at<float>(0) = new_pos.at<float>(0);
		state->at<float>(1) = new_pos.at<float>(1);
		//state->at<float>(2) = new_pos.at<float>(2);

		//state->at<float>(9) = new_or.at<float>(0);
		//state->at<float>(10) = new_or.at<float>(1);
		state->at<float>(11) = new_or.at<float>(2);

		printf("Vstate:[%f, %f, %f]\n", state->at<float>(0), state->at<float>(1), state->at<float>(2));
		*/
	}



	/* store current frame as previous frame */
	/*
	prev_frame_descriptors = descriptors; // copy -> switch to pointer?
	prev_frame_ip = current_frame_ip; // copy -> switch to pointer?
	imagepoints_to_world3d(current_frame_ip, prev_frame_wc);
	*/

	/* add frame to canvas */
	Mat frameMat(gray);
	vector<Point3f> image_corners_wc;
	imagepoints_to_world3d(image_corners, image_corners_wc);
	controller->visual_map.update(frameMat, image_corners, image_corners_wc);
	/*
	for (size_t i = 0; i < image_corners_wc.size(); i++)
		printf("%f, %f, %f\n", image_corners_wc[i].x, image_corners_wc[i].y, image_corners_wc[i].z);
	printf("\n\n");
	*/

	frame_counter++;
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


void slam_module_frame::imagepoints_to_world3d(vector<Point2f>& src, vector<Point3f>& dst)
{
	Mat cam_pos(3, 1, CV_32F);
	Mat cam_or(3, 1, CV_32F);
	Mat cam_rot(3, 3, CV_32F);

	Mat rot(3, 3, CV_32F); // tmp

	get_localcam(cam_pos, cam_or);
	//if (cam_pos.at<float>(2) > 0.0f)
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
	pos.at<float>(0) = state->at<float>(0);
	pos.at<float>(1) = state->at<float>(1);
	pos.at<float>(2) = state->at<float>(2);

	or.at<float>(0) = state->at<float>(9);
	or.at<float>(1) = state->at<float>(10);
	or.at<float>(2) = state->at<float>(11);
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
	Mat rot(3, 3, CV_32F);
	cv::RotationMatrix3D(or, rot);
	rot = rot.t();

	pos.at<float>(0) *= -1.0f;
	pos.at<float>(1) *= -1.0f;
	pos = rot * pos;

	Rodrigues(rot, or); // rotation matrix to rotation vector
}


void slam_module_frame::localcam_to_object(Mat& pos, Mat& or)
{
	Mat rot(3, 3, CV_32F);
	cv::RotationMatrix3D(or, rot);
	rot = rot.t();

	pos = rot * pos;
	pos.at<float>(0) *= -1.0f;
	pos.at<float>(1) *= -1.0f;

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
	//or = rot * or;
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
	//or = rot * or;
}
/***/



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
	//if (SLAM_USE_OBSTACLE_MASK)
	//	calculate_frame_mask(img->width, img->height);

	// frame_mask is ignored when empty
	//if (SLAM_USE_OBSTACLE_MASK)
	//	fd->detect(img, v, frame_mask);
	//else

	//SurfFeatureDetector blup = new SurfFeatureDetector(SLAM_SURF_HESSIANTHRESHOLD, 3, 4);
	//fd->detect(img, v);

	SURF surf_extractor(SLAM_SURF_HESSIANTHRESHOLD);
	surf_extractor(img, Mat(), v);

	return v.size();
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