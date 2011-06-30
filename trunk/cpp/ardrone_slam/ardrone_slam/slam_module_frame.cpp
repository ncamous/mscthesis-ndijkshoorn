#include "global.h"
#include "slam_module_frame.h"
#include "bot_ardrone.h"

#include "opencv_helpers.h"
#include <cv.hpp>
#include <cxcore.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;


slam_module_frame::slam_module_frame(slam *controller)
{
	this->controller = controller;

	fd = new SurfFeatureDetector(SLAM_SURF_HESSIANTHRESHOLD, 3, 4);
	de = new SurfDescriptorExtractor();

	frame = NULL;
	prev_frame_descriptors = NULL;

	last_loc[0] = 300;
	last_loc[1] = 300;

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
	estimated_pos[0] = 0.0f;
	estimated_pos[1] = 0.0f;
	estimated_pos[2] = 0.0f;
}


slam_module_frame::~slam_module_frame(void)
{
}

void slam_module_frame::process(bot_ardrone_frame *f)
{
	if (frame == NULL)
	{
		unsigned short w, h;

		memcpy_s(&w, 2, &f->data[0], 2);
		memcpy_s(&h, 2, &f->data[2], 2);

		w = htons(w);
		h = htons(h);

		frame = cvCreateImageHeader(cvSize(w, h), IPL_DEPTH_8U, 3);
		gray = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);
	}

	frame->imageData = &f->data[4];


	/* display image */
	//add_noise(frame);
	/*
	add_noise(frame);
	imshow("Image:", frame);
	cvWaitKey(4);
	return;
	*/
	/**/


	// frames from the real ardrone are received in RGB order instead of BGR
	if (!f->usarsim)
		cvCvtColor( frame, frame, CV_RGB2BGR );


	/* find features */
	vector<cv::KeyPoint> keypoints;
	int features_found = find_features(frame, keypoints);



	/* calculate descriptors (on greyscale image) */
	Mat descriptors;
	cvCvtColor(frame, gray, CV_RGB2GRAY);
    de->compute(gray, keypoints, descriptors);




	/* match with previous frame */
	if (frame_counter > 0)
	{
		double ransacReprojThreshold = 3.0;

		if (keypoints.size() < 20)
		{
			printf("Not enough features found: dropping frame\n");
			return;
		}

		vector<DMatch> matches;
		dm.match(descriptors, prev_frame_descriptors, matches);

		if (matches.size() < 20)
			return;



		/* calculate transformation (RANSAC) */
		vector<int> queryIdxs( matches.size() ), trainIdxs( matches.size() );
		for( size_t i = 0; i < matches.size(); i++ )
		{
			queryIdxs[i] = matches[i].queryIdx;
			trainIdxs[i] = matches[i].trainIdx;
		}
	

		vector<Point2f> points1;
		KeyPoint::convert(keypoints, points1, queryIdxs);

		vector<Point2f> points2;
		KeyPoint::convert(prev_frame_keypoints, points2, trainIdxs);




		/* retrieve camera motion from two frames */
		Mat points3d(9, 1, CV_32FC3);
		Mat imagePoints(9, 1, CV_32FC2);
		vector <Point2f> imagePoints2;

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{

				points3d.at<Vec3f>(i*3 + j, 0)[0] = 40.0f * i;
				points3d.at<Vec3f>(i*3 + j, 0)[1] = 40.0f * j;
				points3d.at<Vec3f>(i*3 + j, 0)[2] = (40.0f * i) - (40.0f * i);

			}
		}

		/*
		for( size_t i = 0; i < matches.size(); i++ )
		{
			points3d.at<Vec3f>(0, i)[0] = points2[i].x * 50.0f;
			points3d.at<Vec3f>(0, i)[1] = points2[i].y * 50.0f;
			points3d.at<Vec3f>(0, i)[2] = 0.0f;
			//points3d.at<Vec3f>(0, i)[2] = 0.0 + rand() * (10.0 - 0.0) / RAND_MAX;

			//printf("%f, %f, %f\n", points3d.at<Vec3f>(0, i)[0], points3d.at<Vec3f>(0, i)[1], points3d.at<Vec3f>(0, i)[2]);

			//elem2f[0] = points1[i].x;
			//elem2f[1] = points1[i].y;
		}
		*/


		//elem3f = points3d.at<Vec3f>(0, 0);

		//printf("%f, %f, %f\n", points3d.at<Vec3f>(0, 0)[0], points3d.at<Vec3f>(0, 0)[1], points3d.at<Vec3f>(0, 0)[2]);

		Mat camera_matrix(3, 3, CV_32F);
		Mat dist_coef(5, 1, CV_32F);
		Mat rotation_vector(3, 1, CV_32F);
		Mat translation_vector(3, 1, CV_32F);

		Mat rotation_vector_calc(3, 1, CV_64F);
		Mat translation_vector_calc(3, 1, CV_64F);

		camera_matrix = 0.0f;
		camera_matrix.at<float>(0, 0) = 31.9850946f; //1.60035f;
		camera_matrix.at<float>(1, 1) = 32.5596395f; //1.60035f;
		camera_matrix.at<float>(2, 2) = 1.f;
		camera_matrix.at<float>(0, 2) = 223.61462402f;
		camera_matrix.at<float>(1, 2) = -39.23768616f;

		dist_coef = 0.0f;
		translation_vector = 0.0f;
		rotation_vector = 0.0f;

		translation_vector.at<float>(2, 0) = -20.0f;
		translation_vector.at<float>(0, 0) = 30.0f;
		rotation_vector.at<float>(2, 0) = 1.57079633f;

		projectPoints( points3d, rotation_vector, translation_vector, camera_matrix, dist_coef, imagePoints2 );

		for( size_t i = 0; i < imagePoints2.size(); i++ )
		{
			imagePoints.at<Vec2f>(i, 0)[0] = imagePoints2[i].x;
			imagePoints.at<Vec2f>(i, 0)[1] = imagePoints2[i].y;

			//printf("%f, %f\n", imagePoints2[i].x, imagePoints2[i].y);
		}

		//printf("\n\n\n");

		solvePnP( points3d, imagePoints, camera_matrix, dist_coef, rotation_vector_calc, translation_vector_calc );

		dumpMatrix(translation_vector_calc);
		printf("\n");
		dumpMatrix(rotation_vector_calc);
		printf("\n\n\n");

		/*
		double world_translation[2];

		controller->get_world_position((double*) translation_vector_calc.data, world_translation);

		estimated_pos[0] += (float) world_translation[1];
		estimated_pos[1] -= (float) world_translation[0];

		printf("%f, %f\n", estimated_pos[0], estimated_pos[1]);
		*/

		//Mat homography = findHomography( Mat(points1), Mat(points2), CV_RANSAC, ransacReprojThreshold );

		//cvCalcImageHomography(


		//dumpMatrix(homography);
		//printf("\n\n");

		//double canvas_pos[3];
		//canvas_pos[0] = homography.at<double>(0,2);
		//canvas_pos[1] = homography.at<double>(1,2);
		//canvas_pos[2] = 0.0;

	}


	/* get center position */
	/*
	double center[] = { (double)frame->width * 0.5, (double)frame->height * 0.5, 0.0 };
	Mat point_center = Mat(1, 3, CV_64F, center);

	Mat tc = point_center * prev_frame_h;

	double *tc_data = (double*) tc.data;
	double *h_data = (double*) prev_frame_h.data;

	last_loc[0] = int(tc_data[0] + h_data[2]);
	last_loc[1] = int(tc_data[1] + h_data[5]);
	*/

	/* get position from KF */
	//double pos[2];
	//if (!controller->get_canvas_position(pos))
	//	return; // OK?

	/*
	double* h_data = (double*) prev_frame_h.data;
	h_data[2] = 300.0 + pos[1];
	h_data[5] = 300.0 - pos[0];

	*/
	//printf("%f, %f\n", h_data[2], h_data[5]);


	/* draw image */
	//CvMat CvHomography = prev_frame_h;
	//cvWarpPerspective(frame, controller->canvas, &CvHomography, CV_INTER_LINEAR);


	/* store current frame as previous frame */
	prev_frame_keypoints = keypoints;
	prev_frame_descriptors = descriptors;

	//imshow("Image:", controller->canvas);
	//cvWaitKey(4);
	/*
	if (frame_counter > 0 && frame_counter % 4 == 0)
	{
	IplImage *canvas_resized = cvCreateImage( cvSize(800,800), 8, 3 );
	cvResize(canvas, canvas_resized);
	imshow("Image:", canvas_resized);
	cvWaitKey(4);
	}
	*/

	frame_counter++;
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

	imshow("Mask:", mask_img);
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