#include "global.h"
#include "slam_visual_map.h"
#include "opencv_helpers.h"
#include <map>

// TMP
#include <cv.hpp>
#include <cxcore.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace std;
using namespace cv;


slam_visual_map::slam_visual_map(void):
	canvas(4096, 4096, CV_8UC4), // image, last channel not used
	undoTranslate(3, 3, CV_64F), // do not forget to set identity matrix
	descriptors(1000, 64, CV_32F),
	keypoints_wc(1000, 1, CV_32FC2),
	descriptors_grid(200, 200, CV_16U),

	termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03)
{
	canvas				= Scalar(140, 140, 140, 0);
	descriptors_grid	= Scalar(0);
	map_updated			= false;

	setIdentity(undoTranslate);

	memset(sync_roi, -1, 4 * sizeof(int));

	resolution		= 0.2048f; // 1 cell (px) is 4.88 mm
	resolution_inv	= 4.8828125f;

	w = 2 * 2048;
	h = 2 * 2048;

	canvasSize.width	= w;
	canvasSize.height	= h;

	origin_x = 2048;
	origin_y = 2048;



	/* features */
	descriptors_count = 1;
}


slam_visual_map::~slam_visual_map(void)
{

}


void slam_visual_map::update(Mat& frame, vector<Point2f>& lc, vector<Point3f>& wc)
{
	Point2f src[4];
	Point2f dst[4];
	memset(frame_roi, -1, 4 * sizeof(int)); // reset frame roi

	for (int i = 0; i < 4; i++)
	{
		src[i] = Point2f(lc[i].x, lc[i].y);
		worldpos_to_canvaspos(wc[i], dst[i]);
		update_roi(dst[i], frame_roi);
	}

	Mat T = getPerspectiveTransform (src, dst);

	int w = frame_roi[1] - frame_roi[0];
	int h = frame_roi[3] - frame_roi[2];

	// DIRTY...
	if (frame_roi[0] < 0 || frame_roi[2] < 0 
		||
		frame_roi[0] + w >= 4096 || frame_roi[2] + h >= 4096)
		return;

	Mat subCanvas(canvas, Rect(frame_roi[0], frame_roi[2], w, h));

	undoTranslate.at<double>(0, 2) = (double) -frame_roi[0];
	undoTranslate.at<double>(1, 2) = (double) -frame_roi[2];

	T = undoTranslate * T;

	canvasSize.width = w;
	canvasSize.height = h;

	warpPerspective(frame, subCanvas, T, canvasSize, INTER_LINEAR, BORDER_TRANSPARENT);
	//warpPerspective(frame, canvas, T, canvasSize, INTER_LINEAR, BORDER_TRANSPARENT);

	// set ROI updated
	update_roi(frame_roi, sync_roi);
	map_updated = true;
}


void slam_visual_map::update(vector<KeyPoint>& keypoints, Mat& descriptors, vector<Point3f>& wc)
{
	int key;
	unsigned short *key_p;
	unsigned short i, key_x, key_y;

	key_p = (unsigned short*) &key;

	int size = keypoints.size();
	unsigned short *x = NULL;
	unsigned short *y = NULL;

	// double the size of the matrix
	if (descriptors_count + size > this->descriptors.rows)
	{
		this->descriptors.resize(this->descriptors.rows * 2);
		this->keypoints_wc.resize(this->keypoints_wc.rows * 2);
	}

	//printf("%i\n", descriptors_count, keypoints.size());


	x = new unsigned short[size]; // faster than vector
	y = new unsigned short[size]; // faster than vector

	// convert all at once
	worldpos_to_dgridpos(wc, x, y);

	cell_best_keypoints.clear();

	for (i = 0; i < descriptors.rows; i++)
	{
		if (!cell_inside_descriptors_grid(x[i], y[i]) || descriptors_grid.at<unsigned short>(y[i], x[i]) > 0)
			continue;

		memcpy_s(key_p, 2, &x[i], 2);
		memcpy_s(key_p + 1, 2, &y[i], 2);

		it = cell_best_keypoints.find(key);

		if (it == cell_best_keypoints.end() || keypoints[i].response > keypoints[it->second].response)
		{
			cell_best_keypoints[key] = i;
			//printf("%u,%u -> %i\n", x[i], y[i], key);
		}
	}



	// get all cells from map
	for(it = cell_best_keypoints.begin(); it != cell_best_keypoints.end(); it++)
	{
		key = it->first;
		i = it->second;

		memcpy_s(&key_x, 2, key_p, 2);
		memcpy_s(&key_y, 2, key_p + 1, 2);

		// descriptor
		memcpy_s(
			this->descriptors.data + (descriptors_count * SLAM_DESCRIPTOR_SIZE),
			this->descriptors.rows * SLAM_DESCRIPTOR_SIZE,
			descriptors.data + (i * SLAM_DESCRIPTOR_SIZE),
			SLAM_DESCRIPTOR_SIZE
		);

		// keypoint
		keypoints_wc.at<Vec2f>(descriptors_count)[0] = wc[i].x;
		keypoints_wc.at<Vec2f>(descriptors_count)[1] = wc[i].y;

		this->keypoints.push_back(keypoints[i]);

		descriptors_grid.at<unsigned short>(key_y, key_x) = descriptors_count++;
	}

	delete [] x;
	delete [] y;

	//imshow("DescriptorGrid", descriptors_grid);
	//cvWaitKey(4);
}


void slam_visual_map::get_local_descriptors(Mat& map_descriptors, Mat& map_keypoints, Point3f& wc, float radius)
{
	// return whole map
	if (radius <= 0.0f)
	{
		// return all keypoints + descriptors
		Range rows = Range(1, descriptors_count);

		map_descriptors = Mat(descriptors, rows);
		map_keypoints = Mat(keypoints_wc, rows);

		return;
	}

	int i = 0;
	unsigned short index;
	unsigned short *indices;
	unsigned short x, y;

	worldpos_to_dgridpos(wc, &x, &y);

	int r = (int) floor(radius * 0.01f);

	/*
	int r2 = r * r;

	indices = new unsigned short[r2*r2]; // can do better

	for (int y2 = -r; y2 <= r; y2++)
	{
		for (int x2 = -r; x2 <= r; x2++)
		{
			if (x2*x2*y2*y2 <= r2)
			{	
				index = descriptors_grid.at<unsigned short>(y + y2, x + x2);

				if (cell_inside_descriptors_grid(x + x2, y + y2) && index > 0)
					indices[i++] = index;
			}
		}
	}
	*/

	int x2, y2, w, h;
	x2 = max(0, x - r);
	y2 = max(0, y - r);
	w = min(descriptors_grid.cols - x2, r * 2);
	h = min(descriptors_grid.rows - y2, r * 2);

	indices = new unsigned short[w * h];

	Mat grid(descriptors_grid, Rect(x2, y2, w, h));

	for (x2 = 0; x2 < grid.cols; x2++)
	{
		for (y2 = 0; y2 < grid.rows; y2++)
		{
			index = grid.at<unsigned short>(y2, x2);
			if (index > 0)
				indices[i++] = index;
		}
	}

	map_descriptors	= Mat(i, descriptors.cols, descriptors.type());
	map_keypoints	= Mat(i, keypoints_wc.cols, keypoints_wc.type());

	int descriptors_rowsize	= SLAM_DESCRIPTOR_SIZE;
	int keypoint_rowsize	= 2 * sizeof(float);

	for (int j = 0; j < i; j++)
	{
		memcpy_s(map_descriptors.data + j * descriptors_rowsize, descriptors_rowsize,
			descriptors.data + indices[j] * descriptors_rowsize, descriptors_rowsize);

		memcpy_s(map_keypoints.data + j * keypoint_rowsize, keypoint_rowsize,
			keypoints_wc.data + indices[j] * keypoint_rowsize, keypoint_rowsize);
	}

	delete [] indices;
}


void slam_visual_map::frame_to_canvas(Mat& frame, Mat& frameT, vector<Point2f>& lc, vector<Point3f>& wc)
{
	Point2f src[4];
	Point2f dst[4];
	memset(frame_roi, -1, 4 * sizeof(int)); // reset frame roi

	for (int i = 0; i < 4; i++)
	{
		src[i] = Point2f(lc[i].x, lc[i].y);
		worldpos_to_canvaspos(wc[i], dst[i]);
		update_roi(dst[i], frame_roi);
	}

	Mat T = getPerspectiveTransform (src, dst);

	int w = frame_roi[1] - frame_roi[0];
	int h = frame_roi[3] - frame_roi[2];

	undoTranslate.at<double>(0, 2) = (double) -frame_roi[0];
	undoTranslate.at<double>(1, 2) = (double) -frame_roi[2];

	T = undoTranslate * T;

	canvasSize.width = w;
	canvasSize.height = h;

	frameT.create(w, h, frame.type());

	warpPerspective(frame, frameT, T, canvasSize, INTER_LINEAR, BORDER_TRANSPARENT);
}


bool slam_visual_map::is_updated(int* roi, bool reset_roi)
{
	if (!map_updated)
		return false;

	memcpy_s(roi, 4 * sizeof(int), sync_roi, 4 * sizeof(int));

	if (reset_roi)
	{
		memset(sync_roi, -1, 4 * sizeof(int));
		map_updated = false;
	}

	return true;
}


byte* slam_visual_map::get_array()
{
	return (byte*) canvas.data;
}


void slam_visual_map::worldpos_to_canvaspos(Point3f& src, Point2f& dst)
{
	// convert local coordinates to world
	// round: add 0.5 and floor
	dst.x = floor(src.x * resolution + 0.5f) + origin_x;
	dst.y = floor(src.y * resolution + 0.5f) + origin_y;
}


void slam_visual_map::canvaspos_to_worldpos(Point2f& src, Point2f& dst)
{
	dst.x = (src.x + frame_roi[0] - (float) origin_x) * resolution_inv;
	dst.y = (src.y + frame_roi[2] - (float) origin_y) * resolution_inv;
}


void slam_visual_map::worldpos_to_dgridpos(std::vector<cv::Point3f>& src, unsigned short *x, unsigned short *y)
{
	for (size_t i = 0; i < src.size(); i++)
	{
		x[i] = (unsigned short) floor(src[i].x * 0.01f + 0.5f) + 100;
		y[i] = (unsigned short) floor(src[i].y * 0.01f + 0.5f) + 100;
	}
}


void slam_visual_map::worldpos_to_dgridpos(cv::Point3f& src, unsigned short *x, unsigned short *y)
{
	*x = (unsigned short) floor(src.x * 0.01f + 0.5f) + 100;
	*y = (unsigned short) floor(src.y * 0.01f + 0.5f) + 100;
}


void slam_visual_map::update_roi(Point2f& p, int *roi)
{
	if (roi[0] == -1 || p.x < roi[0])
		roi[0] = (int) p.x;
	
	if (roi[1] == -1 || p.x > roi[1])
		roi[1] = (int) p.x;

	if (roi[2] == -1 || p.y < roi[2])
		roi[2] = (int) p.y;

	if (roi[3] == -1 || p.y > roi[3])
		roi[3] = (int) p.y;
}


void slam_visual_map::update_roi(int *src, int *dst)
{
	if (dst[0] == -1 || src[0] < dst[0])
		dst[0] = src[0];
	
	if (dst[1] == -1 || src[1] > dst[1])
		dst[1] = src[1];

	if (dst[2] == -1 || src[2] < dst[2])
		dst[2] = src[2];

	if (dst[3] == -1 || src[3] > dst[3])
		dst[3] = src[3];
}


bool slam_visual_map::inside(Mat& m, Rect& r)
{
	Size size = m.size();

	if (r.x + r.width >= size.width || r.y + r.height >= size.height || r.x - r.width < 0 || r.y - r.height < 0)
		return false;

	return true;
}


inline bool slam_visual_map::cell_inside_descriptors_grid(unsigned short x, unsigned short y)
{
	return (x >= 0 && y >= 0 && x < 200 && y < 200);
}


void slam_visual_map::save_canvas()
{
	imwrite("dataset/visual_map_canvas.png", canvas);
	printf("Saved visual map canvas\n");
}