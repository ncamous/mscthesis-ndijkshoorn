#include "global.h"
#include "slam_visual_map.h"
#include "opencv_helpers.h"

using namespace std;
using namespace cv;


slam_visual_map::slam_visual_map(void):
	canvas(4096, 4096, CV_8UC4), // image, last channel not used
	undoTranslate(3, 3, CV_64F) // do not forget to set identity matrix
{
	canvas = Scalar(140, 140, 140, 0);
	map_updated = false;

	setIdentity(undoTranslate);

	memset(sync_roi, -1, 4 * sizeof(int));

	resolution = 0.2048f; // 1 cell (px) is 4.88 mm

	w = 2 * 2048;
	h = 2 * 2048;

	canvasSize.width = w;
	canvasSize.height = h;

	origin_x = 2048;
	origin_y = 2048;
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
		worldpos_to_cell(wc[i], dst[i]);
		update_roi(dst[i], frame_roi);
	}

	Mat T = getPerspectiveTransform (src, dst);

	int w = frame_roi[1] - frame_roi[0];
	int h = frame_roi[3] - frame_roi[2];
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


void slam_visual_map::worldpos_to_cell(Point3f& src, Point2f& dst)
{
	// convert local coordinates to world
	// round: add 0.5 and floor
	dst.x = floor(src.x * resolution + 0.5f) + origin_x;
	dst.y = floor(src.y * resolution + 0.5f) + origin_y;
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