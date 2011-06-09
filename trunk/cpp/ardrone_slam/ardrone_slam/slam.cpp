#include "global.h"
#include "slam.h"
#include "bot_ardrone.h"

#include <cv.hpp>



slam::slam()
{
	running = false;

	/*
	if (!running)
		run();

    IplImage* img1 = cvLoadImage("puzzel1.jpg");
    IplImage* img2 = cvLoadImage("puzzel2.jpg");
	IplImage* img3 = cvLoadImage("puzzel5.jpg");

	process_frame(img1);
	cvReleaseImage(&img1);
	process_frame(img2);
	cvReleaseImage(&img2);
	process_frame(img3);
	*/
}


slam::~slam()
{
}


void slam::run()
{
	running = true;

	canvas = cvCreateImage(cvSize(800,800), 8, 3);

	/* modules */
	m_frame = new slam_module_frame((slam*) this);
	m_sensor = new slam_module_sensor((slam*) this);
	m_ui = new slam_module_ui((slam*) this);


	initial_height = -1;

	/* start threads */
	thread_process_frame = CreateThread(NULL, 0, start_process_frame, (void*) this, 0, NULL);
	thread_process_sensor = CreateThread(NULL, 0, start_process_sensor, (void*) this, 0, NULL);
	thread_ui = CreateThread(NULL, 0, start_ui, (void*) this, 0, NULL);
}


void slam::add_input_frame(bot_ardrone_frame *f)
{
	if (!running)
		run();

	// drop frame is queue enabled but not empty
	if (SLAM_USE_QUEUE && queue_frame.empty())
		queue_frame.push(f);
	else if(!SLAM_USE_QUEUE)
		m_frame->process(f);
}


void slam::add_input_sensor(bot_ardrone_measurement *m)
{
	if (!running)
		run();

	if (SLAM_USE_QUEUE)
		queue_sensor.push(m);
	else if(!SLAM_USE_QUEUE)
		m_sensor->process(m);
}


/* threads */
static DWORD WINAPI start_process_frame(void* Param)
{
	slam* This = (slam*) Param; 
	slam_queue<bot_ardrone_frame*> *q = &This->queue_frame;
	bot_ardrone_frame *item;
	slam_module_frame *processor = This->m_frame;

	while (!exit_application)
	{
		if (q->empty())
			q->wait_until_filled(2000);

		// just in case
		if (q->empty())
			continue;

		item = q->front();
		q->pop();

		processor->process(item);
	}

	return 1;
}


static DWORD WINAPI start_process_sensor(void* Param)
{
	slam* This = (slam*) Param; 
	slam_queue<bot_ardrone_measurement*> *q = &This->queue_sensor;
	bot_ardrone_measurement *item;
	slam_module_sensor *processor = This->m_sensor;

	while (!exit_application)
	{
		if (q->empty())
			q->wait_until_filled(2000);

		// just in case
		if (q->empty())
			continue;

		item = q->front();
		q->pop();

		processor->process(item);
	}

	return 1;
}


static DWORD WINAPI start_ui(void* Param)
{
	slam* This = (slam*) Param; 
	slam_module_ui *processor = This->m_ui;

	while (!exit_application)
	{
		processor->update();
		Sleep(2000); // update every 2 seconds
	}

	return 1;
}