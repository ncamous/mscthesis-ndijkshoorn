#include "global.h"
#include "slam.h"
#include "bot_ardrone.h"
#include "opencv_helpers.h"

#include <cv.hpp>

using namespace cv;


slam::slam(unsigned char mode, unsigned char bot_id):
	EKF(15, 15, 0)
{
	running = false;

	m_frame = NULL;
	m_sensor = NULL;
	m_ui = NULL;

	m_sensor_paused = false;

	this->_mode = mode;
	this->bot_id = bot_id;
}


slam::~slam()
{
}


void slam::run()
{
	running = true;

	init_ekf();


	/* modules */
	m_frame		= new slam_module_frame((slam*) this);
	m_sensor	= new slam_module_sensor((slam*) this);
	m_ui		= new slam_module_ui((slam*) this);


	/* events */
	event_sensor_resume = CreateEvent(NULL, false, false, (LPTSTR) "SLAM_SENSOR_RESUME");


	/* start threads */
	thread_process_frame	= CreateThread(NULL, 0, start_process_frame, (void*) this, 0, NULL);
	thread_process_sensor	= CreateThread(NULL, 0, start_process_sensor, (void*) this, 0, NULL);
	thread_ui				= CreateThread(NULL, 0, start_ui, (void*) this, 0, NULL);
}


void slam::init_ekf()
{
	// Initial state
	//randn(EKF.statePost, Scalar::all(0), Scalar::all(0.001));
	//EKF.statePost.at<float>(0) = -5000.0f;


	// F vector
	setIdentity(EKF.transitionMatrix); // completed (T added) when measurement received and T is known


	EKF.processNoiseCov = 0.0f;
	float PNC[15] = {
		2.0f, 2.0f, 2.0f,		// p: mm
		2.0f, 2.0f, 2.0f,		// v: mm/s
		0.2f, 0.2f, 0.2f,		// a: mm/s2
		0.002f, 0.002f, 0.002f,	// or: rad
		0.001f, 0.001f, 0.001f	// sonar: raw, vel, accel
	};
	MatSetDiag(EKF.processNoiseCov, PNC);


	float ECP[15] = {
		30.0f, 30.0f, 30.0f,
		20.0f, 20.0f, 20.0f,
		0.0f, 0.0f, 0.0f,
		0.01f, 0.01f, 0.01f,
		0.01f, 0.01f, 0.01f,
	};
	MatSetDiag(EKF.errorCovPost, ECP);


	setIdentity(EKF.measurementMatrix);
}


// calculated Jocobian matrix
void slam::update_transition_matrix(float difftime)
{
	for (int i = 0; i < 3; i++)
	{
		// position (p)
		EKF.transitionMatrix.at<float>(i, 3+i) = difftime;
		EKF.transitionMatrix.at<float>(i, 6+i) = 0.5f * difftime*difftime;
		// velocity (v)
		EKF.transitionMatrix.at<float>(3+i, 6+i) = difftime;

#ifndef BOT_ARDRONE_USE_ONBOARD_OR
		// attitude (q)
		EKF.transitionMatrix.at<float>(9+i, 12+i) = difftime;
#endif
	}
}


bool slam::mode(unsigned char mode)
{
	return ((this->_mode & mode) != 0x00);
}


void slam::on(unsigned char mode)
{
	this->_mode |= mode;

	printf("# SLAM ON: ");

	switch (mode)
	{
	case SLAM_MODE_VISUALMOTION:
		printf("VISUALMOTION\n"); break;

	case SLAM_MODE_ACCEL:
		printf("ACCEL\n"); break;

	case SLAM_MODE_VEL:
		printf("VEL\n"); break;

	case SLAM_MODE_VISUALLOC:
		printf("VISUALLOC\n"); break;

	case SLAM_MODE_MAP:
		printf("MAP\n"); break;
	}
}


void slam::off(unsigned char mode)
{
	this->_mode &= ~mode;

	printf("# SLAM OFF: ");

	switch (mode)
	{
	case SLAM_MODE_VISUALMOTION:
		printf("VISUALMOTION\n"); break;

	case SLAM_MODE_ACCEL:
		printf("ACCEL\n"); break;

	case SLAM_MODE_VEL:
		printf("VEL\n"); break;

	case SLAM_MODE_VISUALLOC:
		printf("VISUALLOC\n"); break;

	case SLAM_MODE_MAP:
		printf("MAP\n"); break;
	}
}


float* slam::get_state()
{
	return (float*) EKF.statePost.data;
}


void slam::add_input_frame(bot_ardrone_frame *f)
{
	if (!running)
		run();

	// frame module not enable: drop frame
	if (m_frame == NULL)
		delete f;

	// drop frame is queue enabled but not empty
	if (SLAM_USE_QUEUE && queue_frame.empty())
	{
		queue_frame.lock();
		queue_frame.push(f);
		queue_frame.release();
	}
	else if(!SLAM_USE_QUEUE)
		m_frame->process(f);
	else
		delete f;
}


void slam::add_input_sensor(bot_ardrone_measurement *m)
{
	if (!running)
		run();

	if (SLAM_USE_QUEUE)
	{
		queue_sensor.lock();
		queue_sensor.push(m);
		queue_sensor.release();
	}
	else if(!SLAM_USE_QUEUE)
		m_sensor->process(m);
}


void slam::get_world_position(float *pos)
{
	memcpy_s(pos, 12, EKF.statePost.data, 12);
	memcpy_s(pos + 3, 12, EKF.statePost.data + 36, 12);
}


void slam::sensor_pause(double time)
{
	if (m_sensor_paused)
		return;

	m_sensor_paused_time	= time;
	m_sensor_paused			= true;

	ResetEvent(event_sensor_resume);
}


void slam::sensor_resume()
{
	m_sensor_paused			= false;

	SetEvent(event_sensor_resume);
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

		q->lock();
		item = q->front();
		q->release();

		processor->process(item);

		q->pop();

		/* free memory */
		delete item;
	}

	return 1;
}


static DWORD WINAPI start_process_sensor(void* Param)
{
	slam* This = (slam*) Param; 
	slam_queue<bot_ardrone_measurement*> *q = &This->queue_sensor;
	bot_ardrone_measurement *item;
	slam_module_sensor *processor = This->m_sensor;

	bool *paused			= &This->m_sensor_paused;
	double *paused_time		= &This->m_sensor_paused_time;

	while (!exit_application)
	{
		if (q->empty())
			q->wait_until_filled(2000);

		// just in case
		if (q->empty())
			continue;

		q->lock();
		item = q->front();
		q->release();

		// paused
		if (*paused && item->time >= *paused_time)
			WaitForSingleObject(This->event_sensor_resume, 5000);

		processor->process(item);

		q->pop();

		/* free memory */
		delete item;
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
		Sleep(35);
	}

	return 1;
}