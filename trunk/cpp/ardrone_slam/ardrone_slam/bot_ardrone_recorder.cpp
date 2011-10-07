#include "global.h"
#include "bot_ardrone_recorder.h"
#include "bot_ardrone.h"
#include "yaml.h"
#include <io.h>

#include <cv.hpp>
#include <cxcore.hpp>
#include <opencv2/highgui/highgui.hpp>


HANDLE bot_ardrone_recorder::ghSemaphore;


void operator >> (const YAML::Node& node, float *f)
{
	int i;
	int len = node.size();

	for(i = 0; i < len; i++)
		node[i] >> f[i];
}

bot_ardrone_recorder::bot_ardrone_recorder(bot_ardrone *bot)
{
	this->bot = bot;

	frame_counter = 1;
}


bot_ardrone_recorder::~bot_ardrone_recorder(void)
{
	// todo
}


void bot_ardrone_recorder::record_measurement(bot_ardrone_measurement *m)
{
	WaitForSingleObject(ghSemaphore, INFINITE);

	fprintf (file_out, "---\n");
	fprintf (file_out, "e: %i\n", BOT_EVENT_MEASUREMENT);
	//fprintf (file_out, "s: %i\n", m->sensor); // USARSim now sends all sensors in a single message
	fprintf (file_out, "t: %f\n", m->time);
	fprintf (file_out, "alt: %i\n", m->altitude);

	fprintf (file_out, "or: [%f, %f, %f]\n", m->or[0], m->or[1], m->or[2]);
	fprintf (file_out, "accel: [%f, %f, %f]\n", m->accel[0], m->accel[1], m->accel[2]);
	fprintf (file_out, "vel: [%f, %f, %f]\n", m->vel[0], m->vel[1], m->vel[2]);

	//fprintf (file_out, "%f,%f,%f,%f,%f,%f\n", m->or[0], m->or[1], m->or[2], m->accel[0], m->accel[1], m->accel[2]);
	//fprintf (file_out, "%f,%f,%f,%f,%f,%f\n", m->or[0], m->or[1], m->or[2], m->vel[0], m->vel[1], m->vel[2]);

	/* USARSim only */
	/*
	if (m->usarsim)
	{
		fprintf (file_out, "gt_loc: [%f, %f, %f]\n", m->gt_loc[0], m->gt_loc[1], m->gt_loc[2]);
		//fprintf (file_out, "gt_or:\n  - %f\n  - %f\n  - %f\n", m->gt_or[0], m->gt_or[1], m->gt_or[2]);
	}
	*/

	ReleaseSemaphore(ghSemaphore, 1, NULL);
}


void bot_ardrone_recorder::record_control(bot_ardrone_control *c)
{
	WaitForSingleObject(ghSemaphore, INFINITE);

	fprintf (file_out, "---\n");
	fprintf (file_out, "e: %i\n", BOT_EVENT_CONTROL);
	fprintf (file_out, "t: %f\n", c->time);
	fprintf (file_out, "vel: [%f, %f, %f, %f]\n", c->velocity[0], c->velocity[1], c->velocity[2], c->velocity[3]);

	ReleaseSemaphore(ghSemaphore, 1, NULL);
}


void bot_ardrone_recorder::record_frame(bot_ardrone_frame *f)
{
	WaitForSingleObject(ghSemaphore, INFINITE);

	char filename[25];
	//printf("recorded frame %i\n", frame_counter);

	sprintf_s(f->filename, 20, "%06d.%s", frame_counter++, BOT_ARDRONE_RECORD_EXT);
	sprintf_s(filename, 25, "%s/%s", dataset_dir, f->filename);

	fprintf (file_out, "---\n");
	fprintf (file_out, "e: %i\n", BOT_EVENT_FRAME);
	fprintf (file_out, "t: %f\n", f->time);
	fprintf (file_out, "s: %i\n", f->data_size);
	fprintf (file_out, "f: %s\n", f->filename);

	// PNG
	if (BOT_ARDRONE_RECORD_EXT == "png")
	{
		Mat frame = Mat(BOT_ARDRONE_FRAME_H, BOT_ARDRONE_FRAME_W, CV_8UC3, NULL, 0);
		frame.data = (uchar*) &f->data[4];
		imwrite(filename, frame);
	}
	// RAW
	else
	{
		ofstream frame_out(filename, ios::out | ios::binary);
		frame_out.write(f->data, f->data_size);
		frame_out.close();
	}

	ReleaseSemaphore(ghSemaphore, 1, NULL);
}


void bot_ardrone_recorder::playback(char *dataset)
{
	char filename[25];
	int wait;
	double last_event_time = DBL_MAX; // play first event without delay
	double event_time;

	sprintf_s(dataset_dir, 25, "dataset/%s", dataset);
	sprintf_s(filename, 25, "%s/output.yaml", dataset_dir);

	// check
	if (fin.is_open())
		printf("ERROR: DATASET PLAYBACK FILE ALREADY OPEN!\n");

	int event_type;
	YAML::Node doc;

	fin.open(filename, ios::in);
	YAML::Parser parser(fin);

    while(parser.GetNextDocument(doc))
	{
		// timer
		event_time = doc["t"];
		wait = int((event_time - last_event_time) * 1000.0);
		last_event_time = event_time;
		if (wait > 0)
			Sleep(wait);
		//else
		//	Sleep(20);

		printf("%f\n", event_time);


		doc["e"] >> event_type;

		switch (event_type)
		{
			case BOT_EVENT_MEASUREMENT:
			{
				bot_ardrone_measurement *m = new bot_ardrone_measurement;
				//bot_ardrone_measurement m;
				doc["t"] >> m->time;
				doc["alt"] >> m->altitude;
				doc["or"] >> m->or;
				doc["accel"] >> m->accel;
				doc["vel"] >> m->vel;

				// usarsim
				//if (doc.FindValue("gt_loc"))
				//	doc["gt_loc"] >> m->gt_loc;

				bot->measurement_received(m);
				break;
			}
		
			case BOT_EVENT_CONTROL:
			{
				//bot_ardrone_control c;
				bot_ardrone_control *c = &bot->control;
				doc["t"] >> c->time;
				doc["vel"] >> c->velocity;

				bot->control_update(c);
				break;
			}

			case BOT_EVENT_FRAME:
			{
				break;
				char filename[30];
				string tmpstring;
				//bot_ardrone_frame f;
				bot_ardrone_frame *f = new bot_ardrone_frame;

				doc["t"] >> f->time;
				doc["s"] >> f->data_size;
				doc["f"] >> tmpstring;
				strcpy_s(f->filename, 25, tmpstring.c_str());

				sprintf_s(filename, 25, "%s/%s", dataset_dir, f->filename);
				ifstream frame_in(filename, ios::in | ios::binary);

				// data buffer
				frame_in.read(f->data + 4, f->data_size - 4);
				frame_in.close();

				Mat img_bgr(BOT_ARDRONE_FRAME_H, BOT_ARDRONE_FRAME_W, CV_8UC3, NULL, 0);
				Mat img_bgra(BOT_ARDRONE_FRAME_H, BOT_ARDRONE_FRAME_W, CV_8UC4);

				img_bgr.data		= (unsigned char*) f->data;

				cvtColor(img_bgr, img_bgra, CV_BGR2BGRA, 4);

				memcpy_s(f->data, BOT_ARDRONBOT_EVENT_FRAME_BUFSIZE, img_bgra.data, img_bgra.rows * img_bgra.cols * 4);

				bot->frame_received(f);

				break;
			}
		}
	}
}


void bot_ardrone_recorder::prepare_dataset()
{
	// file already open check?

	int i = 1;
	char filename[25];

	sprintf_s(filename, 25, "dataset/%03d", i);

	while ((_access(filename, 0)) == 0)
	{
		sprintf_s(filename, 25, "dataset/%03d", ++i);
	}

	// dir
	sprintf_s(dataset_dir, 25, "%s", filename);
	CreateDirectory(dataset_dir, NULL);

	// filename
	sprintf_s(filename, 25, "%s/output.yaml", dataset_dir);

	// file
	//fout.open(filename, ios::out); // SLOW :(
	fopen_s (&file_out, filename , "w");

	printf("Created dataset %03d\n", i);


	// Semaphore
	ghSemaphore = CreateSemaphore( 
        NULL,           // default security attributes
        MAX_SEM_COUNT,  // initial count
        MAX_SEM_COUNT,  // maximum count
        NULL);          // unnamed semaphore

    if (ghSemaphore == NULL) 
        printf("CreateSemaphore error: %d\n", GetLastError());
}