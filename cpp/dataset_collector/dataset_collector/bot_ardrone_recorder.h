#pragma once
#include <windows.h>
#include <fstream>
#include "yaml.h"


// Semaphore
#define MAX_SEM_COUNT 10
#define THREADCOUNT 3


class bot_ardrone;
struct bot_ardronBOT_EVENT_MEASUREMENT;
struct bot_ardronBOT_EVENT_CONTROL;
struct bot_ardronBOT_EVENT_FRAME;

using namespace std;


class bot_ardrone_recorder
{
public:
	bot_ardrone_recorder(bot_ardrone *bot);
	~bot_ardrone_recorder(void);

	/* record */
	void record_measurement(bot_ardronBOT_EVENT_MEASUREMENT *m);
	void record_control(bot_ardronBOT_EVENT_CONTROL *c);
	void record_frame(bot_ardronBOT_EVENT_FRAME *f);

	/* playback */
	void playback(char *dataset);

	/* other */
	void prepare_dataset();

private:
	bot_ardrone *bot;
	char dataset_dir[25];
	ifstream fin;
	int frame_counter;

	// resource sharing
	static HANDLE ghSemaphore;

	/* faster */
	FILE *file_out;
};