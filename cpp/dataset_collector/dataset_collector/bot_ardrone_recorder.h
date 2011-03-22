#pragma once
#include <windows.h>
#include <fstream>
#include "yaml.h"


// Semaphore
#define MAX_SEM_COUNT 10
#define THREADCOUNT 3


class bot_ardrone;
struct bot_ardrone_measurement;
struct bot_ardrone_control;
struct bot_ardrone_frame;

void YAML_double3(const YAML::Node& node, float *f);
void YAML_float3(const YAML::Node& node, float *f);
void YAML_float4(const YAML::Node& node, float *f);

using namespace std;


class bot_ardrone_recorder
{
public:
	bot_ardrone_recorder(bot_ardrone *bot);
	~bot_ardrone_recorder(void);

	/* record */
	void record_measurement(bot_ardrone_measurement *m);
	void record_control(bot_ardrone_control *c);
	void record_frame(bot_ardrone_frame *f);

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