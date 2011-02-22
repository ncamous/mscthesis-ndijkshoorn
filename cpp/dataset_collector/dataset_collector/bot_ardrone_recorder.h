#pragma once
#include <fstream>

class bot_ardrone;

struct bot_ardrone_measurement;
struct bot_ardrone_control;
struct bot_ardrone_frame;

using namespace std;


class bot_ardrone_recorder
{
public:
	bot_ardrone_recorder(bot_ardrone *bot);
	~bot_ardrone_recorder(void);
	void record_measurement(bot_ardrone_measurement *m);
	void record_control(bot_ardrone_control *c);
	void record_frame(bot_ardrone_frame *f);
	void playback(char *dataset);
	void prepare_dataset();

private:
	bot_ardrone *bot;
	char dataset_dir[25];
	ofstream fout;
	ifstream fin;
	int frame_counter;
};

