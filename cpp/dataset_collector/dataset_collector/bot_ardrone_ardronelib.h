#pragma once
#include "botinterface.h"
#include <string>

class bot_ardrone;
struct bot_ardrone_control;
struct bot_ardrone_measurement;
struct bot_ardrone_frame;

// so i dont have to include header files from the ARDroneLib
class navdata_demo_t;

using namespace std;

extern "C" void bot_ardrone_ardronelib_process_navdata(navdata_demo_t*);

class bot_ardrone_ardronelib : public botinterface
{
public:
	bot_ardrone_ardronelib(bot_ardrone *bot);
	~bot_ardrone_ardronelib(void);

	void init();
	void control_update(void *control);
	void control_send(char *message);
	void socket_callback(int id, char *message, int bytes);
	void process_measurement(char *message, int bytes);
	void process_frame(char *message, int bytes);
	static bot_ardrone_ardronelib* instance();

	static bot_ardrone_ardronelib* myinstance;
	bot_ardrone_frame *frame;

private:
	bot_ardrone *bot;
};