#pragma once
#include "botinterface.h"
#include <string>

#include <navdata_common.h>
// so i dont have to include header files from the ARDroneLib
//class navdata_demo_t;

class bot_ardrone;
struct bot_ardrone_control;
struct bot_ardrone_measurement;
struct bot_ardrone_frame;

using namespace std;

extern "C" {
	void bot_ardrone_ardronelib_navdata_callback(navdata_demo_t *navdata);
	void bot_ardrone_ardronelib_video_callback(unsigned char* rgbtexture, int w, int h);
}

static DWORD WINAPI start_ardrone_api_thread(void* Param);


class bot_ardrone_ardronelib : public botinterface
{
public:
	bot_ardrone_ardronelib(bot_ardrone *bot);
	~bot_ardrone_ardronelib(void);

	void init();
	void control_update(void *control);
	void control_send(char *message);
	void process_measurement(navdata_demo_t *navdata);
	void process_frame(unsigned char* rgbtexture, int w, int h);
	static bot_ardrone_ardronelib* instance();
	void socket_callback(int id, char *message, int bytes); // not used

	static bot_ardrone_ardronelib* myinstance;
	bot_ardrone_frame *frame;

private:
	bot_ardrone *bot;
	HANDLE ardrone_thread;
};