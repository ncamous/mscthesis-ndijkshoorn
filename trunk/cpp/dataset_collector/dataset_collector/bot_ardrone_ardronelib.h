#pragma once
#include "botinterface.h"
#include <windows.h>
#include <string>
#include "navdata_common.h"
#include "ardrone_api.h"


#define DRONE_VIDEO_MAX_WIDTH 640
#define DRONE_VIDEO_MAX_HEIGHT 480

class bot_ardrone;
struct bot_ardronBOT_EVENT_CONTROL;
struct bot_ardronBOT_EVENT_MEASUREMENT;
struct bot_ardronBOT_EVENT_FRAME;


using namespace std;


// C application calls these functions
extern "C" {
	void bot_ardrone_ardronelib_process_navdata(navdata_unpacked_t *n);
	void bot_ardrone_ardronelib_process_frame(unsigned char* rgbtexture, int w, int h);
};


static DWORD WINAPI start_ardrone_api_thread(void* Param);


class bot_ardrone_ardronelib : public botinterface
{
public:
	bot_ardrone_ardronelib(bot_ardrone *bot);
	~bot_ardrone_ardronelib(void);

	void init();

	/* control */
	void control_update(void *control);
	void take_off();
	void land();

	/* handlers */
	void process_measurement(navdata_unpacked_t *n);
	void process_frame(unsigned char* rgbtexture, int w, int h);

	static bot_ardrone_ardronelib* instance();

	static bot_ardrone_ardronelib* myinstance;
	bot_ardronBOT_EVENT_FRAME *frame;

private:
	bot_ardrone *bot;
	HANDLE ardrone_thread;

	// temp
	int m_counter;
};