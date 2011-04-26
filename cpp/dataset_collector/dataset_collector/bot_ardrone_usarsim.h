#pragma once
#include "botinterface.h"
#include "mysocket.h"
#include <string>

#define BOT_ARDRONE_USARSIM_SOCKET_CONTROL 1
#define BOT_ARDRONE_USARSIM_SOCKET_FRAME 2

// forward declarations
class bot_ardrone;
struct bot_ardronBOT_EVENT_CONTROL;
struct bot_ardronBOT_EVENT_MEASUREMENT;
struct bot_ardronBOT_EVENT_FRAME;


using namespace std;


class bot_ardrone_usarsim : public botinterface
{
public:
	bot_ardrone_usarsim(bot_ardrone *bot);
	~bot_ardrone_usarsim(void);

	void init();

	/* control */
	void control_update(void *control);
	void control_send(char *message);

	/* handlers */
	void socket_callback(int id, char *message, int bytes);
	void process_measurement(char *message, int bytes);
	void process_frame(char *message, int bytes);

	void bot_ardrone_usarsim::reset_frame(bot_ardronBOT_EVENT_FRAME *f);

	bot_ardronBOT_EVENT_FRAME *frame;

private:
	bot_ardrone *bot;
	mysocket *control_socket;
	mysocket *frame_socket;
};