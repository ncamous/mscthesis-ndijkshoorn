#pragma once
#include "botinterface.h"
#include "mysocket.h"

#define BOT_ARDRONE_USARSIM_SOCKET_CONTROL 1
#define BOT_ARDRONE_USARSIM_SOCKET_CAM 2

class bot_ardrone;

class bot_ardrone_usarsim : public botinterface
{
public:
	bot_ardrone_usarsim(bot_ardrone *bot);
	~bot_ardrone_usarsim(void);

	void init();
	void control_send(char *message);
	void socket_callback(int id, char *message, int bytes);

	int cam_header_size;
	char cam_header[4];

	int cam_image_dest_size;
	int cam_image_size;
	char *cam_image;

private:
	bot_ardrone *bot;
	mysocket *control_socket;
	mysocket *cam_socket;
};

