#include "global.h"
#include "bot_ardrone_usarsim.h"
#include "bot_ardrone.h"
#include "mysocket.h"


bot_ardrone_usarsim::bot_ardrone_usarsim(bot_ardrone *bot)
{
	this->bot = bot;

	cam_header_size = 0;
	cam_image_dest_size = 0;
	cam_image_size = 0;

	control_socket = new mysocket(BOT_ARDRONE_USARSIM_SOCKET_CONTROL, DEFAULT_USARIM_PORT, DEFAULT_USARSIM_IP, 300, (botinterface*) this);
	cam_socket = new mysocket(BOT_ARDRONE_USARSIM_SOCKET_CAM, DEFAULT_UPIS_PORT, DEFAULT_USARSIM_IP, 5000, (botinterface*) this);
}


bot_ardrone_usarsim::~bot_ardrone_usarsim(void)
{
}


void bot_ardrone_usarsim::init(void)
{
	control_send("INIT {ClassName USARBot.ARDrone} {Location 0.0,0.0,0.8}\r\n");
}


void bot_ardrone_usarsim::control_send(char *message)
{
	control_socket->send(message);
}

void bot_ardrone_usarsim::socket_callback(int id, char *message, int bytes)
{

	switch (id)
	{
		case BOT_ARDRONE_USARSIM_SOCKET_CONTROL:
			message[bytes] = '\0';
			printf("CONTROL RECEIVE: %s\n", message);
			break;

		case BOT_ARDRONE_USARSIM_SOCKET_CAM:
			int copy_len;

			// we dont know image size (bytes) yet
			if (cam_image_dest_size == 0)
			{
				copy_len = min(5-cam_header_size, bytes);
				memcpy(&cam_header[cam_header_size], message, copy_len);
				cam_header_size += copy_len;

				if (cam_header_size == 5) {
					cam_image_dest_size = (cam_image_dest_size << 8) + cam_header[1];
					cam_image_dest_size = (cam_image_dest_size << 8) + cam_header[2];
					cam_image_dest_size = (cam_image_dest_size << 8) + cam_header[3];
					cam_image_dest_size = (cam_image_dest_size << 8) + cam_header[4];
					cam_image = new char[cam_image_dest_size];
				}

				// we have bytes left for the image
				bytes -= copy_len;
				if (bytes > 0)
					message = &message[copy_len];
			}


			// copy to image
			if (cam_image_dest_size > 0 && bytes > 0) {
				copy_len = min(bytes, cam_image_dest_size-cam_image_size);
				memcpy(&cam_image[cam_image_size], message, copy_len);
				cam_image_size += bytes;
			}

			// image complete
			if (cam_image_size > 5000 && bytes != 5000 /*cam_image_dest_size > 0 && cam_image_size >= cam_image_dest_size*/) {
				bot->cam_received(cam_image, cam_image_dest_size);

				delete[] cam_image;
				cam_header_size = 0;
				cam_image_dest_size = 0;
				cam_image_size = 0;

				// request new image
				//printf("SEND OK\n");
				Sleep(500);
				cam_socket->send("OK");
			}

			break;

	}
}