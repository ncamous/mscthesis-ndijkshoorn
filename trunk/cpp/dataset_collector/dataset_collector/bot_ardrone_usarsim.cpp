#include "global.h"
#include "bot_ardrone_usarsim.h"
#include "bot_ardrone.h"
#include "mysocket.h"
#include <string>

using namespace std;

bot_ardrone_usarsim::bot_ardrone_usarsim(bot_ardrone *bot)
{
	this->bot = bot;

	frame = new bot_ardrone_frame;

	/* sockets */
	control_socket = new mysocket(BOT_ARDRONE_USARSIM_SOCKET_CONTROL, USARIM_PORT, USARSIM_IP, NULL, USARSIM_CONTROL_BLOCKSIZE, (botinterface*) this);
	frame_socket = new mysocket(BOT_ARDRONE_USARSIM_SOCKET_CAM, UPIS_PORT, USARSIM_IP, frame->data, USARSIM_FRAME_BLOCKSIZE, (botinterface*) this);
}


bot_ardrone_usarsim::~bot_ardrone_usarsim(void)
{
}


void bot_ardrone_usarsim::init(void)
{
	control_send("INIT {ClassName USARBot.ARDrone} {Name ARDrone} {Location 0.0,0.0,0.8}\r\n");
	control_send("SET {Type Viewports} {Config SingleView} {Viewport1 Camera2}\r\n");
	//control_send("SET {Type Camera} {Robot ARDrone} {Name Camera2} {Client 10.0.0.2}\r\n");
	//control_send("SET {Type Viewports} {Config QuadView} {Viewport1 Camera} {Viewport2 Camera2}\r\n");
}


void bot_ardrone_usarsim::control_update(bot_ardrone_control &control)
{
	char msg[200];

	sprintf_s(msg, 200, "DRIVE {AltitudeVelocity %f} {LinearVelocity %f} {LateralVelocity %f} {RotationalVelocity %f} {Normalized false}\r\n",
		control.velocity[BOT_ARDRONE_AltitudeVelocity],
		control.velocity[BOT_ARDRONE_LinearVelocity],
		control.velocity[BOT_ARDRONE_LateralVelocity],
		control.velocity[BOT_ARDRONE_RotationalVelocity]);

	control_send(msg);
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
			process_measurement(message, bytes);
			break;

		case BOT_ARDRONE_USARSIM_SOCKET_CAM:
			process_frame(message, bytes);
			break;
	}
}


void bot_ardrone_usarsim::process_measurement(char *message, int bytes)
{
	string type;
	size_t found;
	size_t tmppos1, tmppos2;
	int i;

	message[bytes] = '\0';

	string message_str(message);
	string key, val, tmp;

	type = message_str.substr(0, 3);

	// skip NFO's
	if (strcmp(type.c_str(), "NFO") == 0)
		return;


	bot_ardrone_measurement m;
	int offset = 0;


	// type
	if (strcmp(type.c_str(), "STA") == 0)
		m.type = BOT_ARDRONE_MEASUREMENT_STA;
	else if (strcmp(type.c_str(), "SEN") == 0)
		m.type = BOT_ARDRONE_MEASUREMENT_SEN;


	while ((found = message_str.find('{', offset)) != string::npos)
	{
		tmppos1 = message_str.find(' ', found + 2);
		if (tmppos1 == string::npos)
			break;

		key = message_str.substr(found+1, tmppos1-found-1);

		tmppos2 = message_str.find('}', tmppos1+2);
		if (tmppos2 == string::npos)
			break;

		val = message_str.substr(tmppos1+1, tmppos2-tmppos1-1);
		offset = tmppos2 + 1;


		/**/
		if (strcmp(key.c_str(), "Battery") == 0)
		{
			m.battery = atoi(val.c_str());
		}

		if (strcmp(key.c_str(), "Location") == 0)
		{
			tmppos1 = 0;
			for(i = 0; i < 3; i++)
			{
				if (i < 2)
					tmppos2 = val.find(',', tmppos1);
				else
					tmppos2 = val.length();

				tmp = val.substr(tmppos1, tmppos2-tmppos1);

				m.groundtruth_loc[i] = strtod(tmp.c_str(), NULL);
				tmppos1 = tmppos2 + 1;
			}
		}
		/**/
	}

	bot->measurement_received(&m);
}


void bot_ardrone_usarsim::process_frame(char *message, int bytes)
{
	// update frame size and buffer pointer
	frame->data_size += bytes;
	frame_socket->buffer += bytes;

	// we dont know image size (bytes) yet
	if (frame->dest_size == 0 && frame->data_size >= 5)
	{
		frame->dest_size = (frame->dest_size << 8) + frame->data[1];
		frame->dest_size = (frame->dest_size << 8) + frame->data[2];
		frame->dest_size = (frame->dest_size << 8) + frame->data[3];
		frame->dest_size = (frame->dest_size << 8) + frame->data[4];
	}

	// image complete
	if (frame->dest_size > 0 && frame->data_size-5 >= frame->dest_size) {
		if (!frame_socket->bytes_waiting())
		{
			/*if (frame->data_size-5 > frame->dest_size)
			{
				printf("TEST TYPE: %i\n", frame->data[frame->dest_size + 5]);
				Sleep(4000);
			}*/

			// remove header
			frame->data += 5;
			frame->data_size -= 5;

			bot->frame_received(frame);

			// delete struct here
			frame = NULL;

			// request new image
			frame = new bot_ardrone_frame;
			frame_socket->buffer = frame->data;

			Sleep(BOT_ARDRONE_USARSIM_FRAME_REQDELAY);
			frame_socket->send("OK");
			Sleep(20);
		}
	}
}


/*
void bot_ardrone_usarsim::process_frame(char *message, int bytes)
{
	int copy_len;

	if (frame == NULL)
		frame = new bot_ardrone_frame;

	// we dont know image size (bytes) yet
	if (frame->dest_size == 0)
	{
		copy_len = min(5 - frame->header_size, bytes);
		memcpy(&frame->header[frame->header_size], message, copy_len);
		frame->header_size += copy_len;

		if (frame->header_size == 5) {
			frame->dest_size = (frame->dest_size << 8) + frame->header[1];
			frame->dest_size = (frame->dest_size << 8) + frame->header[2];
			frame->dest_size = (frame->dest_size << 8) + frame->header[3];
			frame->dest_size = (frame->dest_size << 8) + frame->header[4];
			//printf("IMG SIZE: %i\n", frame->dest_size);
			//printf("IMG TYPE: %i\n", frame->header[0]);
			frame->data = new char[frame->dest_size + 4*USARSIM_FRAME_BLOCKSIZE]; // reserve extra space
		}

		// we have bytes left for the image
		bytes -= copy_len;
		if (bytes > 0)
			message = &message[copy_len];
	}

	// copy to image
	if (frame->dest_size > 0 && bytes > 0) {
		memcpy(&frame->data[frame->data_size], message, bytes);
		frame->data_size += bytes;
	}

	// image complete
	if (frame->dest_size > 0 && frame->data_size >= frame->dest_size) {
		if (!frame_socket->bytes_waiting())
		{
			bot->frame_received(frame);

			// delete struct here
			frame = NULL;

			// request new image
			Sleep(BOT_ARDRONE_USARSIM_FRAME_REQDELAY);
			frame_socket->send("OK");
			Sleep(20);
		}
	}
}
*/