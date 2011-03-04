#include "global.h"
#include "bot_ardrone_usarsim.h"
#include "bot_ardrone.h"
#include "usarsim_msgparser.h"
#include "mysocket.h"

using namespace std;

bot_ardrone_usarsim::bot_ardrone_usarsim(bot_ardrone *bot)
{
	this->bot = bot;

	frame = new bot_ardrone_frame;

	/* sockets */
	control_socket = new mysocket(BOT_ARDRONE_USARSIM_SOCKET_CONTROL, usarsim_PORT, USARSIM_IP, NULL, USARSIM_CONTROL_BLOCKSIZE, (botinterface*) this);
	frame_socket = new mysocket(BOT_ARDRONE_USARSIM_SOCKET_CAM, UPIS_PORT, USARSIM_IP, frame->data, USARSIM_FRAME_BLOCKSIZE, (botinterface*) this);
}


bot_ardrone_usarsim::~bot_ardrone_usarsim(void)
{
}


void bot_ardrone_usarsim::init(void)
{
	control_send("INIT {ClassName USARBot.ARDrone} {Name ARDrone} {Location 0.0,0.0,25.0}\r\n");
	control_send("SET {Type Viewports} {Config SingleView} {Viewport1 Camera2}\r\n");
	//control_send("SET {Type Camera} {Robot ARDrone} {Name Camera2} {Client 10.0.0.2}\r\n");
	//control_send("SET {Type Viewports} {Config QuadView} {Viewport1 Camera} {Viewport2 Camera2}\r\n");
}


void bot_ardrone_usarsim::control_update(void *control)
{
	bot_ardrone_control *control_struct = (bot_ardrone_control*) control;

	char msg[200];

	sprintf_s(msg, 200, "DRIVE {AltitudeVelocity %f} {LinearVelocity %f} {LateralVelocity %f} {RotationalVelocity %f} {Normalized false}\r\n",
		control_struct->velocity[BOT_ARDRONE_AltitudeVelocity],
		control_struct->velocity[BOT_ARDRONE_LinearVelocity],
		control_struct->velocity[BOT_ARDRONE_LateralVelocity],
		control_struct->velocity[BOT_ARDRONE_RotationalVelocity]);

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
			if (bytes >= USARSIM_CONTROL_BLOCKSIZE)
				printf("ERROR: USARSIM MEASUREMENT BUFFER FULL!\n");
			else
				process_measurement(message, bytes);
			break;

		case BOT_ARDRONE_USARSIM_SOCKET_CAM:
			process_frame(message, bytes);
			break;
	}
}


void bot_ardrone_usarsim::process_measurement(char *message, int bytes)
{
	int pos;
	int lineoffset = 0;

	message[bytes] = '\0';
	string msg(message);

	while ((pos = msg.find('\n', lineoffset)) != string::npos)
	{
		// get line
		string line(message, lineoffset, pos-lineoffset-1);
		lineoffset = pos+1;

		string type(line, 0, 3);

		// skip NFO's en RES'
		if (type == "NFO" || type == "RES")
			return;

		bot_ardrone_measurement m;
		m.usarsim = true;

		// BOT_ARDRONE_MEASUREMENT_STA
		if (type == "STA")
		{
			//m.type = BOT_ARDRONE_MEASUREMENT_STA;
			m.battery = usarsim_msgparser_int(&line, "{Battery");
		}

		// BOT_ARDRONE_MEASUREMENT_SEN
		else if (type == "SEN")
		{
			//m.type = BOT_ARDRONE_MEASUREMENT_SEN;
			m.sensor = usarsim_msgparser_type(&line);

			if (m.sensor == BOT_ARDRONE_SENSOR_UNKNOW)
				printf("UNKNOW SENSOR TYPE\n");

			switch (m.sensor)
			{
				case BOT_ARDRONE_SENSOR_GT:
				{
					usarsim_msgparser_float3(&line, "{Location", m.gt_loc);
					usarsim_msgparser_float3(&line, "{Orientation", m.gt_or);
					// todo velocity / acceleration
					break;
				}

				case BOT_ARDRONE_SENSOR_INS:
				{
					//printf("%s\n", message);
					//usarsim_msgparser_float3(&line, "{Location", m.ins_loc);
					usarsim_msgparser_float3(&line, "{Orientation", m.ins_or);
					// todo velocity / acceleration
					break;
				}

				case BOT_ARDRONE_SENSOR_SONAR:
				{
					m.altitude = (int)usarsim_msgparser_float(&line, "Name Sonar1 Range");
					break;
				}
			}
		}

		bot->measurement_received(&m);
	} // get line
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