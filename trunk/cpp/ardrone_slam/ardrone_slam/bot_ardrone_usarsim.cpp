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
	frame->usarsim = true;

	/* sockets */
	printf("Connecting to USARSim\n");
	control_socket = new mysocket(BOT_ARDRONE_USARSIM_SOCKET_CONTROL, USARSIM_PORT, USARSIM_IP, NULL, BOT_ARDONE_USARSIM_CONTROL_BUFSIZE, (botinterface*) this);
	printf("Connecting to UPIS\n");
	frame_socket = new mysocket(BOT_ARDRONE_USARSIM_SOCKET_FRAME, UPIS_PORT, USARSIM_IP, frame->data, /*BOT_ARDRONBOT_EVENT_FRAME_BUFSIZE*/BOT_ARDRONE_USARSIM_FRAME_BLOCKSIZE, (botinterface*) this);
}


bot_ardrone_usarsim::~bot_ardrone_usarsim(void)
{
}


void bot_ardrone_usarsim::init(void)
{
	//control_send("INIT {ClassName USARBot.ARDrone} {Name ARDrone} {Location 0.0,0.0,1.28}\r\n");

	// ROBOCUP 2006
	//control_send("INIT {ClassName USARBot.ARDrone} {Name ARDrone} {Location -12600.0,800.0,1000.0}\r\n");

	// doolhof
	//control_send("INIT {ClassName USARBot.ARDrone} {Name ARDrone} {Location -52.0,5.68,-4.0}\r\n");

	// zebrapad
	control_send("INIT {ClassName USARBot.ARDrone} {Name ARDrone} {Location -19.3,57.1,-1.1}\r\n");

	// 0,0,0
	//control_send("INIT {ClassName USARBot.ARDrone} {Name ARDrone} {Location 0.0,10.0,-5.0}\r\n");

	control_send("SET {Type Viewports} {Config SingleView} {Viewport1 Camera2}\r\n");
	//control_send("SET {Type Camera} {Robot ARDrone} {Name Camera2} {Client 10.0.0.2}\r\n");
	//control_send("SET {Type Viewports} {Config QuadView} {Viewport1 Camera} {Viewport2 Camera2}\r\n");

	// at 50cm height
	char *msg = "DRIVE {AltitudeVelocity 0.0} {LinearVelocity 0.0} {LateralVelocity 0.0} {RotationalVelocity 0.0} {Normalized false}\r\n";
	control_send(msg);
}


void bot_ardrone_usarsim::control_update(void *control)
{
	bot_ardrone_control *c = (bot_ardrone_control*) control;

	char msg[200];

	sprintf_s(msg, 200, "DRIVE {AltitudeVelocity %f} {LinearVelocity %f} {LateralVelocity %f} {RotationalVelocity %f} {Normalized false}\r\n",
		c->velocity[BOT_ARDRONE_AltitudeVelocity],
		c->velocity[BOT_ARDRONE_LinearVelocity],
		c->velocity[BOT_ARDRONE_LateralVelocity],
		c->velocity[BOT_ARDRONE_RotationalVelocity]);

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
			if (bytes >= BOT_ARDRONBOT_EVENT_FRAME_BUFSIZE)
				printf("ERROR: USARSIM MEASUREMENT BUFFER FULL!\n");
			else
				process_measurement(message, bytes);
			break;

		case BOT_ARDRONE_USARSIM_SOCKET_FRAME:
			process_frame(message, bytes);
			break;
	}
}


void bot_ardrone_usarsim::process_measurement(char *message, int bytes)
{
	int pos;
	int lineoffset = 0;
	bot_ardrone_measurement *m = NULL;

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
		{
			return;
		}

		// BOT_ARDRONBOT_EVENT_MEASUREMENT_STA
		else if (type == "STA")
		{
			bot->battery = (int) (((float)usarsim_msgparser_int(&line, "{Battery") / (float)BOT_ARDRONE_BATTERYLIFE)*100.0f);
		}

		// BOT_ARDRONBOT_EVENT_MEASUREMENT_SEN
		else if (type == "SEN")
		{
			if (m == NULL)
			{
				m = new bot_ardrone_measurement;
				m->usarsim = true;
				m->type = BOT_ARDRONBOT_EVENT_MEASUREMENT_SEN;
				m->vel[2] = 100.0f;
			}

			m->sensor = usarsim_msgparser_type(&line);

			switch (m->sensor)
			{
				case BOT_ARDRONE_SENSOR_GT:
				{
					//printf("%s\n", line.c_str());

					usarsim_msgparser_float3(&line, "{Location", m->gt_loc);
					break;
				}

				case BOT_ARDRONE_SENSOR_IMU:
				{
					//printf("%s\n", line.c_str());

					/*
					usarsim_msgparser_float3(&line, "{Orientation", m->or);
					
					// rad to mili-degrees
					m->or[0] = usarsim_msgparser_rad_to_mildeg(m->or[0]); // x
					m->or[1] = usarsim_msgparser_rad_to_mildeg(m->or[1]); // y
					m->or[2] = usarsim_msgparser_rad_to_mildeg(m->or[2]); // z
					*/
					break;
				}

				case BOT_ARDRONE_SENSOR_SONAR:
				{
					m->altitude = (int) (usarsim_msgparser_float(&line, "Name Sonar1 Range")*1000.0f);
					break;
				}

				case BOT_ARDRONE_SENSOR_ACCEL:
					//printf("%s\n", line.c_str());

					// Accelerations are received in m/s2
					// a = dv/dt = (vfinal - vinitial) / (tfinal - tinitial) 
					usarsim_msgparser_float3(&line, "{Acceleration", m->accel);
					m->accel[0] = usarsim_msgparser_ms2_to_mg(m->accel[0]); // m -> cm
					m->accel[1] = usarsim_msgparser_ms2_to_mg(m->accel[1]);
					m->accel[2] = usarsim_msgparser_ms2_to_mg(m->accel[2]);

					/*
					usarsim_msgparser_float3(&line, "{Velocity", m->vel);
					m->vel[0] *= 1000.0f; // cm -> mm
					m->vel[1] *= 1000.0f;
					m->vel[2] *= 1000.0f;
					*/

					// get orientation from GT
					usarsim_msgparser_float3(&line, "{Orientation", m->or);

					// rad to mili-degrees
					m->or[0] = usarsim_msgparser_rad_to_mildeg(m->or[0]); // x
					m->or[1] = usarsim_msgparser_rad_to_mildeg(m->or[1]); // y
					m->or[2] = usarsim_msgparser_rad_to_mildeg(m->or[2]); // z

					break;
			}
		}
	} // get line

	if (m != NULL)
		bot->measurement_received(m);
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
		//frame->dest_size = ntohl((long) frame->dest_size);

		// fix by Sander
		frame->time = bot->get_clock(); // get clock time now
	}

	//printf("Received %i of advertised %i\n", frame->data_size, frame->dest_size);

	// image complete
	if (frame->dest_size > 0 && frame->data_size-5 >= frame->dest_size) {
		if (USARIM_FRAME_USERAW || !frame_socket->bytes_waiting())
		{
			//printf("completed with %i of advertised %i\n", frame->data_size-5, frame->dest_size); 

			// remove header
			frame->data += 5;
			frame->data_size -= 5;

			bot->frame_received(frame);

			//reset_frame(frame);
			frame = new bot_ardrone_frame;
			frame->usarsim = true;
			frame_socket->buffer = frame->data;

			// slam enabled and queue not empty -> wait
			if (BOT_ARDRONE_USARSIM_FRAME_MODE == 1 && bot->slam_state)
			{
				//printf("waiting for event\n");
				bot->slamcontroller->queue_frame.wait_until_empty(INFINITE); // move this function to bot_ardrone?
				//printf("done waiting\n");
			}
			// slam not (yet) enabled, or fixed fps mode
			else if ((BOT_ARDRONE_USARSIM_FRAME_MODE == 1 && !bot->slam_state) || BOT_ARDRONE_USARSIM_FRAME_MODE == 2)
				Sleep(BOT_ARDRONE_USARSIM_FRAME_REQDELAY - 30);

			frame_socket->send("OK");
			Sleep(30); // wait a bit before receiving new frame data
		}
	}
}


void bot_ardrone_usarsim::reset_frame(bot_ardrone_frame *f)
{
	f->data = f->data_start;
	f->data_size = f->dest_size = 0;
	f->filename[0] = '\0';
}