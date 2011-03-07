#include "global.h"
#include "bot_ardrone_recorder.h"
#include "bot_ardrone.h"
#include "yaml.h"
#include <io.h>


bot_ardrone_recorder::bot_ardrone_recorder(bot_ardrone *bot)
{
	this->bot = bot;

	frame_counter = 1;
}


bot_ardrone_recorder::~bot_ardrone_recorder(void)
{
	// todo
}


void bot_ardrone_recorder::record_measurement(bot_ardrone_measurement *m)
{
	fprintf (file_out, "---\n");
	fprintf (file_out, "e: %i\n", BOT_ARDRONE_EVENT_MEASUREMENT);
	fprintf (file_out, "t: %f\n", m->time);

	// usarsim only
	if (m->usarsim)
	{
		fprintf (file_out, "s: %i\n", m->sensor); // dont want this
		fprintf (file_out, "gt_loc:\n  - %f\n  - %f\n  - %f\n", m->gt_loc[0], m->gt_loc[1], m->gt_loc[2]);
		fprintf (file_out, "gt_or:\n  - %f\n  - %f\n  - %f\n", m->gt_or[0], m->gt_or[1], m->gt_or[2]);
		fprintf (file_out, "gt_vel:\n  - %f\n  - %f\n  - %f\n", m->gt_vel[0], m->gt_vel[1], m->gt_vel[2]);
	}

	fprintf (file_out, "bat: %i\n", m->battery);
	fprintf (file_out, "alt: %i\n", m->altitude);
	fprintf (file_out, "ins_or:\n  - %f\n  - %f\n  - %f\n", m->ins_or[0], m->ins_or[1], m->ins_or[2]);
	fprintf (file_out, "ins_vel:\n  - %f\n  - %f\n  - %f\n", m->ins_vel[0], m->ins_vel[1], m->ins_vel[2]);
}


void bot_ardrone_recorder::record_control(bot_ardrone_control *c)
{
	fprintf (file_out, "---\n");
	fprintf (file_out, "e: %i\n", BOT_ARDRONE_EVENT_CONTROL);
	fprintf (file_out, "t: %f\n", c->time);
	fprintf (file_out, "v:\n  - %f\n  - %f\n  - %f\n", c->velocity[0], c->velocity[1], c->velocity[2]);
}


void bot_ardrone_recorder::record_frame(bot_ardrone_frame *f)
{
	char filename[25];

	sprintf_s(f->filename, 20, "%06d%s", frame_counter++, BOT_ARDRONE_RECORDER_FRAME_EXT);
	sprintf_s(filename, 25, "%s/%s", dataset_dir, f->filename);

	fprintf (file_out, "---\n");
	fprintf (file_out, "e: %i\n", BOT_ARDRONE_EVENT_FRAME);
	fprintf (file_out, "t: %f\n", f->time);
	fprintf (file_out, "s: %i\n", f->data_size);
	fprintf (file_out, "f: %s\n", f->filename);

	ofstream frame_out(filename, ios::out | ios::binary);
	frame_out.write(f->data, f->data_size);
	frame_out.close();
}


void bot_ardrone_recorder::playback(char *dataset)
{
	/*
	char filename[25];

	sprintf_s(dataset_dir, 25, "dataset/%s", dataset);
	sprintf_s(filename, 25, "%s/output.yaml", dataset_dir);

	// check
	if (fin.is_open())
		printf("ERROR: DATASET PLAYBACK FILE ALREADY OPEN!\n");

	int event_type;
	YAML::Node doc;

	fin.open(filename, ios::in);
	YAML::Parser parser(fin);

    while(parser.GetNextDocument(doc))
	{
		doc["e"] >> event_type;

		switch (event_type)
		{
			case BOT_ARDRONE_EVENT_MEASUREMENT:
			{
				bot_ardrone_measurement m;
				doc["time"] >> m.t;
				doc["sensor"] >> m.s;
				doc["bat:"] >> m.b;
				doc["gt_loc"] >> m.gt_loc;
				doc["gt_or"] >> m.gt_or;
				doc["ins_loc"] >> m.ins_loc;
				doc["ins_or"] >> m.ins_or;
				doc["sonar"] >> m.sonar;

				bot->measurement_received(&m);
				break;
			}
		
			case BOT_ARDRONE_EVENT_CONTROL:
			{
				bot_ardrone_control c;
				doc["time"] >> c.t;
				doc["velocity"] >> c.v;

				bot->control_update(&c);
				break;
			}

			case BOT_ARDRONE_EVENT_FRAME:
			{
				char filename[30];
				string tmpstring;
				bot_ardrone_frame f;
				doc["time"] >> f.time;
				doc["data_size"] >> f.data_size;
				doc["filename"] >> tmpstring;
				strcpy_s(f.filename, 30, tmpstring.c_str());

				sprintf_s(filename, 30, "%s/%s", dataset_dir, f.filename);
				ifstream frame_in(filename, ios::in | ios::binary);

				// data buffer
				frame_in.read(f.data, f.data_size);
				frame_in.close();

				bot->frame_received(&f);

				break;
			}
		}
	}
	*/
}


void bot_ardrone_recorder::prepare_dataset()
{
	if (fout.is_open())
		printf("ERROR: DATASET OUTPUT FILE ALREADY OPEN!\n");

	int i = 1;
	char filename[25];

	sprintf_s(filename, 25, "dataset/%03d", i);

	while ((_access(filename, 0)) == 0)
	{
		sprintf_s(filename, 25, "dataset/%03d", ++i);
	}

	// dir
	sprintf_s(dataset_dir, 25, "%s", filename);
	CreateDirectory(dataset_dir, NULL);

	// filename
	sprintf_s(filename, 25, "%s/output.yaml", dataset_dir);

	// file
	//fout.open(filename, ios::out); // SLOW :(
	fopen_s (&file_out, filename , "w");

	printf("Created dataset %03d\n", i);
}



/* operators */
YAML::Emitter& operator << (YAML::Emitter& out, const float *f)
{
	int i;
	out << YAML::BeginSeq;
	for(i = 0; i < 3; i++)
		out << f[i];
	out << YAML::EndSeq;
	return out;
}


void operator >> (const YAML::Node& node, float *f)
{
	int i;
	for(i = 0; i < 3; i++)
		node[i] >> f[i];
}
/* end operators */