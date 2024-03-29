#include <time.h>
#ifndef _WIN32
	#include <sys/time.h>
#else
    #include <sys/timeb.h>
    #include <Winsock2.h>  // for timeval structure
#endif

#include <VP_Os/vp_os_malloc.h>

#include <config.h>
#include <ardrone_tool/Academy/academy_stage_recorder.h>
#include <utils/ardrone_date.h>

#define ACADEMY_RECORD_FILE_EXTENSION "enc"
#define ACADEMY_RECORD_FILE_EXTENSION_INPROGRESS "tmp"

#ifndef ACADEMY_FILE_DEFAULT_PATH
#define ACADEMY_FILE_DEFAULT_PATH flight_dir
extern char flight_dir[];
#endif

static void ardrone_academy_stage_recorder_internal_close(ardrone_academy_stage_recorder_config_t *cfg);

ardrone_academy_stage_recorder_config_t ardrone_academy_stage_recorder_config;

const vp_api_stage_funcs_t ardrone_academy_stage_recorder_funcs = {
  (vp_api_stage_handle_msg_t) NULL,
  (vp_api_stage_open_t) ardrone_academy_stage_recorder_open,
  (vp_api_stage_transform_t) ardrone_academy_stage_recorder_transform,
  (vp_api_stage_close_t) ardrone_academy_stage_recorder_close
};

bool_t ardrone_academy_stage_recorder_state(void)
{
	return (ardrone_academy_stage_recorder_config.startRec != ACADEMY_RECORD_STOP);
}

C_RESULT
ardrone_academy_stage_recorder_handle (ardrone_academy_stage_recorder_config_t * cfg, PIPELINE_MSG msg_id, void *callback, void *param)
{
	C_RESULT result = C_FAIL;

	switch (msg_id)
	{
		case PIPELINE_MSG_START:
			if(cfg->startRec == ACADEMY_RECORD_STOP)
            {
				time_t t;
                char date[ARDRONE_DATE_MAXSIZE];
                char video_filename[ACADEMY_MAX_FILENAME];
                char media_dirname[ACADEMY_MAX_FILENAME];

                

				ardrone_time2date(*((uint32_t*)param), ARDRONE_FILE_DATE_FORMAT, date);

                sprintf(media_dirname, "%s/media_%s", ACADEMY_FILE_DEFAULT_PATH, date);
#ifdef _WIN32
								PA_DEBUG("Assume success\n");
				
#else
				struct stat statbuf;

				if((stat(media_dirname, &statbuf) != 0) && (mkdir(media_dirname, 0777) >= 0))
					PA_DEBUG("Create local media directory %s if not exist\n", media_dirname);
#endif
				t = time(NULL);
				ardrone_time2date(t, ARDRONE_FILE_DATE_FORMAT, date);

				sprintf(cfg->video_filename, "%s/video_%s.%s",
						media_dirname,
						date,
						ACADEMY_RECORD_FILE_EXTENSION);

				strcpy(video_filename, cfg->video_filename);
				strcat(video_filename, ".");
				strcat(video_filename, ACADEMY_RECORD_FILE_EXTENSION_INPROGRESS);

				cfg->fp = fopen(video_filename, "wb");
				if (cfg->fp != NULL)
				{
					printf("Start recording %s\n", cfg->video_filename);
					cfg->startRec = ACADEMY_RECORD_START;
					result = C_OK;
				}
				else
				{
					printf ("Can't open file %s\n", video_filename);
				}
            }
			break;
  
		case PIPELINE_MSG_STOP:
			if(cfg->startRec != ACADEMY_RECORD_STOP)
            {
				cfg->startRec = ACADEMY_RECORD_STOP;
                printf("Stop recording %s\n", cfg->video_filename);
                ardrone_academy_stage_recorder_internal_close(cfg);
            }
			break;

		default:
			break;
	}

	return (VP_SUCCESS);
}

void ardrone_academy_stage_recorder_enable(bool_t enable, uint32_t timestamp)
{
	printf("Recording %s...\n", enable ? "started" : "stopped");
    ardrone_academy_stage_recorder_handle (&ardrone_academy_stage_recorder_config, (enable ? PIPELINE_MSG_START : PIPELINE_MSG_STOP), NULL, &timestamp);    
}

void ardrone_academy_stage_recorder_internal_close(ardrone_academy_stage_recorder_config_t *cfg)
{
    char video_filename[ACADEMY_MAX_FILENAME];
    strcpy(video_filename, cfg->video_filename);
    strcat(video_filename, ".");
    strcat(video_filename, ACADEMY_RECORD_FILE_EXTENSION_INPROGRESS);
    
    fclose(cfg->fp);
    cfg->fp=NULL;    
    
    rename(video_filename, cfg->video_filename);
}

C_RESULT ardrone_academy_stage_recorder_open(ardrone_academy_stage_recorder_config_t *cfg)
{
	cfg->startRec = ACADEMY_RECORD_STOP;
	return C_OK;
}

C_RESULT ardrone_academy_stage_recorder_transform(ardrone_academy_stage_recorder_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
	C_RESULT result = C_OK;
#ifndef _WIN32
	vp_os_mutex_lock( &out->lock );

	if( out->status == VP_API_STATUS_INIT )
	{
		out->numBuffers   = 1;
		out->indexBuffer  = 0;
		out->lineSize     = NULL;
	}

	out->size     = in->size;
	out->status   = in->status;
	out->buffers  = in->buffers;

	if( in->status == VP_API_STATUS_ENDED )
	{
		out->status = in->status;
	}
	else if(in->status == VP_API_STATUS_STILL_RUNNING)
    {
		out->status = VP_API_STATUS_PROCESSING;
	}
	else
	{
		out->status = in->status;
	}

	if( (cfg->fp != NULL) && (in->size > 0) && (out->status == VP_API_STATUS_PROCESSING) && (cfg->startRec == ACADEMY_RECORD_START))
	{
        fwrite(&in->size, 1, sizeof(int32_t), cfg->fp);
        fwrite(in->buffers[in->indexBuffer], 1, in->size, cfg->fp);
    }

	vp_os_mutex_unlock( &out->lock );
#endif
	return result;
}

C_RESULT ardrone_academy_stage_recorder_close(ardrone_academy_stage_recorder_config_t *cfg)
{
  if( cfg->fp != NULL )
  {
      ardrone_academy_stage_recorder_internal_close(cfg);
  }
  return C_OK;
}
