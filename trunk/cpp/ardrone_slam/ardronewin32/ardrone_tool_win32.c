/********************************************************************
 *                    COPYRIGHT PARROT 2010
 ********************************************************************
 *       PARROT - A.R.Drone SDK Windows Client Example
 *-----------------------------------------------------------------*/
/**
 * @file ardrone_tool_win32.c 
 * @brief Main program of all client programs using ARDroneTool.
 *
 * @author Stephane Piskorski <stephane.piskorski.ext@parrot.fr>
 * @date   Sept, 8. 2010
 * @warning  This file is a modified version of the 'ardrone_tool.c'
 *			 of the A.R. Drone SDK
 *
 *******************************************************************/

#pragma warning( disable : 4996 ) // disable deprecation warning 

#include <stdlib.h>

#include <ardrone_tool/ardrone_tool.h>
#include <ardrone_tool/ardrone_tool_configuration.h>
#include <ardrone_tool/Control/ardrone_control.h>
#include <ardrone_tool/Navdata/ardrone_navdata_client.h>
#include <ardrone_tool/Video/video_stage.h>
#include <ardrone_tool/UI/ardrone_input.h>
#include <VP_Com/vp_com.h>
#include <ardrone_tool/Com/config_com.h>

#include <ardrone_tool_win32.h>

#ifndef SUCCEED
#define SUCCEED VP_SUCCEEDED
#endif


extern HANDLE ardrone_ready;


// from C:\svn\ARdroneMap\ARDrone_SDK_2_0\Examples\Linux\Navigation\Sources\ihm\ihm_stages_o_gtk.h

	#define NUM_MAX_SCREEN_POINTS (DEFAULT_NB_TRACKERS_WIDTH * DEFAULT_NB_TRACKERS_HEIGHT)



typedef struct _vp_stages_draw_trackers_config_t
{
    int32_t num_points;
    uint32_t locked[NUM_MAX_SCREEN_POINTS];
    screen_point_t points[NUM_MAX_SCREEN_POINTS];

    uint32_t detected;
    //patch_ogb_type_t type[4];
    screen_point_t patch_center[4];
    uint32_t width[4];
    uint32_t height[4];

    video_decoder_config_t * last_decoded_frame_info;

} vp_stages_draw_trackers_config_t;

extern const vp_api_stage_funcs_t draw_trackers_funcs;

typedef struct _vp_stages_gtk_config_ {
    //  int max_width;
    //  int max_height;
    int rowstride;
    void * last_decoded_frame_info;
    int desired_display_width;
    int desired_display_height;
    int gdk_interpolation_mode;
} vp_stages_gtk_config_t;

extern const vp_api_stage_funcs_t vp_stages_output_gtk_funcs;

// from C:\svn\ARdroneMap\ARDrone_SDK_2_0\Examples\Linux\Navigation\Sources\ihm\ihm_stages_o_gtk.h

#ifndef SUCCESS
#define SUCCESS 0
#endif

// from C:\svn\ARdroneMap\ARDrone_SDK_2_0\Examples\Linux\Navigation\Sources\ihm\ihm_stages_o_gtk.c

static inline void reverse(uint8_t * x){
	uint8_t r=*(x);
	uint8_t g=*(x+1);
	uint8_t b=*(x+2);
	*(x)   = r+128;
	*(x+1) = g+128;
	*(x+2) = b+128;
}

void trace_reverse_rgb_h_segment(vp_api_picture_t * picture,int line,int start,int stop)
{
	int i;
	uint8_t *linepointer;
	if (line<0 || line>(int )(picture->height-1)) return;
	linepointer = &picture->raw[3*picture->width*line];
	for ( i=max(start,0);  i<(int )(picture->width-1) && i<stop ; i++ ) {
          reverse(&linepointer[3*i]);
	};
}

void trace_reverse_rgb_v_segment(vp_api_picture_t * picture,int column,int start,int stop)
{
	int i;
	uint8_t *columnpointer;
	if (column<0 || column>(int )(picture->width-1)) return;
	columnpointer = &picture->raw[3*(picture->width*start+column)];
	for ( i=max(start,0);  i<(int )(picture->height-1) && i<stop ; i++ ) {
		reverse(&columnpointer[0]);
		columnpointer+=3*picture->width;
	};
}

void trace_reverse_rgb_rectangle( vp_api_picture_t * picture,screen_point_t center, int width, int height)
{

	if (!picture) { return; }
	if (!picture->raw) { printf("NULL pointer\n");return; }
	/*if (PIX_FMT_RGB24!=picture->format) {
		printf("%s:%d - Invalid format : %d/%d\n",__FUNCTION__,__LINE__,PIX_FMT_BGR8,picture->format); return;
	};*/
	trace_reverse_rgb_h_segment(picture,center.y-height/2,center.x-width/2,center.x+width/2);
	trace_reverse_rgb_h_segment(picture,center.y+height/2,center.x-width/2,center.x+width/2);
	trace_reverse_rgb_v_segment(picture,center.x-width/2 ,center.y-height/2,center.y+height/2);
	trace_reverse_rgb_v_segment(picture,center.x+width/2 ,center.y-height/2,center.y+height/2);

	trace_reverse_rgb_h_segment(picture,center.y,center.x-width/4 ,center.x+width/4);
	trace_reverse_rgb_v_segment(picture,center.x,center.y-height/4,center.y+height/4);

}
 /***************************************************************************************************/
static vp_os_mutex_t draw_trackers_update;
/*static*/ vp_stages_draw_trackers_config_t draw_trackers_cfg = {0};

/*
void set_draw_trackers_config(vp_stages_draw_trackers_config_t* cfg) {
    void*v;
    vp_os_mutex_lock(&draw_trackers_update);
    v = draw_trackers_cfg.last_decoded_frame_info;
    vp_os_memcpy(&draw_trackers_cfg, cfg, sizeof (draw_trackers_cfg));
    draw_trackers_cfg.last_decoded_frame_info = v;
    vp_os_mutex_unlock(&draw_trackers_update);
}
*/

C_RESULT draw_trackers_stage_open(vp_stages_draw_trackers_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out) {
    vp_os_mutex_lock(&draw_trackers_update);
	{
    int32_t i;
    for (i = 0; i < NUM_MAX_SCREEN_POINTS; i++) {
        cfg->locked[i] = C_OK;
    }

    PRINT("Draw trackers inited with %d trackers\n", cfg->num_points);
	}
    vp_os_mutex_unlock(&draw_trackers_update);

    return (SUCCESS);
}

C_RESULT draw_trackers_stage_transform(vp_stages_draw_trackers_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
    int32_t i;
    video_decoder_config_t * dec_config;
    vp_api_picture_t * picture;
    uint32_t pixbuf_width;
    uint32_t pixbuf_height;

    dec_config = (video_decoder_config_t *) cfg->last_decoded_frame_info;
    pixbuf_width = dec_config->src_picture->width;
    pixbuf_height = dec_config->src_picture->height;

    vp_os_mutex_lock(&draw_trackers_update);

    picture = dec_config->dst_picture;
    picture->raw = in->buffers[in->indexBuffer];

    if (in->size > 0) {
#if defined DEBUG && 0
        for (i = 0; i < cfg->num_points; i++) {
            int32_t dist;
            uint8_t color;
            screen_point_t point;

            point = cfg->points[i];
            //       point.x += ACQ_WIDTH / 2;
            //       point.y += ACQ_HEIGHT / 2;

            if (point.x >= STREAM_WIDTH || point.x < 0 || point.y >= STREAM_HEIGHT || point.y < 0) {
                PRINT("Bad point (%d,%d) received at index %d on %d points\n", point.x, point.y, i, cfg->num_points);
                continue;
            }

            if (SUCCEED(cfg->locked[i])) {
                dist = 3;
                color = 0;
            } else {
                dist = 1;
                color = 0xFF;
            }

            vision_trace_cross(&point, dist, color, picture);
        }
#endif

        for (i = 0; i < (int )(cfg->detected); i++) {
            //uint32_t centerX,centerY;
            uint32_t width, height;
            screen_point_t center;
            if (cfg->last_decoded_frame_info != NULL) {

                center.x = cfg->patch_center[i].x * pixbuf_width / 1000;
                center.y = cfg->patch_center[i].y * pixbuf_height / 1000;
                width = cfg->width[i] * pixbuf_width / 1000;
                height = cfg->height[i] * pixbuf_height / 1000;

                width = min(2 * (uint32_t )(center.x), width);
                width = min(2 * (pixbuf_width - center.x), width) - 1;
                height = min(2 * (uint32_t )(center.y), height);
                width = min(2 * (pixbuf_height - center.y), height) - 1;


                trace_reverse_rgb_rectangle(dec_config->dst_picture,center, width, height);

            } else {
                printf("Problem drawing rectangle.\n");
            }
        }
    }

    vp_os_mutex_unlock(&draw_trackers_update);

    out->size = in->size;
    out->indexBuffer = in->indexBuffer;
    out->buffers = in->buffers;

    out->status = VP_API_STATUS_PROCESSING;

    return (SUCCESS);
}

C_RESULT draw_trackers_stage_close(vp_stages_draw_trackers_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out) {
    return (SUCCESS);
}

const vp_api_stage_funcs_t draw_trackers_funcs = {
    NULL,
    (vp_api_stage_open_t) draw_trackers_stage_open,
    (vp_api_stage_transform_t) draw_trackers_stage_transform,
    (vp_api_stage_close_t) draw_trackers_stage_close
};

/****************************************************/

C_RESULT output_gtk_stage_open(vp_stages_gtk_config_t *cfg)//, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
    return (SUCCESS);
}

C_RESULT output_gtk_stage_transform(vp_stages_gtk_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out) {
    
#ifdef NEEDED

    if (!ihm_is_initialized) return SUCCESS;
    if (ihm_ImageWin == NULL) return SUCCESS;
    if (image_vision_window_view != WINDOW_VISIBLE) return SUCCESS;


    gdk_threads_enter(); //http://library.gnome.org/devel/gdk/stable/gdk-Threads.html
    static struct timeval tvPrev = {0, 0}, tvNow = {0, 0};
    static int nbFramesForCalc = 1;
#define CALCULATE_EVERY_X_FRAMES 10
    if (0 == --nbFramesForCalc)
      {
        nbFramesForCalc = CALCULATE_EVERY_X_FRAMES;
        tvPrev.tv_sec = tvNow.tv_sec;
        tvPrev.tv_usec = tvNow.tv_usec;
        gettimeofday(&tvNow, NULL);
        if (0 != tvPrev.tv_sec) // Avoid first time calculation
          {
            float timeDiffMillis = ((tvNow.tv_sec - tvPrev.tv_sec) * 1000.0) + ((tvNow.tv_usec - tvPrev.tv_usec) / 1000.0);
            DEBUG_fps = (0.8 * DEBUG_fps) + (0.2 * ((1000.0 * CALCULATE_EVERY_X_FRAMES) / timeDiffMillis));
          }
      }

    video_decoder_config_t * dec_config;
    dec_config = (video_decoder_config_t *) cfg->last_decoded_frame_info;
    pixbuf_width = dec_config->src_picture->width;
    pixbuf_height = dec_config->src_picture->height;
    pixbuf_rowstride = dec_config->rowstride;
    pixbuf_data = (uint8_t*) in->buffers[in->indexBuffer];

    if (pixbuf != NULL) {
        g_object_unref(pixbuf);
        pixbuf = NULL;
    }

    pixbuf = gdk_pixbuf_new_from_data(pixbuf_data,
        GDK_COLORSPACE_RGB,
        FALSE,
        8,
        pixbuf_width,
        pixbuf_height,
        pixbuf_rowstride,
        NULL,
        NULL);

    if (fullscreen != NULL && fullscreen_window != NULL) {
        if (pixbuf2 != NULL) {
            g_object_unref(pixbuf2);
            pixbuf2 = NULL;
        }

        pixbuf2 = gdk_pixbuf_scale_simple(pixbuf,
            gdk_screen_get_width(fullscreen),
            gdk_screen_get_height(fullscreen),
            /*GDK_INTERP_HYPER*/
            cfg->gdk_interpolation_mode);
        /*if (fullscreen_image == NULL)
          {
          fullscreen_image  = (GtkImage*) gtk_image_new_from_pixbuf( pixbuf );
          //if (fullscreen_image == NULL) { printf("Probleme.\n"); }
          //gtk_container_add( GTK_CONTAINER( fullscreen_window ), GTK_WIDGET(fullscreen_image) );
          gtk_fixed_put(ihm_fullScreenFixedContainer,fullscreen_image,0,0);
          }*/
        if (fullscreen_image != NULL) {
            gtk_image_set_from_pixbuf(fullscreen_image, pixbuf2);
            //gtk_widget_show_all (GTK_WIDGET(fullscreen_window));
            gtk_widget_show(GTK_WIDGET(fullscreen_image));
            //gtk_widget_show(ihm_fullScreenHBox);
        }
    } else {

        if (cfg->desired_display_height != 0 && cfg->desired_display_width != 0) /* 0 and 0 means auto mode */ {
            if (pixbuf2 != NULL) {
                g_object_unref(pixbuf2);
                pixbuf2 = NULL;
            }

            pixbuf2 = gdk_pixbuf_scale_simple(pixbuf,
                cfg->desired_display_width,
                cfg->desired_display_height,
                cfg->gdk_interpolation_mode);
        } else {
            /* A copy of pixbuf is always made into pixbuf 2.
              If pixbuf is used directly, GTK renders the video from the buffer allocated by the FFMPEG decoding stage,
              which becomes invalid when the decoder is resetted (when a codec change occurs for example).
              This makes GTK crash.
              TODO : find a reliable way of rendering from the FFMPEG output buffer to avoid the data copy from pixbuf to pixbuf2
             */

            if (pixbuf2 != NULL) {
                g_object_unref(pixbuf2);
                pixbuf2 = NULL;
            }

            pixbuf2 = gdk_pixbuf_copy(pixbuf);
        }

        if (image == NULL && (pixbuf != NULL || pixbuf2 != NULL)) {
            image = (GtkImage*) gtk_image_new_from_pixbuf((pixbuf2) ? (pixbuf2) : (pixbuf));
            gtk_signal_connect(GTK_OBJECT(image), "destroy", G_CALLBACK(destroy_image_callback), NULL);
            if (GTK_IS_WIDGET(ihm_ImageWin))
                if (GTK_IS_WIDGET(ihm_VideoStream_VBox))
                    gtk_container_add(GTK_CONTAINER(ihm_VideoStream_VBox), (GtkWidget*) image);
        }
        if (image != NULL && (pixbuf != NULL || pixbuf2 != NULL)) {
            if (!videoPauseStatus) gtk_image_set_from_pixbuf(image, (pixbuf2) ? (pixbuf2) : (pixbuf));
        }
    }

	/*---- Display statistics ----*/


    float DEBUG_percentMiss = DEBUG_nbSlices * 100.0 /  DEBUG_totalSlices;


    video_information_buffer_index =
    			snprintf(video_information_buffer,
					sizeof(video_information_buffer),
					"%s - %s %dx%d\n",
					(icc.configs[icc.last_active_socket]->protocol == VP_COM_TCP)?"TCP":(icc.configs[icc.last_active_socket]->protocol == VP_COM_UDP)?"UDP":"?",
					/*codec*/
							(video_stage_decoder_lastDetectedCodec == CODEC_MPEG4_AVC )?"H.264":
							(video_stage_decoder_lastDetectedCodec == CODEC_MPEG4_VISUAL) ? "MP4":
							(video_stage_decoder_lastDetectedCodec == CODEC_VLIB) ? "VLIB":
							(video_stage_decoder_lastDetectedCodec == CODEC_P264) ? "P.264": "?",
							pixbuf_width,pixbuf_height
					);

    if (video_stage_decoder_lastDetectedCodec == CODEC_MPEG4_AVC )
    {
    	video_information_buffer_index+=
    			snprintf(video_information_buffer+video_information_buffer_index,
						sizeof(video_information_buffer)-video_information_buffer_index,
						"Mean missed slices :%6.3f/%2.0f (%5.1f%%)\nMissed frames : %10d\nFPS : %4.1f | Bitrate : %6.2f Kbps\nLatency : %5.1f ms | Protocol : %s",\
							DEBUG_nbSlices,
							DEBUG_totalSlices,
							DEBUG_percentMiss,
							DEBUG_missed,
							DEBUG_fps,
							DEBUG_bitrate,
							DEBUG_latency,
							(1 == DEBUG_isTcp) ? "TCP" : "UDP");

    }
    else
    {
    	video_information_buffer_index+=
    	    			snprintf(video_information_buffer+video_information_buffer_index,
    							sizeof(video_information_buffer)-video_information_buffer_index,
    							"Missed frames : %10d\nFPS : %4.1f | Bitrate : %6.2f Kbps\nLatency : %5.1f ms | Protocol : %s",\
    								DEBUG_missed,
    								DEBUG_fps,
    								DEBUG_bitrate,
    								DEBUG_latency,
    								(1 == DEBUG_isTcp) ? "TCP" : "UDP");

    }

    if (video_information){
		gtk_label_set_text((GtkLabel *)video_information,(const gchar*)video_information_buffer);
		gtk_label_set_justify((GtkLabel *)video_information,GTK_JUSTIFY_LEFT);
    }

    gtk_widget_show_all(ihm_ImageWin);
    gdk_threads_leave();

#endif // NEEDED

    return (SUCCESS);
}

C_RESULT output_gtk_stage_close(vp_stages_gtk_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out) {
    return (SUCCESS);
}
const vp_api_stage_funcs_t vp_stages_output_gtk_funcs = {
    NULL,
    (vp_api_stage_open_t) output_gtk_stage_open,
    (vp_api_stage_transform_t) output_gtk_stage_transform,
    (vp_api_stage_close_t) output_gtk_stage_close
};

// from C:\svn\ARdroneMap\ARDrone_SDK_2_0\Examples\Linux\Navigation\Sources\ihm\ihm_stages_o_gtk.c

/*---------------------------------------------------------------------------------------------------------------------
Tests the network connection to the drone by fetching the drone version number
through the FTP server embedded on the drone.
This is how FreeFlight checks if a drone sofware update is required.

The FTP connection process is a quick and (very)dirty one. It uses FTP passive mode.
---------------------------------------------------------------------------------------------------------------------*/
int test_drone_connection()
{
	vp_com_socket_t ftp_client;
	static Write ftp_write = NULL;
	static Read  ftp_read = NULL;
	int timeout_windows = 1000; /*milliseconds*/

	printf("test_drone_connection: size of vp_com_socket_t %d\n", sizeof(vp_com_socket_t));
	/* Connects to the FTP server */
	wifi_config_socket(&ftp_client,VP_COM_CLIENT,FTP_PORT,WIFI_ARDRONE_IP);
	ftp_client.protocol = VP_COM_TCP;
	if(VP_FAILED(vp_com_init(wifi_com())))
		return -1;
	if(VP_FAILED(vp_com_open(wifi_com(), &ftp_client, &ftp_read, &ftp_write)))
		return -2;
	setsockopt((int32_t)ftp_client.priv, 
								SOL_SOCKET, 
								SO_RCVTIMEO, 
								(const char*)&timeout_windows, sizeof(timeout_windows)
								);

	/* Clean up */
	vp_com_close(wifi_com(), &ftp_client);

	return 0;
}




/*---------------------------------------------------------------------------------------------------------------------
Main application function
---------------------------------------------------------------------------------------------------------------------*/

int ardronewin32()
{
	START_THREAD(ihm, 0);

	return 1;
}

DEFINE_THREAD_ROUTINE(ihm, data)
{
	C_RESULT res;

	WSADATA wsaData = {0};
	int iResult = 0;


	/* Initializes Windows socket subsystem */
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0)
	{
		wprintf(L"WSAStartup failed: %d\n", iResult);
		return 1;
	}

 
	/* Initializes communication sockets */	
	res = test_drone_connection(); // Nick disabled the press enter (wait)
	if(res!=0)
	{
		printf("%s","Could not detect the drone version ... press <Enter> to try connecting anyway.\n");
		getchar();
		//WSACleanup();
		exit(-1);
	}


	res = ardrone_tool_setup_com( NULL );
	if( FAILED(res) )
	{
		PRINT("Wifi initialization failed.\n");
		return -1;
	}

#if 0
	// from http://mediabox.grasp.upenn.edu/roswiki/doc/api/ardrone_brown/html/vp__api__picture_8h.html
	#define 	QVGA_WIDTH   320
	#define 	QVGA_HEIGHT   240
#endif 



	// from C:\svn\ARdroneMap\ARDrone_SDK_2_0\Examples\Linux\Navigation\Sources\common\mobile_config.h 
	#ifndef STREAM_WIDTH
  #define STREAM_WIDTH QVGA_WIDTH
 #endif
 #ifndef STREAM_HEIGHT
  #define STREAM_HEIGHT QVGA_HEIGHT
 #endif
	// from C:\svn\ARdroneMap\ARDrone_SDK_2_0\Examples\Linux\Navigation\Sources\common\mobile_main.c 
	{ /* block for definitions */

	vp_stages_latency_estimation_config_t vlat;
	vp_stages_gtk_config_t gtkconf;
	// extern video_decoder_config_t vec;
    extern vp_stages_draw_trackers_config_t draw_trackers_cfg;

	/************************ VIDEO STAGE CONFIG ******************************/
	
    #define NB_NAVIGATION_POST_STAGES   10
    uint8_t post_stages_index = 0;

    //Alloc structs
    specific_parameters_t * params             = (specific_parameters_t *)vp_os_calloc(1,sizeof(specific_parameters_t));
    specific_stages_t * navigation_pre_stages  = (specific_stages_t*)vp_os_calloc(1, sizeof(specific_stages_t));
    specific_stages_t * navigation_post_stages = (specific_stages_t*)vp_os_calloc(1, sizeof(specific_stages_t));
    vp_api_picture_t  * in_picture             = (vp_api_picture_t*) vp_os_calloc(1, sizeof(vp_api_picture_t));
    vp_api_picture_t  * out_picture            = (vp_api_picture_t*) vp_os_calloc(1, sizeof(vp_api_picture_t));

    in_picture->width          = STREAM_WIDTH;
    in_picture->height         = STREAM_HEIGHT;

    out_picture->framerate     = 20;
    out_picture->format        = PIX_FMT_RGB24;
    out_picture->width         = STREAM_WIDTH;
    out_picture->height        = STREAM_HEIGHT;

    out_picture->y_buf         = vp_os_malloc( STREAM_WIDTH * STREAM_HEIGHT * 3 );
    out_picture->cr_buf        = NULL;
    out_picture->cb_buf        = NULL;

    out_picture->y_line_size   = STREAM_WIDTH * 3;
    out_picture->cb_line_size  = 0;
    out_picture->cr_line_size  = 0;

    //Alloc the lists
    navigation_pre_stages->stages_list  = NULL;
    navigation_post_stages->stages_list = (vp_api_io_stage_t*)vp_os_calloc(NB_NAVIGATION_POST_STAGES,sizeof(vp_api_io_stage_t));

    //Fill the POST-stages------------------------------------------------------
    vp_os_memset(&vlat,         0, sizeof( vlat ));
    vlat.state = LE_DISABLED;
    //vlat.last_decoded_frame_info = (void*)&vec;
	vlat.last_decoded_frame_info = (vlib_stage_decoding_config_t *)&vec;
    //navigation_post_stages->stages_list[post_stages_index].name    = "(latency estimator)";
    navigation_post_stages->stages_list[post_stages_index].type    = VP_API_FILTER_DECODER;
    navigation_post_stages->stages_list[post_stages_index].cfg     = (void*)&vlat;
    navigation_post_stages->stages_list[post_stages_index++].funcs = vp_stages_latency_estimation_funcs;

    #ifdef RECORD_RAW_VIDEO
    vp_os_memset(&vrc,         0, sizeof( vrc ));
    #warning Recording RAW video option enabled in Navigation.
    vrc.stage = 3;
    #warning We have to get the stage number an other way
    vp_os_memset(&vrc, 0, sizeof(vrc));
    navigation_post_stages->stages_list[post_stages_index].name    = "(raw video recorder)";
    navigation_post_stages->stages_list[post_stages_index].type    = VP_API_FILTER_DECODER;
    navigation_post_stages->stages_list[post_stages_index].cfg     = (void*)&vrc;
    navigation_post_stages->stages_list[post_stages_index++].funcs   = video_recorder_funcs;
    #endif // RECORD_RAW_VIDEO


    #if defined(FFMPEG_SUPPORT) && defined(RECORD_FFMPEG_VIDEO)
    #warning Recording FFMPEG (reencoding)video option enabled in Navigation.
    vp_os_memset(&ffmpeg_vrc, 0, sizeof(ffmpeg_vrc));
    ffmpeg_vrc.numframes = &vec.controller.num_frames;
    ffmpeg_vrc.stage = pipeline.nb_stages;
    navigation_post_stages->stages_list[post_stages_index].name    = "(ffmpeg recorder)";
    navigation_post_stages->stages_list[post_stages_index].type    = VP_API_FILTER_DECODER;
    navigation_post_stages->stages_list[post_stages_index].cfg     = (void*)&ffmpeg_vrc;
    navigation_post_stages->stages_list[post_stages_index++].funcs   = video_ffmpeg_recorder_funcs;
    #endif


    vp_os_memset(&draw_trackers_cfg,         0, sizeof( draw_trackers_funcs ));
    draw_trackers_cfg.last_decoded_frame_info = (void*)&vec;
    navigation_post_stages->stages_list[post_stages_index].type    = VP_API_FILTER_DECODER;
    navigation_post_stages->stages_list[post_stages_index].cfg     = (void*)&draw_trackers_cfg;
    navigation_post_stages->stages_list[post_stages_index++].funcs   = draw_trackers_funcs;


    vp_os_memset(&gtkconf,         0, sizeof( gtkconf ));
    gtkconf.rowstride               = out_picture->width * 3;
    gtkconf.last_decoded_frame_info = (void*)&vec;
    gtkconf.desired_display_width   = 0;  /* auto */
    gtkconf.desired_display_height  = 0;  /* auto */
    gtkconf.gdk_interpolation_mode  = 0;  /* fastest */
//    navigation_post_stages->stages_list[post_stages_index].name    = "(Gtk display)";
    navigation_post_stages->stages_list[post_stages_index].type    = VP_API_OUTPUT_SDL;
    navigation_post_stages->stages_list[post_stages_index].cfg     = (void*)&gtkconf;
    navigation_post_stages->stages_list[post_stages_index++].funcs   = vp_stages_output_gtk_funcs;

    //Define the list of stages size
    navigation_pre_stages->length  = 0;
    navigation_post_stages->length = post_stages_index;

    params->in_pic = in_picture;
    params->out_pic = out_picture;
    params->pre_processing_stages_list  = navigation_pre_stages;
    params->post_processing_stages_list = navigation_post_stages;
    params->needSetPriority = 0;
    params->priority = 0;

	
	/*******End Of *********** VIDEO STAGE CONFIG ******************************/
	// from C:\svn\ARdroneMap\ARDrone_SDK_2_0\Examples\Linux\Navigation\Sources\common\mobile_main.c 

	START_THREAD(video_stage, params);

	} /* end of block for definitions */

	res = ardrone_tool_init(WIFI_ARDRONE_IP, strlen(WIFI_ARDRONE_IP), NULL, ARDRONE_CLIENT_APPNAME, ARDRONE_CLIENT_USRNAME,NULL, NULL, MAX_FLIGHT_STORING_SIZE, NULL);

	ardrone_tool_set_refresh_time(20); // 20 ms

	ardrone_at_reset_com_watchdog();


	// config
	ardrone_control_config.euler_angle_max = 0.20943951f; // 12 degrees
	ardrone_control_config.video_channel	= ZAP_CHANNEL_VERT;
	ardrone_control_config.video_codec		= UVLC_CODEC; //P264_CODEC;
	ardrone_control_config.navdata_demo		= FALSE;
	ardrone_control_config.altitude_max		= 10000;
	ardrone_control_config.control_vz_max	= 1000.0f;
	ardrone_control_config.outdoor			= FALSE;
	//ardrone_control_config.flight_without_shell = TRUE;


	ARDRONE_TOOL_CONFIGURATION_ADDEVENT(video_channel, &ardrone_control_config.video_channel, (ardrone_tool_configuration_callback) ardrone_demo_config_callback);
	//ARDRONE_TOOL_CONFIGURATION_ADDEVENT(video_channel, &ardrone_control_config.video_channel, NULL);
	//ARDRONE_TOOL_CONFIGURATION_ADDEVENT(video_codec, &ardrone_control_config.video_codec, NULL);
	ARDRONE_TOOL_CONFIGURATION_ADDEVENT (navdata_demo, &ardrone_control_config.navdata_demo, NULL);
	ARDRONE_TOOL_CONFIGURATION_ADDEVENT (altitude_max, &ardrone_control_config.altitude_max, NULL);
	ARDRONE_TOOL_CONFIGURATION_ADDEVENT (control_vz_max, &ardrone_control_config.control_vz_max, NULL);
	//ARDRONE_TOOL_CONFIGURATION_ADDEVENT (outdoor, &ardrone_control_config.outdoor, NULL);
	//ARDRONE_TOOL_CONFIGURATION_ADDEVENT (flight_without_shell, &ardrone_control_config.flight_without_shell, NULL);
	ARDRONE_TOOL_CONFIGURATION_ADDEVENT (euler_angle_max, &ardrone_control_config.euler_angle_max, NULL);


	// flat trim
	ardrone_at_set_flat_trim();


	//SetEvent(ardrone_ready);
	//ardrone_demo_redirect_to_interface = 1;


	while( VP_SUCCEEDED(res) && ardrone_tool_exit() == FALSE )
	{
		res = ardrone_tool_update();
	}

	JOIN_THREAD(video_stage);

	res = ardrone_tool_shutdown();

	WSACleanup();

	return (THREAD_RET)res;
}


void demo_video_client_process()
{
#ifdef INTERFACE_API18
	video_stage_config_t *config = video_stage_get(); //Np get anymore!
	//vlib_stage_decoding_config_t *config = video_stage_get();
	 
	if (ardrone_demo_redirect_to_interface)
		bot_ardrone_ardronelib_process_frame((unsigned char*) config->data, config->widthImage, config->heightImage);
#endif
	
}


void ardrone_demo_config_callback(unsigned int success)
{
	ardrone_tool_configuration_callback cb = (ardrone_tool_configuration_callback) ardrone_demo_config_callback;

	if (success && ++ardrone_nr_configs_suc <= ardrone_demo_nr_configs)
	{
		switch (ardrone_nr_configs_suc)
		{
		/*
		case 1:
			ARDRONE_TOOL_CONFIGURATION_ADDEVENT(video_codec, &ardrone_control_config.video_codec, cb);
			break;
		*/

		case 1:
			ARDRONE_TOOL_CONFIGURATION_ADDEVENT (navdata_demo, &ardrone_control_config.navdata_demo, cb);
			break;

		case 2:
			ARDRONE_TOOL_CONFIGURATION_ADDEVENT (altitude_max, &ardrone_control_config.altitude_max, cb);
			break;

		case 3:
			ARDRONE_TOOL_CONFIGURATION_ADDEVENT (control_vz_max, &ardrone_control_config.control_vz_max, cb);
			break;

		case 4:
			ARDRONE_TOOL_CONFIGURATION_ADDEVENT (outdoor, &ardrone_control_config.outdoor, cb);
			break;

		case 5:
			SetEvent(ardrone_ready);
			ardrone_demo_redirect_to_interface = 1;
			break;
		}
	}
}


void ardronewin32_take_off()
{
	ardrone_tool_set_ui_pad_start(1);
}


void ardronewin32_land()
{
	ardrone_tool_set_ui_pad_start(0);
}


void ardronewin32_progress(int enable, float roll, float pitch, float gaz, float yaw)
{
	ardrone_at_set_progress_cmd(enable, roll, pitch, gaz, yaw);
}


void ardronewin32_recover(int send)
{
	if (send == 1)
		ardrone_tool_set_ui_pad_select(1);
	else
		ardrone_tool_set_ui_pad_select(0);
}





inline C_RESULT demo_navdata_client_init( void* data )
{
  return C_OK;
}

inline C_RESULT demo_navdata_client_process( const navdata_unpacked_t* const navdata )
{
    navdata_unpacked_t *nd = (navdata_unpacked_t*)navdata;

	if (ardrone_demo_redirect_to_interface)
		bot_ardrone_ardronelib_process_navdata(nd);	

	return C_OK;
}

inline C_RESULT demo_navdata_client_release( void )
{
  return C_OK;
}



/* Implementing thread table in which you add routines of your application and those provided by the SDK */
BEGIN_THREAD_TABLE
  THREAD_TABLE_ENTRY( ihm, 20 )
  THREAD_TABLE_ENTRY( ardrone_control, 20 )
  THREAD_TABLE_ENTRY( navdata_update, 20 )
  THREAD_TABLE_ENTRY( video_stage, 20 )
END_THREAD_TABLE


/* 
Registering the navdata handling function to 'navdata client' which is part 
of the ARDroneTool.
You can add as many navdata handlers as you want.
Terminate the table with a NULL pointer.
*/
BEGIN_NAVDATA_HANDLER_TABLE
  NAVDATA_HANDLER_TABLE_ENTRY(demo_navdata_client_init, demo_navdata_client_process, demo_navdata_client_release, NULL)
END_NAVDATA_HANDLER_TABLE