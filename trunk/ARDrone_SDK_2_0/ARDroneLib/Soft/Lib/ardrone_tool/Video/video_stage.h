/*
 *  video_stage.h
 *  Test
 *
 *  Created by Frederic D'HAEYER on 22/02/10.
 *  Copyright 2010 Parrot SA. All rights reserved.
 *
 */
#ifndef _VIDEO_STAGE_H_
#define _VIDEO_STAGE_H_

#include <VP_Os/vp_os.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_types.h>
#include <VP_Os/vp_os_signal.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_delay.h>

#include <VP_Api/vp_api.h>
#include <VP_Api/vp_api_error.h>
#include <VP_Api/vp_api_stage.h>
#include <VP_Api/vp_api_picture.h>
#include <VP_Api/vp_api_thread_helper.h>

#ifdef _WIN32
#include <VLIB/Stages/vlib_stage_decode.h>

# define mp4h264_config_t vlib_stage_decoding_config_t // this is the hack

#if NEEDED
typedef struct _video_decoder_config_t
{
  // Input data : dst_picture->format
  // Output : all others
  vp_api_picture_t *src_picture;
  vp_api_picture_t *dst_picture;
  uint32_t num_frames;
  uint32_t num_picture_decoded;
  uint32_t rowstride;
  uint32_t bpp;

  // Internal datas
  bool_t vlibMustChangeFormat;
  vlib_stage_decoding_config_t *vlibConf;
  vp_api_io_data_t *vlibOut;
  mp4h264_config_t *mp4h264Conf;
  vp_api_io_data_t *mp4h264Out;
} video_decoder_config_t;

C_RESULT video_stage_decoder_open (video_decoder_config_t *cfg);
C_RESULT video_stage_decoder_transform (video_decoder_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);
C_RESULT video_stage_decoder_close (video_decoder_config_t *cfg);

extern const vp_api_stage_funcs_t video_decoding_funcs;
#else
#include <ardrone_tool/Video/video_stage_decoder.h>
#endif

#endif

#include <ardrone_tool/Video/video_com_stage.h>
#include <ardrone_tool/Video/video_stage_tcp.h>
#include <ardrone_tool/Video/video_stage_decoder.h>
#include <ardrone_tool/Video/video_stage_merge_slices.h>
#include <ardrone_tool/Video/video_stage_latency_estimation.h>

// From ControlEngine/Iphone
//#include "ConstantsAndMacros.h"
#ifdef _WIN32
#include <ardrone_api.h>
#include <control_states.h>
#include <ardrone_tool/ardrone_version.h>
#include <ardrone_tool/ardrone_tool.h>
#include <ardrone_tool/ardrone_tool_configuration.h>
#include <ardrone_tool/Academy/academy.h>
#include <ardrone_tool/Academy/academy_download.h>
#include <ardrone_tool/Control/ardrone_control.h>
#include <ardrone_tool/Control/ardrone_control_ack.h>
#include <ardrone_tool/Control/ardrone_control_configuration.h>
#include <ardrone_tool/Navdata/ardrone_navdata_client.h>
#include <ardrone_tool/UI/ardrone_input.h>
#include <ardrone_tool/Com/config_com.h>
#include <ardrone_tool/Video/video_stage.h>
#include <ardrone_tool/Video/video_stage_latency_estimation.h>
#include <ardrone_tool/Video/video_recorder_pipeline.h>
#include <ardrone_tool/Video/video_navdata_handler.h>

#include <utils/ardrone_time.h>
#include <utils/ardrone_date.h>

#include <Maths/time.h>

#include <VP_Os/vp_os.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_types.h>
#include <VP_Os/vp_os_signal.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_delay.h>

#include <VP_Api/vp_api.h>
#include <VP_Api/vp_api_error.h>
#include <VP_Api/vp_api_stage.h>
#include <VP_Api/vp_api_picture.h>
#include <VP_Api/vp_api_thread_helper.h>

#include <VLIB/Stages/vlib_stage_decode.h>

#include <VLIB/video_codec.h>

#include <iniparser3.0b/src/iniparser.h>

#include <time.h>

#endif

#ifdef __OBJC__
#import <Foundation/Foundation.h>
#import <AudioToolbox/AudioToolbox.h>
#import <AVFoundation/AVFoundation.h>
#import <MediaPlayer/MediaPlayer.h>
#import <UIKit/UIKit.h>
#import <OpenGLES/EAGL.h>
#import <CoreMotion/CoreMotion.h>
#import <CoreLocation/CoreLocation.h>
#import <SystemConfiguration/SystemConfiguration.h>
#import "quicktime_encoder_stage.h"
#import <QuartzCore/QuartzCore.h>
#endif

typedef struct _specific_stages_t_
{
    vp_api_io_stage_t * stages_list;
    uint8_t length;
} specific_stages_t;

typedef struct _specific_parameters_t_
{
    specific_stages_t * pre_processing_stages_list;
    specific_stages_t * post_processing_stages_list;
    vp_api_picture_t * in_pic;
    vp_api_picture_t * out_pic;
    int needSetPriority;
    int priority;
} specific_parameters_t;

extern video_decoder_config_t vec;

PROTO_THREAD_ROUTINE(video_stage, data);

static inline unsigned long RoundPower2(unsigned long x)
{
	unsigned long rval=512;
	// rval<<=1 Is A Prettier Way Of Writing rval*=2; 
	while(rval < x)
		rval<<=1;
	return rval;
}

void video_stage_init(void);
void video_stage_suspend_thread(void);
void video_stage_resume_thread(void);
uint32_t video_stage_get_num_retries(void);


#endif // _VIDEO_STAGE_H_
