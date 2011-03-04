/*
 *  ardrone_tool_configuration.h
 *  
 *
 *  Created by Frederic D'HAEYER on 13/09/10.
 *  Copyright 2010 Parrot SA. All rights reserved.
 *
 */
#ifndef _ARDRONE_TOOL_CONFIGURATION_H_
#define _ARDRONE_TOOL_CONFIGURATION_H_
#include <ardrone_api.h>
#include <VP_Os/vp_os_types.h>
#include <VP_Os/vp_os_signal.h>
#include <VP_Api/vp_api_thread_helper.h>

#include <ardrone_tool/Control/ardrone_control_ack.h>
#include <ardrone_tool/Control/ardrone_control_configuration.h>

#include <iniparser3.0b/src/iniparser.h>

#undef ARDRONE_CONFIG_KEY_IMM
#undef ARDRONE_CONFIG_KEY_REF
#undef ARDRONE_CONFIG_KEY_STR
#define ARDRONE_CONFIG_KEY_IMM(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK)
#define ARDRONE_CONFIG_KEY_REF(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK)
#define ARDRONE_CONFIG_KEY_STR(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK)
#include <config_keys.h>

typedef void (*ardrone_tool_configuration_callback)(bool_t result);

typedef struct _ardrone_tool_configuration_data_t {
	char* key;
	ardrone_configuration_value_t value;
	ardrone_at_configuration_set callback;
	ARDRONE_CONTROL_MODE control_mode;
	ardrone_tool_configuration_callback result_callback;
	ardrone_control_event_t* event;
	char        string_value[256];
	char        string_param[256];
} ardrone_tool_configuration_data_t;

#define ARDRONE_TOOL_CONFIGURATION_SET(NAME, VALUE) 				ARDRONE_TOOL_CONFIGURATION_ADDEVENT(NAME,VALUE,NULL)
#define ARDRONE_TOOL_CONFIGURATION_SET_WITH_CALLBACK(NAME, VALUE,CALLBACK) ARDRONE_TOOL_CONFIGURATION_ADDEVENT(NAME,VALUE,CALLBACK)

#define ARDRONE_TOOL_CONFIGURATION_ADDEVENT(NAME, VALUE, CALLBACK)	ardrone_tool_configuration_addevent_##NAME((ardrone_configuration_value_t)VALUE, CALLBACK)
#define ARDRONE_TOOL_CONFIGURATION_GET(CALLBACK)					ardrone_tool_configuration_get(CALLBACK)

#undef ARDRONE_CONFIG_KEY_IMM
#undef ARDRONE_CONFIG_KEY_REF
#undef ARDRONE_CONFIG_KEY_STR
#define ARDRONE_CONFIG_KEY_IMM(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK) \
bool_t ardrone_tool_configuration_addevent_##NAME(ardrone_configuration_value_t value, ardrone_tool_configuration_callback result_callback);
#define ARDRONE_CONFIG_KEY_REF(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK)
#define ARDRONE_CONFIG_KEY_STR(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK) \
bool_t ardrone_tool_configuration_addevent_##NAME(ardrone_configuration_value_t value, ardrone_tool_configuration_callback result_callback);
#include <config_keys.h>

extern ardrone_config_t 	ardrone_control_config;
extern const ardrone_config_t 	ardrone_control_config_default;

void ardrone_tool_configuration_init(void);
bool_t ardrone_tool_configuration_get(ardrone_tool_configuration_callback result_callback);

PROTO_THREAD_ROUTINE(ardrone_configuration, data);

#endif // _ARDRONE_TOOL_CONFIGURATION_H_
