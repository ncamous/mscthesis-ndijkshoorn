//
//  ardrone_tool_configuration.c
//
//  Created by D'HAEYER Frederic on 13/09/10.
//  Copyright 20010 Parrot SA. All rights reserved.
//
#include <ardrone_tool/ardrone_tool_configuration.h>
#include <config.h>

#define ARDRONE_TOOL_CONFIGURATION_MAX_EVENT	128

#undef ARDRONE_CONFIG_KEY_IMM
#undef ARDRONE_CONFIG_KEY_REF
#undef ARDRONE_CONFIG_KEY_STR
#define ARDRONE_CONFIG_KEY_IMM(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK) DEFAULT,
#define ARDRONE_CONFIG_KEY_REF(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK) DEFAULT,
#define ARDRONE_CONFIG_KEY_STR(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK) DEFAULT,
const ardrone_config_t ardrone_control_config_default =
{
#include <config_keys.h>
};

ardrone_config_t ardrone_control_config =
{
#include <config_keys.h>
};


static ardrone_tool_configuration_data_t ardrone_tool_configuration_data[ARDRONE_TOOL_CONFIGURATION_MAX_EVENT];
static dictionary *ardrone_tool_configuration_dict;
static bool_t ardrone_tool_configuration_is_init = FALSE;
static int ardrone_tool_configuration_current_index = 0;
static int ardrone_tool_configuration_nb_event = 0;
static vp_os_mutex_t ardrone_tool_configuration_mutex;

static void ardrone_tool_configuration_event_configure(void);

#undef ARDRONE_CONFIG_KEY_IMM
#undef ARDRONE_CONFIG_KEY_REF
#undef ARDRONE_CONFIG_KEY_STR
#define ARDRONE_CONFIG_KEY_IMM(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK)										\
bool_t ardrone_tool_configuration_addevent_##NAME(ardrone_configuration_value_t value, ardrone_tool_configuration_callback result_callback)	\
{																																	\
	bool_t res = FALSE;																												\
	if(((RW) & K_WRITE) == K_WRITE)																									\
	{																																\
		vp_os_mutex_lock(&ardrone_tool_configuration_mutex);																		\
		if(ardrone_tool_configuration_current_index == (ardrone_tool_configuration_nb_event + 1) % ARDRONE_TOOL_CONFIGURATION_MAX_EVENT)	\
		{																																\
			printf("ARDRONE_TOOL_CONFIGURATION QUEUE FILLED !!\n");																		\
		}																																\
		else																															\
		{																																\
			ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event].control_mode = ACK_CONTROL_MODE;						\
			ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event].result_callback = result_callback;						\
			ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event].key = #NAME;											\
			if(strcmp(#INI_TYPE, "INI_FLOAT") == 0)																						\
				ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event].value.f_value = value.f_value;						\
			else if(strcmp(#INI_TYPE, "INI_INT") == 0)																					\
				ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event].value.i_value = value.i_value;						\
			else if(strcmp(#INI_TYPE, "INI_BOOLEAN") == 0)																				\
				ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event].value.b_value = value.b_value;						\
			else																														\
				ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event].value.value = value.value;							\
			ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event].callback	= (ardrone_at_configuration_set)&ARDRONE_CONFIGURATION_SET_FUNCTION(NAME);	\
			ardrone_tool_configuration_nb_event = (ardrone_tool_configuration_nb_event + 1) % ARDRONE_TOOL_CONFIGURATION_MAX_EVENT;		\
			if(ardrone_tool_configuration_nb_event == ((ardrone_tool_configuration_current_index + 1) % ARDRONE_TOOL_CONFIGURATION_MAX_EVENT))	\
				ardrone_tool_configuration_event_configure();																				\
			res = TRUE;																													\
		}																																\
		vp_os_mutex_unlock(&ardrone_tool_configuration_mutex);																		\
	}																																	\
	return res;																															\
}

#define ARDRONE_CONFIG_KEY_REF(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK)
#define ARDRONE_CONFIG_KEY_STR(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK)										\
bool_t ardrone_tool_configuration_addevent_##NAME(ardrone_configuration_value_t value, ardrone_tool_configuration_callback result_callback)	\
{																																	\
	ardrone_tool_configuration_data_t *atcd;																							\
	bool_t res = FALSE;																												\
	if((value.c_value != NULL) && (((RW) & K_WRITE) == K_WRITE))																	\
	{																																\
		vp_os_mutex_lock(&ardrone_tool_configuration_mutex);																		\
		if(ardrone_tool_configuration_current_index == (ardrone_tool_configuration_nb_event + 1) % ARDRONE_TOOL_CONFIGURATION_MAX_EVENT) \
		{ 																																\
			printf("ARDRONE_TOOL_CONFIGURATION QUEUE FILLED !!\n"); 																	\
		}																																\
		else																															\
		{																																\
			atcd = &ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event];												\
			if ((strlen(value.c_value)+2)>=sizeof(atcd->string_value)) 																	\
				{ printf("Major error in %s:l.%i : buffer too small.",__FUNCTION__,__LINE__); return FALSE; }							\
			vp_os_memcpy(atcd->string_value,																							\
						 value.c_value,																									\
						 strlen(value.c_value)+1);																						\
			atcd->control_mode = ACK_CONTROL_MODE;																						\
			atcd->result_callback = result_callback;																					\
			atcd->key = #NAME;																											\
			atcd->value.c_value = atcd->string_value;																					\
			atcd->callback	= (ardrone_at_configuration_set)&ARDRONE_CONFIGURATION_SET_FUNCTION(NAME);									\
			ardrone_tool_configuration_nb_event = (ardrone_tool_configuration_nb_event + 1) % ARDRONE_TOOL_CONFIGURATION_MAX_EVENT;		\
			\
			if(ardrone_tool_configuration_nb_event == ((ardrone_tool_configuration_current_index + 1) % ARDRONE_TOOL_CONFIGURATION_MAX_EVENT))	\
				ardrone_tool_configuration_event_configure();																				\
			res = TRUE;																													\
		}																																\
		vp_os_mutex_unlock(&ardrone_tool_configuration_mutex);																		\
	}																																	\
	return res;																															\
}
#include <config_keys.h>


bool_t ardrone_tool_configuration_get(ardrone_tool_configuration_callback result_callback)
{
	bool_t res = FALSE;
	vp_os_mutex_lock(&ardrone_tool_configuration_mutex);
	if(ardrone_tool_configuration_current_index == (ardrone_tool_configuration_nb_event + 1) % ARDRONE_TOOL_CONFIGURATION_MAX_EVENT)
	{
		printf("ARDRONE_TOOL_CONFIGURATION QUEUE FILLED !!\n");
	}
	else 
	{
		ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event].key = NULL;
		ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event].callback = NULL;
		
		ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event].control_mode = CFG_GET_CONTROL_MODE;
		ardrone_tool_configuration_data[ardrone_tool_configuration_nb_event].result_callback = result_callback;
		ardrone_tool_configuration_nb_event = (ardrone_tool_configuration_nb_event + 1) % ARDRONE_TOOL_CONFIGURATION_MAX_EVENT;
		if(ardrone_tool_configuration_nb_event == ((ardrone_tool_configuration_current_index + 1) % ARDRONE_TOOL_CONFIGURATION_MAX_EVENT))
			ardrone_tool_configuration_event_configure();
		res = TRUE;
	}
	vp_os_mutex_unlock(&ardrone_tool_configuration_mutex);

	return res;
}

void ardrone_tool_configuration_init(void)
{
	if(!ardrone_tool_configuration_is_init)
	{
		ardrone_tool_configuration_dict = dictionary_new(0);

#undef ARDRONE_CONFIG_KEY_IMM
#undef ARDRONE_CONFIG_KEY_REF
#undef ARDRONE_CONFIG_KEY_STR
#define ARDRONE_CONFIG_KEY_IMM(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK) \
iniparser_alias(ardrone_tool_configuration_dict, KEY ":" #NAME, INI_TYPE, &ardrone_control_config.NAME, NULL,RW);
#define ARDRONE_CONFIG_KEY_REF(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK) \
iniparser_alias(ardrone_tool_configuration_dict, KEY ":" #NAME, INI_TYPE, &ardrone_control_config.NAME, NULL,RW);
#define ARDRONE_CONFIG_KEY_STR(KEY, NAME, INI_TYPE, C_TYPE, C_TYPE_PTR, RW, DEFAULT, CALLBACK) \
iniparser_alias(ardrone_tool_configuration_dict, KEY ":" #NAME, INI_TYPE, &ardrone_control_config.NAME[0], NULL,RW);
#include <config_keys.h>

		ardrone_tool_configuration_current_index = 0;
		ardrone_tool_configuration_nb_event = 0;
		vp_os_memset(&ardrone_tool_configuration_data[0], 0, sizeof(ardrone_tool_configuration_data_t) * ARDRONE_TOOL_CONFIGURATION_MAX_EVENT);
		vp_os_mutex_init(&ardrone_tool_configuration_mutex);
		ardrone_tool_configuration_is_init = TRUE;
	}
}

static void ardrone_tool_configuration_event_configure_end(struct _ardrone_control_event_t* event)
{
	vp_os_mutex_lock(&ardrone_tool_configuration_mutex);
	switch(event->status)
	{
		case ARDRONE_CONTROL_EVENT_FINISH_SUCCESS:
			if(ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].result_callback != NULL)
				ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].result_callback(TRUE);

			vp_os_free(ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event);
			ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event = NULL;
			ardrone_tool_configuration_current_index = (ardrone_tool_configuration_current_index + 1) % ARDRONE_TOOL_CONFIGURATION_MAX_EVENT;
			break;
			
		case ARDRONE_CONTROL_EVENT_FINISH_FAILURE:
			if(ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].result_callback != NULL)
				ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].result_callback(FALSE);
			break;
			
		default:
			// Nothing to do
			break;
	}

	if(ardrone_tool_configuration_current_index != ardrone_tool_configuration_nb_event)
		ardrone_tool_configuration_event_configure();

	vp_os_mutex_unlock(&ardrone_tool_configuration_mutex);
}

static void ardrone_tool_configuration_event_configure(void)
{
	bool_t control_mode_ok = FALSE;
	switch(ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].control_mode)
	{
		case ACK_CONTROL_MODE:
		{
			ardrone_control_ack_event_t *event = NULL;

			if (ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].callback)
			ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].callback(ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].value);

			if(ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event == NULL)
				ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event = vp_os_malloc(sizeof(ardrone_control_ack_event_t));

			event = (ardrone_control_ack_event_t*)ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event;
			event->ack_state = ACK_COMMAND_MASK_TRUE;

			control_mode_ok = TRUE;
		}
		break;

		case CFG_GET_CONTROL_MODE:
		{
			ardrone_control_configuration_event_t *event = NULL;

			if(ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event == NULL)
				ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event = vp_os_malloc(sizeof(ardrone_control_configuration_event_t));

			event = (ardrone_control_configuration_event_t*)ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event;
			event->config_state = CONFIG_REQUEST_INI;
			event->ini_dict = ardrone_tool_configuration_dict;

			control_mode_ok = TRUE;
		}
		break;

		default:
			break;
	}

	if(control_mode_ok)
	{
		ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event->event = ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].control_mode;
		ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event->status	= ARDRONE_CONTROL_EVENT_WAITING;
		ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event->num_retries	= 10;
		ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event->ardrone_control_event_start = NULL;
		ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event->ardrone_control_event_end	= ardrone_tool_configuration_event_configure_end;

		ardrone_control_send_event( ardrone_tool_configuration_data[ardrone_tool_configuration_current_index].event );
	}
}
