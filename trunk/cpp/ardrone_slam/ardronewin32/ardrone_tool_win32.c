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



extern HANDLE ardrone_ready;



/*---------------------------------------------------------------------------------------------------------------------
Tests the network connection to the drone by fetching the drone version number
through the FTP server embedded on the drone.
This is how FreeFlight checks if a drone sofware update is required.

The FTP connection process is a quick and (very)dirty one. It uses FTP passive mode.
---------------------------------------------------------------------------------------------------------------------*/
int test_drone_connection()
{
	const char * passivdeModeHeader = "\r\n227 PASV ok (";
	vp_com_socket_t ftp_client,ftp_client2;
	char buffer[1024];
	static Write ftp_write = NULL;
	static Read  ftp_read = NULL;
	int bytes_to_send,received_bytes;
	int i,L,x[6],port;
	int timeout_windows = 1000; /*milliseconds*/
	
	vp_os_memset(buffer,0,sizeof(buffer));

	/* Connects to the FTP server */
		wifi_config_socket(&ftp_client,VP_COM_CLIENT,FTP_PORT,WIFI_ARDRONE_IP);
		ftp_client.protocol = VP_COM_TCP;
		if(VP_FAILED(vp_com_init(wifi_com()))) return -1;
		if(VP_FAILED(vp_com_open(wifi_com(), &ftp_client, &ftp_read, &ftp_write))) return -2;
		setsockopt((int32_t)ftp_client.priv, 
								SOL_SOCKET, 
								SO_RCVTIMEO, 
								(const char*)&timeout_windows, sizeof(timeout_windows)
								); 

	/* Request version file */
		bytes_to_send = _snprintf(buffer,sizeof(buffer),"%s",
			"USER anonymous\r\nCWD /\r\nPWD\r\nTYPE A\r\nPASV\r\nRETR version.txt\r\n");
		ftp_write(&ftp_client,buffer,&bytes_to_send);
		/* Dirty. We should wait for data to arrive with some kind of synchronization
		or make the socket blocking.*/
		Sleep(1000);

	/* Gets the data port */
		received_bytes = sizeof(buffer);
		ftp_read(&ftp_client,buffer,&received_bytes);
		if (received_bytes<1) { vp_com_close(wifi_com(), &ftp_client); return -3; }
		L=received_bytes-strlen(passivdeModeHeader);

	/* Searches for the passive mode acknowlegment from the FTP server */
		for (i=0;i<L;i++) {
			if (strncmp((buffer+i),passivdeModeHeader,strlen(passivdeModeHeader))==0)  break; 
		}
		if (i==L) {
			vp_com_close(wifi_com(), &ftp_client); return -4; 
		}
		i+=strlen(passivdeModeHeader);
		if (sscanf(buffer+i,"%i,%i,%i,%i,%i,%i)",&x[0],&x[1],&x[2],&x[3],&x[4],&x[5])!=6)
			{ vp_com_close(wifi_com(), &ftp_client); return -5; }
		port=(x[4]<<8)+x[5];

	/* Connects to the FTP server data port */
		wifi_config_socket(&ftp_client2,VP_COM_CLIENT,port,"192.168.1.1");
		ftp_client2.protocol = VP_COM_TCP;
		if(VP_FAILED(vp_com_init(wifi_com()))) 
				{ vp_com_close(wifi_com(), &ftp_client2); return -6; }
		if(VP_FAILED(vp_com_open(wifi_com(), &ftp_client2, &ftp_read, &ftp_write)))
			{ vp_com_close(wifi_com(), &ftp_client2); return -7; }

	/* Gets the data */
		received_bytes = sizeof(buffer);
		ftp_read(&ftp_client2,buffer,&received_bytes);
		if (received_bytes>0) {
			buffer[min(received_bytes,sizeof(buffer)-1)]=0;
			//printf("Drone version %s detected ... press <Enter> to start the application.\n",buffer);
			//getchar();
			printf("Drone version %s detected\n",buffer);
		}
	
	/* Clean up */
		vp_com_close(wifi_com(), &ftp_client);
		vp_com_close(wifi_com(), &ftp_client2);

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

		
	START_THREAD(video_stage, 0);

	res = ardrone_tool_init(WIFI_ARDRONE_IP, strlen(WIFI_ARDRONE_IP), NULL, ARDRONE_CLIENT_APPNAME, ARDRONE_CLIENT_USRNAME);

	//ardrone_tool_set_refresh_time(20); // 20 ms

	ardrone_at_reset_com_watchdog();


	// config
	ardrone_control_config.video_channel	= ZAP_CHANNEL_VERT;
	ardrone_control_config.video_codec		= UVLC_CODEC; //P264_CODEC;
	ardrone_control_config.navdata_demo		= FALSE;
	ardrone_control_config.altitude_max		= 10000;
	ardrone_control_config.control_vz_max	= 1000.0f;
	ardrone_control_config.outdoor			= FALSE;

	ARDRONE_TOOL_CONFIGURATION_ADDEVENT(video_channel, &ardrone_control_config.video_channel, NULL);
	ARDRONE_TOOL_CONFIGURATION_ADDEVENT(video_codec, &ardrone_control_config.video_codec, NULL);
	ARDRONE_TOOL_CONFIGURATION_ADDEVENT (navdata_demo, &ardrone_control_config.navdata_demo, NULL);
	ARDRONE_TOOL_CONFIGURATION_ADDEVENT (altitude_max, &ardrone_control_config.altitude_max, NULL);
	ARDRONE_TOOL_CONFIGURATION_ADDEVENT (control_vz_max, &ardrone_control_config.control_vz_max, NULL);
	ARDRONE_TOOL_CONFIGURATION_ADDEVENT (outdoor, &ardrone_control_config.outdoor, NULL);


	// flat trim
	ardrone_at_set_flat_trim();

	SetEvent(ardrone_ready);

	while( VP_SUCCEEDED(res) && ardrone_tool_exit() == FALSE )
	{
		res = ardrone_tool_update();
	}

	JOIN_THREAD(video_stage);

	res = ardrone_tool_shutdown();

	WSACleanup();

	//return VP_SUCCEEDED(res) ? 0 : -1;
	return (THREAD_RET)res;
}


void demo_video_client_process()
{
	video_stage_config_t *config = video_stage_get();

	bot_ardrone_ardronelib_process_frame((unsigned char*) config->data, config->widthImage, config->heightImage);
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