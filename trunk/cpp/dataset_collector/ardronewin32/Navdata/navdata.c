/********************************************************************
 *                    COPYRIGHT PARROT 2010
 ********************************************************************
 *       PARROT - A.R.Drone SDK Windows Client Example
 *-----------------------------------------------------------------*/
/**
 * @file navdata.c 
 * @brief Navdata handling code
 *
 * @author sylvain.gaeremynck@parrot.com
 * @date 2009/07/01
 *
 * @author Stephane Piskorski <stephane.piskorski.ext@parrot.fr>
 * @date   Sept, 8. 2010
 *
 *******************************************************************/


/* Includes required to handle navigation data */
	#include <ardrone_tool/Navdata/ardrone_navdata_client.h>
	#include <Navdata/navdata.h>

#include <custom_code.h>


/*---------------------------------------------------------------------------------------------------------------------
Function taking the drone control state stored as an integer, and prints in a string 
the names of the set control bits.

The drone control state is an integer sent in the navdata which says if the drone is landed, flying,
hovering, taking-off, crashed, etc ...
---------------------------------------------------------------------------------------------------------------------*/

	#define CTRL_STATES_STRING
	#include "control_states.h"

const char* ctrl_state_str(uint32_t ctrl_state)
{
  #define MAX_STR_CTRL_STATE 256
  static char str_ctrl_state[MAX_STR_CTRL_STATE];

  ctrl_string_t* ctrl_string;
  uint32_t major = ctrl_state >> 16;
  uint32_t minor = ctrl_state & 0xFFFF;

  if( strlen(ctrl_states[major]) < MAX_STR_CTRL_STATE )
  {
    vp_os_memset(str_ctrl_state, 0, sizeof(str_ctrl_state));

    strcat_s(str_ctrl_state, sizeof(str_ctrl_state),ctrl_states[major]);
    ctrl_string = control_states_link[major];

    if( ctrl_string != NULL && (strlen(ctrl_states[major]) + strlen(ctrl_string[minor]) < MAX_STR_CTRL_STATE) )
    {
      strcat_s( str_ctrl_state,sizeof(str_ctrl_state), " | " );
      strcat_s( str_ctrl_state, sizeof(str_ctrl_state),ctrl_string[minor] );
    }
  }
  else
  {
    vp_os_memset( str_ctrl_state, '#', sizeof(str_ctrl_state) );
  }

  return str_ctrl_state;
}



/*---------------------------------------------------------------------------------------------------------------------
Initialization local variables before event loop  
---------------------------------------------------------------------------------------------------------------------*/
inline C_RESULT demo_navdata_client_init( void* data )
{
	/**	======= INSERT USER CODE HERE ========== **/
	/* Initialize your navdata handler */
	/**	======= INSERT USER CODE HERE ========== **/

  return C_OK;
}





/*---------------------------------------------------------------------------------------------------------------------
Navdata handling function, which is called every time navdata are received
---------------------------------------------------------------------------------------------------------------------*/
inline C_RESULT demo_navdata_client_process( const navdata_unpacked_t* const navdata )
{
	static int cpt=0;

    navdata_unpacked_t *nd = (navdata_unpacked_t*)navdata;

	/*
	FILE *file_out;
	char *filename = "gyros.csv";
	fopen_s (&file_out, filename , "a");

	fprintf(file_out, "%f,%f,%f,%f,%f,%f\n",
		navdata->navdata_phys_measures.phys_gyros[1],
		navdata->navdata_phys_measures.phys_gyros[0],
		navdata->navdata_phys_measures.phys_gyros[2],

		navdata->navdata_demo.vx,
		navdata->navdata_demo.vy,
		navdata->navdata_altitude.altitude_vz
	);

	fclose(file_out)
		*/


	/** ======= INSERT USER CODE HERE ========== **/
	bot_ardrone_ardronelib_process_navdata(nd);	

	return C_OK;
}






/*---------------------------------------------------------------------------------------------------------------------
 Relinquish the local resources after the event loop exit 
---------------------------------------------------------------------------------------------------------------------*/
inline C_RESULT demo_navdata_client_release( void )
{
	/**	======= INSERT USER CODE HERE ========== **/
	/* Clean up your navdata handler */
	/**	======= INSERT USER CODE HERE ========== **/
  return C_OK;
}





/* 
Registering the navdata handling function to 'navdata client' which is part 
of the ARDroneTool.
You can add as many navdata handlers as you want.
Terminate the table with a NULL pointer.
*/
BEGIN_NAVDATA_HANDLER_TABLE
  NAVDATA_HANDLER_TABLE_ENTRY(demo_navdata_client_init, demo_navdata_client_process, demo_navdata_client_release, NULL)
END_NAVDATA_HANDLER_TABLE

