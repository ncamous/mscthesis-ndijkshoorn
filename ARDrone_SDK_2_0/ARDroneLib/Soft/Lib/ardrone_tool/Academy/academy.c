/**
 *  academy.c
 *
 *  Created by Frederic D'HAEYER on 8/04/11.
 *  Copyright 2011 Parrot SA. All rights reserved.
 *
 */
#include <ardrone_tool/Academy/academy.h>
#include <ardrone_tool/Academy/academy_download.h>
#include <ardrone_tool/Academy/academy_upload.h>
#include <utils/ardrone_date.h>
#include <wchar.h>
//#include <ftw.h>

#ifndef	_FTW_H
// from cygwin /usr/include/ftw.h

#define	FTW_F		0	/* File.  */
#define	FTW_D		1	/* Directory.  */
#define	FTW_DNR		2	/* Directory without read permission.  */
#define	FTW_DP		3	/* Directory with subdirectories visited.  */
#define	FTW_NS		4	/* Unknown type; stat() failed.  */
#define	FTW_SL		5	/* Symbolic link.  */
#define	FTW_SLN		6	/* Sym link that names a nonexistent file.  */

int	ftw(const char *, int (*)(const char *, const struct stat *, int), int);

#endif
PROTO_THREAD_ROUTINE(academy, data);

char flight_dir[ROOT_NAME_SIZE] = ".";

static THREAD_HANDLE academy_thread;
static bool_t academy_started = FALSE;

typedef struct _academy_t_
{
	vp_os_mutex_t mutex;
	vp_os_cond_t cond;
	int16_t  flight_max_storing_size;
	uint32_t flight_sum;
	char flight_oldest_date[ARDRONE_DATE_MAXSIZE];
} academy_t;

static academy_t academy;

void academy_init(const char *flightdir, int max_storing_size)
{
	if(!academy_started)
	{
        PA_WARNING("MEMORY SPACE ALLOWED : %d MB\n", max_storing_size);
		vp_os_memset(&academy, 0, sizeof(academy_t));
		vp_os_mutex_init(&academy.mutex);
		vp_os_cond_init(&academy.cond, &academy.mutex);
		academy.flight_max_storing_size = max_storing_size;

        // copy root directory
		if((flightdir != NULL) && strlen(flightdir) < ROOT_NAME_SIZE)
			strcpy(flight_dir, flightdir);

		academy_started = TRUE;
		vp_os_thread_create(thread_academy, (THREAD_PARAMS)NULL, &academy_thread);
	}

	academy_upload_init();
}

void academy_shutdown(void)
{
	if(academy_started)
	{
		academy_started = FALSE;
		vp_os_mutex_lock(&academy.mutex);
		vp_os_cond_signal(&academy.cond);
		vp_os_mutex_unlock(&academy.mutex);
		vp_os_thread_join(academy_thread);
	}

	academy_upload_shutdown();
}

void academy_check_memory(void)
{
	vp_os_mutex_lock(&academy.mutex);
	vp_os_cond_signal(&academy.cond);
	vp_os_mutex_unlock(&academy.mutex);
}

void academy_setState(academy_state_t *state, ACADEMY_STATE academy_state)
{
	state->state 		= academy_state;
	state->result 	 	= ACADEMY_RESULT_NONE;
}

void academy_resetState(academy_state_t *state)
{
	state->state 		= ACADEMY_STATE_NONE;
	state->result 	 	= ACADEMY_RESULT_NONE;
}

void academy_nextState(academy_state_t *state)
{
	state->state 		= (state->state + 1) % ACADEMY_STATE_MAX;
	state->result 		= ACADEMY_RESULT_NONE;
}

void academy_stateOK(academy_state_t *state)
{
	state->result 		= ACADEMY_RESULT_OK;
}

void academy_stateERROR(academy_state_t *state)
{
	state->result 		= ACADEMY_RESULT_FAILED;
}

int academy_remove(const char* fpath, const struct stat *sb, int typeflag)
{
	if(typeflag == FTW_F)
		remove(fpath);

	return 0;
}

int academy_compute_memory_used(const char* fpath, const struct stat *sb, int typeflag)
{
	char prefix[FLIGHT_NAME_SIZE];
	sprintf(prefix, "%s/flight_", flight_dir);

	if(strncmp(fpath, prefix, strlen(prefix)) == 0)
	{
		if(typeflag == FTW_F)
		{
			academy.flight_sum += sb->st_size;
		}
		else if(typeflag == FTW_D)
		{
            if(academy.flight_oldest_date[0] == '\0')
            {
                sscanf(fpath + strlen(prefix), "%s", academy.flight_oldest_date);
            }
		}
	}

	return 0;
}

char *academy_get_next_item_with_prefix(const char *list, char **next_item, const char *prefix, bool_t isDirectory)
{
	char *result = NULL;
	static char directoryLine[ACADEMY_MAX_LINE];

	if(*next_item == NULL)
		*next_item = (char*)list;

	if(*next_item && prefix)
	{
		char *ptr = *next_item;
		while(!result && ptr)
		{
			ptr = strchr(*next_item, '\r');
			if(ptr != NULL)
			{
				int length = (int )(ptr - *next_item);
				memcpy(directoryLine, *next_item, length);
				directoryLine[length] = '\0';
				*next_item = ptr + 2; // To remove \r\n to end of line

				// One line found
				ptr = strrchr(directoryLine, ' ');
				if(ptr != NULL)
				{
					ptr++;
					if((directoryLine[0] == (isDirectory ? 'd' : '-')) && (strncmp(ptr, prefix, strlen(prefix)) == 0))
						result = ptr;
				}
			}
		}
	}

	return result;
}

// Can remove ftp_directory if there is only files inside
_ftp_status academy_remove_ftp_directory(_ftp_t *ftp, const char *directory_name)
{
	char *list = NULL;
	char *ptr = NULL;
	_ftp_status academy_status = ftpCd(ftp, directory_name);
	if(FTP_SUCCEDED(academy_status))
	{
		char *next_item = NULL;
		academy_status = ftpList(ftp, &list, NULL);
		PA_DEBUG("list file :\n%s\n", list);
		while(FTP_SUCCEDED(academy_status) && (ptr = academy_get_next_item_with_prefix(list, &next_item, "", FALSE)))
		{
			PA_DEBUG("Removing %s...\n", ptr);
			academy_status = ftpRemove(ftp, ptr);
		}

		if(list != NULL)
		{
			vp_os_free(list);
			list = NULL;
		}

		academy_status = ftpCd(ftp, "..");
		if(FTP_SUCCEDED(academy_status))
		{
			PA_DEBUG("Removing %s...\n", directory_name);
			academy_status = ftpRmdir(ftp, directory_name);
		}
	}

	return academy_status;
}

DEFINE_THREAD_ROUTINE(academy, data)
{
	
    bool_t needToCheckMemory = TRUE;
    
	printf("Start thread %s\n", __FUNCTION__);

	while( academy_started && !ardrone_tool_exit() )
	{
        if(!needToCheckMemory)
        {
            vp_os_mutex_lock(&academy.mutex);
            vp_os_cond_wait(&academy.cond);
            needToCheckMemory = TRUE;
            vp_os_mutex_unlock(&academy.mutex);
        }
        
        while(needToCheckMemory)
        {
            academy.flight_sum = 0;
            academy.flight_oldest_date[0] = '\0';
  #ifndef _WIN32 
			// according to http://mingw-users.1079350.n2.nabble.com/File-Tree-Walker-td5937923.html
			/* I am not aware of any open source, really good (simple but not too 
simple, with a C API) ftw() type function for Windows. Sure, you might 
find a straightforward no-frills Unix implementation that is easyish 
to port. Or you might find some monster C++ class for the task 
programmed in Microsoft style with <tchar.h> and other obsolete 
hideousness. Or if you are really unlucky, you might find Visual Basic 
code;). 

With "really good" I also mean one that actually would work in all 
situations on Windows. A straightforward port of obvious Unix code, 
using opendir(), readdir() , stat() etc, will not work in all cases. 
To be able to see all possible file names you need to use the wide 
character file names and APIs like _wopendir() etc. But otherwise such 
code should work.
*/

            if(!ftw(flight_dir, &academy_compute_memory_used, 1))
            {
                if(academy.flight_sum > MBYTE_TO_BYTE(academy.flight_max_storing_size))
                {
                    char remove_dir[ACADEMY_MAX_FILENAME];
                    sprintf(remove_dir, "%s/flight_%s", flight_dir, academy.flight_oldest_date);
                    
                    // remove oldest flight
                    PA_WARNING("Too much memory used %d MB > %d MB, removing oldest flight %s...", BYTE_TO_MBYTE(academy.flight_sum), academy.flight_max_storing_size, remove_dir);
                  
                    if(!ftw(remove_dir, &academy_remove, 1))
                    {
                        rmdir(remove_dir);
                        PA_WARNING("OK.\n");
                    }

                }
                else
                    needToCheckMemory = FALSE;
            }
            else
				#endif
                needToCheckMemory = FALSE;
        }
	}

	THREAD_RETURN(C_OK);
}
