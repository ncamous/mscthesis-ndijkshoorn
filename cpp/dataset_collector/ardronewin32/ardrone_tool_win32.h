#ifndef _ARDRONE_TOOL_WIN32_H_
#define _ARDRONE_TOOL_WIN32_H_

#pragma warning( disable : 4996 ) // disable deprecation warning 

#ifdef __cplusplus

extern "C" {
	int ardronewin32();
	void ardronewin32_take_off();
	void ardronewin32_land();
	void ardronewin32_progress(int enable, float roll, float pitch, float gaz, float yaw);
	void ardronewin32_recover(int send);
}

#endif

#endif