#include "botinterface_usarsim.h"
#include "global.h"
#include "mysocket.h"


botinterface_usarsim::botinterface_usarsim(void) : botinterface()
{
	control_socket = new mysocket(DEFAULT_USARIM_PORT, DEFAULT_USARSIM_IP, (botinterface*) this);
}


botinterface_usarsim::~botinterface_usarsim(void)
{
}

void botinterface_usarsim::control_receive(char *message, int bytes)
{
	message[bytes] = '\0';
	printf("CONTROL RECEIVE: %s\n", message);
}