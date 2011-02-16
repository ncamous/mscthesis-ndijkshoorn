#include "global.h"
#include "botinterface.h"
#include "mysocket.h"


botinterface::botinterface(void)
{
}


botinterface::~botinterface(void)
{
}


void botinterface::control_send(char *message)
{
	control_socket->Send(message);
}


void botinterface::control_receive(char *message, int bytes)
{
}