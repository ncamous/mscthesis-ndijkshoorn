#pragma once

class botinterface
{
public:
	virtual void init(void) = 0;
	virtual void socket_callback(int id, char *message, int bytes) = 0;
	virtual void control_update(void *control) = 0;
};
