#pragma once

class botinterface
{
public:
	virtual void socket_callback(int id, char *message, int bytes) = 0;
};

