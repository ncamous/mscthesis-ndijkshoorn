#pragma once
#include "botinterface.h"

class botinterface_usarsim :
	public botinterface
{
public:
	botinterface_usarsim(void);
	~botinterface_usarsim(void);
	void botinterface_usarsim::control_receive(char *message, int bytes);
};

