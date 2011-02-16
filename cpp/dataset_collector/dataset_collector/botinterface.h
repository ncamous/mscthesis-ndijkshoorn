#pragma once

class mysocket;

class botinterface
{
public:
	botinterface(void);
	~botinterface(void);
	void botinterface::control_send(char *message);
	virtual void botinterface::control_receive(char *message, int bytes);

protected:
	mysocket *control_socket;
};

