#pragma once

#include <winsock.h>
#include <windows.h>

class botinterface;

#define SCK_VERSION1            0x0101
#define SCK_VERSION2            0x0202

static DWORD WINAPI ReceiveThread(void* Param);

class mysocket
{
	HANDLE h;

	int mysocket::StartListener(void);
	bool mysocket::ConnectToHost(int PortNo, char *IPAddress);
	void mysocket::CloseConnection ();

public:
	mysocket::mysocket(int PortNo, char *IPAddress, botinterface *i);
	~mysocket(void);
	int mysocket::Send(char *message);

	SOCKET s;
	bool connected;
	botinterface *i;
};

