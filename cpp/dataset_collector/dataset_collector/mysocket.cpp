#include "global.h"
#include "mysocket.h"
#include "windows.h"
#include "botinterface.h"

mysocket::mysocket(int PortNo, char *IPAddress, botinterface *i)
{
	bool result;
	connected = false;
	this->i = i;

	result = ConnectToHost(PortNo, IPAddress);

	if (result)
	{
		connected = true;
		StartListener();
	}
}


mysocket::~mysocket(void)
{
	CloseConnection();
}


int mysocket::Send(char *message)
{
	return send(s, message, strlen(message), 0);
}


static DWORD WINAPI ReceiveThread(void* Param)
{
	mysocket* This = (mysocket*) Param; 

	int bytes;
	bytes = SOCKET_ERROR;
	char *cServerMessage;
	cServerMessage = new char[600];

	while(bytes = recv(This->s, cServerMessage, 599, 0))
	{
		if(bytes == SOCKET_ERROR)
		{
			//CloseConnection();
			return 0;
		}
    
		if (bytes == 0 || bytes == WSAECONNRESET)
		{
			//CloseConnection();
			return 0;
		}

		if(bytes < 1)
		{
			Sleep(300);
			continue;
		}

		This->i->control_receive(cServerMessage, bytes);
		delete [] cServerMessage;
		cServerMessage = new char[600];
		Sleep(100); // Don't consume too much CPU power.
	}

	return 0;
}

int mysocket::StartListener(void)
{
	DWORD ThreadID;
	h = CreateThread( NULL, 0, ReceiveThread, (void*) this, 0, &ThreadID);

	return 1;
}


bool mysocket::ConnectToHost(int PortNo, char* IPAddress)
{
    //Start up Winsock

    WSADATA wsadata;

    int error = WSAStartup(0x0202, &wsadata);

    //Did something happen?

    if (error)
        return false;

    //Did we get the right Winsock version?

    if (wsadata.wVersion != 0x0202)
    {
        WSACleanup(); //Clean up Winsock

        return false;
    }

    //Fill out the information needed to initialize a socket�

    SOCKADDR_IN target; //Socket address information

    target.sin_family = AF_INET; // address family Internet

    target.sin_port = htons (PortNo); //Port to connect on

    target.sin_addr.s_addr = inet_addr (IPAddress); //Target IP


    s = socket (AF_INET, SOCK_STREAM, IPPROTO_TCP); //Create socket

    if (s == INVALID_SOCKET)
    {
        return false; //Couldn't create the socket

    }  

    //Try connecting...


    if (connect(s, (SOCKADDR *)&target, sizeof(target)) == SOCKET_ERROR)
    {
        return false; //Couldn't connect

    }
    else
        return true; //Success

}

//CLOSECONNECTION � shuts down the socket and closes any connection on it

void mysocket::CloseConnection ()
{
    //Close the socket if it exists

    if (s)
        closesocket(s);

    WSACleanup(); //Clean up Winsock

	connected = false;
}