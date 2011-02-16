#include <math.h>
#include "mex.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#define _WIN32_WINNT 0x0501

#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <iphlpapi.h>
#include <stdio.h>
#include <unistd.h>

#pragma comment(lib, "ws2_32.lib")
#define DEFAULT_BUFLEN 512
#define DEFAULT_SLEEPTIME 50000

SOCKET ConnectSocket = INVALID_SOCKET;

int listener()
{
    WSADATA wsaData;
    int iResult;
    char recvbuf[DEFAULT_BUFLEN];
    char *buf;
    int recvbuflen = DEFAULT_BUFLEN;
    
    FILE *file = NULL;
    
    

    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed: %d\n", iResult);
        return 0;
    }
    
    
    struct addrinfo *result = NULL,
                    *ptr = NULL,
                    hints;

    ZeroMemory( &hints, sizeof(hints) );
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    
    // Resolve the server address and port
    iResult = getaddrinfo("localhost", "5003", &hints, &result);
    if (iResult != 0) {
        printf("getaddrinfo failed: %d\n", iResult);
        WSACleanup();
        return 1;
    }
    
    // Attempt to connect to the first address returned by
    // the call to getaddrinfo
    ptr=result;

    // Create a SOCKET for connecting to server
    ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype, 
        ptr->ai_protocol);
    
    if (ConnectSocket == INVALID_SOCKET) {
        printf("Error at socket(): %ld\n", WSAGetLastError());
        freeaddrinfo(result);
        WSACleanup();
        return 1;
    }
    
    // Connect to server.
    iResult = connect( ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
    if (iResult == SOCKET_ERROR) {
        closesocket(ConnectSocket);
        ConnectSocket = INVALID_SOCKET;
    }
 
    // Should really try the next address returned by getaddrinfo
    // if the connect call failed
    // But for this simple example we just free the resources
    // returned by getaddrinfo and print an error message

    freeaddrinfo(result);

    if (ConnectSocket == INVALID_SOCKET) {
        printf("Unable to connect to server!\n");
        WSACleanup();
        return 1;
    }
    
    // keep alive
	BOOL bOptVal = TRUE;
	int bOptLen = sizeof(BOOL);

 if (setsockopt(ConnectSocket, SOL_SOCKET, SO_KEEPALIVE, (char*)&bOptVal, bOptLen) != SOCKET_ERROR) {
    printf("Set SO_KEEPALIVE: ON\n");
  }
    
    
    usleep(DEFAULT_SLEEPTIME);
        
    // Receive data until the server closes the connection
    int i = 1;
    int bytesread = 0;
    int bytesoffset = 0;
    int bufferlen = 0;
    char filename[20];
    do {    
        iResult = recv(ConnectSocket, recvbuf, recvbuflen, 0);
                      
        if (iResult > 0) {
            // open file
            if (!file) {
                sprintf(filename, "img/%i.jpg", i++);
                file = fopen(filename, "wb");
            }
            
            /*printf("Bytes received: %d\n", iResult);*/
            buf = recvbuf;
            bufferlen = iResult;
                       
            if (bytesread <= 5 && bytesread + iResult > 5) {
                bytesoffset = 5 - bytesread;
                buf = &buf[bytesoffset];
                bufferlen -= bytesoffset;
            }
            
            bytesread += iResult;

            if (bytesread > 5)
                fwrite(buf, 1, bufferlen, file);
                       
        // request next frame
        }
        
        if (iResult < DEFAULT_BUFLEN && bytesread > 0) {
            fclose(file);
            file = NULL;
            bytesread = 0;

            sendOK();
            usleep(DEFAULT_SLEEPTIME);
        }

    } while (iResult > 0);
    
    fclose(file);
    
	closesocket(ConnectSocket);
	WSACleanup();  
}

int sendOK()
{
	int iResult;
    char *sendbuf = "OK";
    
    // Send an initial buffer
    iResult = send( ConnectSocket, sendbuf, (int)strlen(sendbuf), 0 );
    if (iResult == SOCKET_ERROR) {
        printf("send failed: %d\n", WSAGetLastError());
        closesocket(ConnectSocket);
        WSACleanup();
        return 1;
    }
}

void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[] ) 
{ 
      
    listener(); 

    return;
    
}
