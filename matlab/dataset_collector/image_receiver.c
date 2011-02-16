/*=================================================================
 *
 * YPRIME.C	Sample .MEX file corresponding to YPRIME.M
 *	        Solves simple 3 body orbit problem 
 *
 * The calling syntax is:
 *
 *		[yp] = yprime(t, y)
 *
 *  You may also want to look at the corresponding M-code, yprime.m.
 *
 * This is a MEX-file for MATLAB.  
 * Copyright 1984-2006 The MathWorks, Inc.
 *
 *=================================================================*/
/* $Revision: 1.10.6.4 $ */
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
#include <process.h> //used for thread
#include <pthread.h> /* for threading */
#include "engine.h"
#include "matrix.h"
// #include "engCallMATLAB.c"

#pragma comment(lib, "ws2_32.lib")
#define DEFAULT_BUFLEN 512
#define DEFAULT_SLEEPTIME 100000

void *listener(void *threadid);
void notify_matlab(int i);
int sendOK();

Engine *ep;
SOCKET ConnectSocket = INVALID_SOCKET;

void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[] ) 
{    
    int ThreadNr; //needed for the thread
    
    /*ep = /( NULL );
    if( ep == NULL )
    {
        printf("Engine did not open.\n");
        printf("One possible reason is MATLAB is not a registered server.\n");
        printf("From Windows, open a Command Prompt and enter:\n");
        printf(">matlab /regserver\n");
        return;
    }*/
    
    
    //start your thread
    //_beginthread( listener, 0, &ThreadNr );
    
    int rc, t;
    int num = 1;
    pthread_t threads[num];
    
    
    rc = pthread_create(&threads[0], NULL, listener, (void *)t);
    if (rc){
        mexErrMsgTxt("problem with return code from pthread_create()");
    }
}

void *listener(void *threadid)
{
    WSADATA wsaData;
    int iResult;
    char recvbuf[DEFAULT_BUFLEN];
    char *buf;
    int recvbuflen = DEFAULT_BUFLEN;
    
    int tid;
    tid = (int)threadid;
    
    FILE *file = NULL;
    
    

    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed: %d\n", iResult);
        return;
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
        return;
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
        return;
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
        return;
    }
    
    // keep alive
	BOOL bOptVal = TRUE;
	int bOptLen = sizeof(BOOL);
    
    
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
            notify_matlab (i);

            sendOK();
            usleep(DEFAULT_SLEEPTIME);
        }

    } while (iResult > 0);
    
    fclose(file);
    
	closesocket(ConnectSocket);
	WSACleanup();  
}

void notify_matlab(int i)
{
    mxArray *prhs1[1];
   
	//mlfAssign(&prhs1[0], mlfScalar(i));

    //mexCallMATLAB(0,NULL,1,prhs1,"ui_display_image");
    mexCallMATLAB(0, NULL, 0, NULL, "ui_display_image");
    
/*
    mxArray   *new_number, *str;
  double out;

  str = mxCreateString("Enter extension:  ");
  //mexCallMATLAB(1,&new_number,1,&str,"input");
  
  engCallMATLAB( ep, 1, &new_number,1,&str,"input");
  
  out = mxGetScalar(new_number);
  mexPrintf("You entered: %.0f ", out);
  mxDestroyArray(new_number);
  mxDestroyArray(str);
  return;   
*/
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
