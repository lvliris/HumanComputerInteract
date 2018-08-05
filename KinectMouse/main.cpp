#include "stdafx.h"
#include "myKinect.h"
#include <iostream>
#include <fstream>
#include <windows.h>
//#include <winsock2.h>
#include <math.h>
#include <time.h>

#pragma comment(lib, "ws2_32.lib")
//#pragma comment(lib, "KinectMouse.lib")
using namespace std;
using namespace cv;

#define TIMER

int main()
{
	////////////////////////////////////////加载套接字////////////////////////////////////////////////////  
	WORD wVersionRequested;//套接字库版本号  
	WSADATA wsaData;
	int err;

	wVersionRequested = MAKEWORD(2, 2);//定义套接字的版本号  

	err = WSAStartup(wVersionRequested, &wsaData);//创建套接字  
	if (err != 0) {
		return 0;
	}
	///创建套接字失败处理  
	if (LOBYTE(wsaData.wVersion) != 2 ||
		HIBYTE(wsaData.wVersion) != 2)
	{
		WSACleanup();
		return 0;
	}

	SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock == INVALID_SOCKET)
	{
		cout << "failed to create socket" << endl;
	}

	SOCKADDR_IN addrServer;
	addrServer.sin_family = AF_INET;
	addrServer.sin_port = htons(0x6618);
	addrServer.sin_addr.s_addr = inet_addr("127.0.0.1");

	sendto(sock, "hello", 5, 0, (sockaddr*)&addrServer, sizeof(sockaddr));

	HandState handstate = HandState_Closed;
	short nhandstate = htons(handstate);
	CBodyBasics myKinect;
	//myKinect.AddROI(roi);
	HRESULT hr = myKinect.InitializeDefaultSensor();
	if (SUCCEEDED(hr)){
		while (1){
#ifdef TIMER
			time_t start = clock();
#endif
			hr = myKinect.Update();
			if (SUCCEEDED(hr))
			{
				sendto(sock, myKinect.data(), 108, 0, (sockaddr*)&addrServer, sizeof(sockaddr));
			}
#ifdef TIMER
			time_t end = clock();
			cout << "time: " << end - start << endl;
#endif			
		}
	}
	else{
		cout << "kinect initialization failed!" << endl;
		system("pause");
	}
}
