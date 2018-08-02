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
	////////////////////////////////////////�����׽���////////////////////////////////////////////////////  
	WORD wVersionRequested;//�׽��ֿ�汾��  
	WSADATA wsaData;
	int err;

	wVersionRequested = MAKEWORD(2, 2);//�����׽��ֵİ汾��  

	err = WSAStartup(wVersionRequested, &wsaData);//�����׽���  
	if (err != 0) {
		return 0;
	}
	///�����׽���ʧ�ܴ���  
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
				//static int i = 0;
				//if (i++ > 1000)
				//	break;
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