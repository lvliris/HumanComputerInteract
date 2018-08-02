#pragma once

//#define DLL_EXPORT
//#ifdef DLL_EXPORT
//#define DLL_API __declspec(dllexport)
//#else
//#define DLL_API __declspec(dllimport)
//#endif

#include <Kinect.h>
#include <Kinect.Face.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "KinectJointFilter.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <windows.h>
#include <time.h>

#define MAX_MOUSE_X 750
#define MAX_MOUSE_Y 450

#define debug

//#ifdef debug
//#define log(...) printf(__VA_ARG__)
//#else
//#define log(...)
//#endif

enum vk
{
	BACKSPACE = 8,
	ENTER = 13,
	SHIFT = 16,
	CTRL = 17,
	ALT = 18,
	LEFT_ARROW = 37,
	K = 'K',
	S = 'S',
	A = 'A',
	D = 'D',
	UP_ARROW = 38,
	RIGHT_ARROW = 39,
	DOWN_ARROW = 40,
	F1 = 112,
	F2,
	F3,
	F4,
	F5,
	F6,
	F7,
	F8,
	F9,
	F10,
	F11,
	F12
};

enum MoveMode
{
	MoveMode_Select = 0,
	MoveMode_Move,
	MoveMode_Quick
};

enum MessageMark
{
	Message_Header = 0,
	Message_Length = 2,
	Message_WorkingTime = 4,
	Message_Reserved1 = 8,
	Message_Detect = 10,
	Message_EyesMid = 12,
	Message_RightEye = 24,
	Message_LeftEye = 36,
	Message_RightHand = 48,
	Message_LeftHand = 60,
	Message_FaceMid = 72,
	Message_FaceNormal = 84,
	Message_RightHandState = 96,
	Message_LeftHandState = 98,
	Message_FaceHeight = 100,
	Message_FaceWidth = 102,
	Message_Reserved2 = 104,
	Message_Total = 108
};

enum DetectBit
{
	DetectBit_Face = 0,
	DetectBit_LeftEye,
	DetectBit_RightEye,
	DetectBit_RightHand,
	DetectBit_LeftHand
};

// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != nullptr)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = nullptr;
	}
}

class CBodyBasics
{
	//kinect 2.0 ����ȿռ�ĸ�*���� 424 * 512���ڹ�������˵��
	static const int        cDepthWidth = 512;
	static const int        cDepthHeight = 424;

	//color
	static const int cColorWidth = 1920;
	static const int cColorHeight = 1080;

	const int overlapThres = 200;
	const float ArmRaisedThreshold = 0.2;
	const float ArmStretchedThreshold = 0.3;
	const float JumpDiffThreshold = 0.05;
	float HeadPreviousPosition = 2;

public:
	CBodyBasics();
	~CBodyBasics();
	HRESULT	                Update();//��ùǼܡ�������ֵͼ�������Ϣ
	HRESULT                 InitializeDefaultSensor();//���ڳ�ʼ��kinect
	HRESULT					closeSensor();
	HRESULT					AddROI(cv::Rect rect);//���һ�����Ȥ�������������һ������ROI����λ��ROI���ģ����ٰڶ��뿪
	HRESULT					RemoveROI(cv::Rect rect);//ɾ��ĳ�����Ȥ������
	HRESULT					SetScaleFactor(float scale);
	char*					data();

	int Xout;
	int Yout;
	int Zout;

private:
	IKinectSensor*          m_pKinectSensor;//kinectԴ
	ICoordinateMapper*      m_pCoordinateMapper;//��������任
	IColorFrameReader*		m_pColorFrameReader;//���ڲ�ɫͼ���ȡ
	IBodyFrameReader*       m_pBodyFrameReader;//���ڹǼ����ݶ�ȡ
	IDepthFrameReader*      m_pDepthFrameReader;//����������ݶ�ȡ
	IBodyIndexFrameReader*  m_pBodyIndexFrameReader;//���ڱ�����ֵͼ��ȡ
	IInfraredFrameReader*   m_pInfraredFrameReader;
	IFaceFrameSource*		m_pFaceFrameSources[BODY_COUNT];
	IFaceFrameReader*		m_pFaceFrameReaders[BODY_COUNT];//�����沿֡��ȡ

	//ͨ����õ�����Ϣ���ѹǼܺͱ�����ֵͼ������
	HRESULT ProcessBody(int nBodyCount, IBody** ppBodies);
	//�����沿֡����λ�沿λ�ú��۾���λ��
	HRESULT ProcessFaces();
	void ProcessFace(IFaceFrameResult *pFaceFrameResult);
	HRESULT GetFaceTextPositionInColorSpace(IBody* pBody, DetectionResult faceProperties[]);
	//���Ǽܺ���
	void DrawBone(const Joint* pJoints, const ColorSpacePoint* depthSpacePosition, JointType joint0, JointType joint1);
	void DrawBone(const Joint* pJoints, const DepthSpacePoint* depthSpacePosition, JointType joint0, JointType joint1);
	//���ֵ�״̬����
	void DrawHandState(const ColorSpacePoint depthSpacePosition, HandState handState);
	void DrawHandState(const DepthSpacePoint depthSpacePosition, HandState handState);
	//�Ը��ٵ����ֵ�����λ�ý��д�����ֹ�������䶯
	void CBodyBasics::ProcessHandPosition(float &x, float &y);
	//ģ�ⰴ���¼�
	void PressKey(vk key);
	//�ж����ؽڵ��ص�
	bool IsTwoColorSpacePointOverlapping(const ColorSpacePoint point1, const ColorSpacePoint point2);
	//�Ƿ��������
	bool FinishControl(Joint *joints, HandState LeftHandState, HandState RightHandState);
	//�Ƿ�ʼ���п���
	bool StartControl(Joint *joints, HandState LeftHandState, HandState RightHandState);
	//�ж��Ƿ�����Чλ��
	bool ValidControlPosition(Joint *joints);
	//�ж��Ƿ�Ϊ��ȡ����Ȩ�Ķ���
	bool getControl(Joint *joints);
	//֫������ӳ�䵽�����¼�
	void MappingGestureToKeyboard(IBody *pBody);
	//�ж�����Ƿ����ROI
	bool CheckROI(int &x, int &y);

	//˫��������ָ���˲���
	FilterDoubleExponential filter;
	//�����Ƶ��
	cv::VideoWriter outVideo;

	//��ʾͼ���Mat
	cv::Mat m_SkeletonImg;
	cv::Mat m_DepthImg;
	cv::Mat m_ColorImg;

	cv::Size m_ControlSpaceSize;
	std::vector<cv::Rect> m_roi;
	float m_ScaleFactor;
	bool m_ControlStatus;
	int m_Controller;
	short m_Detect;

	time_t m_StartTimeStamp;
	time_t m_PresentTimeStamp;

	MoveMode m_MoveMode;

	char m_Output[108];

	const float m_QuickThresh = 0.04;//0.06;
	const float m_MoveThresh = 0.04;//�ƶ���ֵ�����ڴ�ֵ�������ƶ�����ֹ�������궶����������Χ����
	const float m_SelectThresh = 0.002;//ѡ��ģʽ�ƶ���ֵ�����ڴ�ֵ�������ƶ�����ֹ�ִ�չ��������ʱ���귢���ƶ�
	const int m_SelectCounts = 33;//ͼ��ɼ��ٶ�30FPS��2�뼴�ɼ�60�Σ�����ѡ��ģʽ
};

