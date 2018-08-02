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
	//kinect 2.0 的深度空间的高*宽是 424 * 512，在官网上有说明
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
	HRESULT	                Update();//获得骨架、背景二值图和深度信息
	HRESULT                 InitializeDefaultSensor();//用于初始化kinect
	HRESULT					closeSensor();
	HRESULT					AddROI(cv::Rect rect);//添加一块感兴趣的区域，输出坐标一旦进入ROI，则定位到ROI中心，快速摆动离开
	HRESULT					RemoveROI(cv::Rect rect);//删除某块感兴趣的区域
	HRESULT					SetScaleFactor(float scale);
	char*					data();

	int Xout;
	int Yout;
	int Zout;

private:
	IKinectSensor*          m_pKinectSensor;//kinect源
	ICoordinateMapper*      m_pCoordinateMapper;//用于坐标变换
	IColorFrameReader*		m_pColorFrameReader;//用于彩色图像读取
	IBodyFrameReader*       m_pBodyFrameReader;//用于骨架数据读取
	IDepthFrameReader*      m_pDepthFrameReader;//用于深度数据读取
	IBodyIndexFrameReader*  m_pBodyIndexFrameReader;//用于背景二值图读取
	IInfraredFrameReader*   m_pInfraredFrameReader;
	IFaceFrameSource*		m_pFaceFrameSources[BODY_COUNT];
	IFaceFrameReader*		m_pFaceFrameReaders[BODY_COUNT];//用于面部帧读取

	//通过获得到的信息，把骨架和背景二值图画出来
	HRESULT ProcessBody(int nBodyCount, IBody** ppBodies);
	//处理面部帧，定位面部位置和眼睛的位置
	HRESULT ProcessFaces();
	void ProcessFace(IFaceFrameResult *pFaceFrameResult);
	HRESULT GetFaceTextPositionInColorSpace(IBody* pBody, DetectionResult faceProperties[]);
	//画骨架函数
	void DrawBone(const Joint* pJoints, const ColorSpacePoint* depthSpacePosition, JointType joint0, JointType joint1);
	void DrawBone(const Joint* pJoints, const DepthSpacePoint* depthSpacePosition, JointType joint0, JointType joint1);
	//画手的状态函数
	void DrawHandState(const ColorSpacePoint depthSpacePosition, HandState handState);
	void DrawHandState(const DepthSpacePoint depthSpacePosition, HandState handState);
	//对跟踪到的手的坐标位置进行处理，防止跳动和蠕动
	void CBodyBasics::ProcessHandPosition(float &x, float &y);
	//模拟按键事件
	void PressKey(vk key);
	//判断两关节点重叠
	bool IsTwoColorSpacePointOverlapping(const ColorSpacePoint point1, const ColorSpacePoint point2);
	//是否结束控制
	bool FinishControl(Joint *joints, HandState LeftHandState, HandState RightHandState);
	//是否开始进行控制
	bool StartControl(Joint *joints, HandState LeftHandState, HandState RightHandState);
	//判断是否处于有效位置
	bool ValidControlPosition(Joint *joints);
	//判断是否为获取控制权的动作
	bool getControl(Joint *joints);
	//肢体语言映射到键盘事件
	void MappingGestureToKeyboard(IBody *pBody);
	//判断输出是否进入ROI
	bool CheckROI(int &x, int &y);

	//双参数线性指数滤波器
	FilterDoubleExponential filter;
	//输出视频流
	cv::VideoWriter outVideo;

	//显示图像的Mat
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
	const float m_MoveThresh = 0.04;//移动阈值，高于此值不进行移动，防止骨骼坐标抖动导致鼠标大范围抖动
	const float m_SelectThresh = 0.002;//选择模式移动阈值，高于此值不进行移动，防止手从展开到握起时坐标发生移动
	const int m_SelectCounts = 33;//图像采集速度30FPS，2秒即采集60次，进入选择模式
};

