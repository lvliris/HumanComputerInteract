#include "stdafx.h"
#include "myKinect.h"


using namespace cv;
using namespace std;

static const DWORD c_FaceFrameFeatures =
FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace
| FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInInfraredSpace
| FaceFrameFeatures::FaceFrameFeatures_PointsInColorSpace
| FaceFrameFeatures::FaceFrameFeatures_PointsInInfraredSpace
| FaceFrameFeatures::FaceFrameFeatures_RotationOrientation;

/// Initializes the default Kinect sensor
HRESULT CBodyBasics::InitializeDefaultSensor()
{
	//用于判断每次读取操作的成功与否
	HRESULT hr;

	//搜索kinect
	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr)){
		return hr;
	}

	//找到kinect设备
	if (m_pKinectSensor)
	{
		// Initialize the Kinect and get coordinate mapper and the body reader
		IColorFrameSource* pColorFrameSource = nullptr;//读取彩色图
		IBodyFrameSource* pBodyFrameSource = nullptr;//读取骨架
		IDepthFrameSource* pDepthFrameSource = nullptr;//读取深度信息
		IBodyIndexFrameSource* pBodyIndexFrameSource = nullptr;//读取背景二值图
		IInfraredFrameSource* pInfraredFrameSource = nullptr;
		//IFaceFrameSource* pFaceFrameSource = nullptr;

		//打开kinect
		hr = m_pKinectSensor->Open();

		//coordinatemapper
		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
		}

		//colorframe
		/*if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
		}
		if (SUCCEEDED(hr))
		{
			hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
		}*/

		//bodyframe
		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
		}

		//depth frame
		if (SUCCEEDED(hr)){
			hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
		}

		if (SUCCEEDED(hr)){
			hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
		}

		//body index frame
		if (SUCCEEDED(hr)){
			hr = m_pKinectSensor->get_BodyIndexFrameSource(&pBodyIndexFrameSource);
		}

		if (SUCCEEDED(hr)){
			hr = pBodyIndexFrameSource->OpenReader(&m_pBodyIndexFrameReader);
		}

		//infrared frame
		if (SUCCEEDED(hr)){
			hr = m_pKinectSensor->get_InfraredFrameSource(&pInfraredFrameSource);
		}

		if (SUCCEEDED(hr)){
			hr = pInfraredFrameSource->OpenReader(&m_pInfraredFrameReader);
		}

		//face frame
		if (SUCCEEDED(hr))
		{
			for (int i = 0; i < BODY_COUNT; i++)
			{
				if (SUCCEEDED(hr))
				{
					hr = CreateFaceFrameSource(m_pKinectSensor, 0, c_FaceFrameFeatures, &m_pFaceFrameSources[i]);
				}
				if (SUCCEEDED(hr))
				{
					hr = m_pFaceFrameSources[i]->OpenReader(&m_pFaceFrameReaders[i]);
				}
				if (SUCCEEDED(hr))
				{
					//cout << "face reader succeed " << i << endl;
				}
			}
		}

		//SafeRelease(pColorFrameSource);
		SafeRelease(pBodyFrameSource);
		SafeRelease(pDepthFrameSource);
		SafeRelease(pBodyIndexFrameSource);
		//SafeRelease(pFaceFrameSource);
	}

	if (!m_pKinectSensor || FAILED(hr))
	{
		std::cout << "Kinect initialization failed!" << std::endl;
		return E_FAIL;
	}

	//m_ColorImg
	/*m_ColorImg.create(cColorHeight, cColorWidth, CV_8UC3);
	m_ColorImg.setTo(0);*/

	//m_SkeletonImg,用于画骨架、背景二值图的MAT
	m_SkeletonImg.create(cDepthHeight, cDepthWidth, CV_8UC3);
	m_SkeletonImg.setTo(0);

	//m_DepthImg,用于画深度信息的MAT
	m_DepthImg.create(cDepthHeight, cDepthWidth, CV_8UC1);
	m_DepthImg.setTo(0);

	//m_pKinectSensor->Close();

	//outVideo.open("./test.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10.0, Size(1920, 1080), true);//CV_FOURCC('M', 'P', '4', '2')
	//if (!outVideo.isOpened())
	//{
	//	cout << "failed to open outvideo" << endl;
	//}
	//outVideo.release();

	return hr;
}

HRESULT CBodyBasics::closeSensor()
{
	if (m_pKinectSensor)
		m_pKinectSensor->Close();
	return 0;
}

/// Main processing function
HRESULT CBodyBasics::Update()
{
	//每次先清空m_SkeletonImg
	m_SkeletonImg.setTo(0);
	m_Detect = 0;
	memset(m_Output + Message_Header + Message_Length, 0, Message_Total - Message_Header - Message_Length);

	//如果丢失了kinect，则不继续操作
	if (!m_pBodyFrameReader)
	{
		return E_FAIL;
	}

	time_t presentTime = clock();
	int WorkingTime = presentTime - m_StartTimeStamp;
	int nWorkingTime = htonl(WorkingTime);
	memcpy(m_Output + Message_WorkingTime, &nWorkingTime, 4);

	IColorFrame* pColorFrame = nullptr;//彩色图像信息
	IBodyFrame* pBodyFrame = nullptr;//骨架信息
	IDepthFrame* pDepthFrame = nullptr;//深度信息
	IBodyIndexFrame* pBodyIndexFrame = nullptr;//背景二值图
	IFaceFrame *pFaceFrame = nullptr;

	//记录每次操作的成功与否
	HRESULT hr;

	//-----------------------------获取骨架并显示----------------------------
	hr = ProcessFaces();

	if (SUCCEEDED(hr))
	{
		short nDetect = htons(m_Detect);
		//cout << "detect result:" << hex << m_Detect << endl;
		memcpy(m_Output + Message_Detect, &nDetect, 2);
		int *pos = (int*)(m_Output + Message_EyesMid);

		imshow("depth image", m_SkeletonImg);
		//int static picnum = 0;
		//string path = "./pic/" + to_string(picnum) + ".jpg";
		//imwrite(path, m_SkeletonImg);
		//picnum++;
		cv::waitKey(10);
		return S_OK;
	}
	else
		return E_FAIL;

}

/// Handle new body data
HRESULT CBodyBasics::ProcessBody(int nBodyCount, IBody** ppBodies)
{
	if (nullptr == m_pBodyFrameReader)
		return E_FAIL;

	//记录操作结果是否成功
	HRESULT hr;
	static int maskType = 0;
	static int leave = 1;

	IBodyFrame* pBodyFrame = nullptr;
	hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);//获取骨架信息

	if (SUCCEEDED(hr))
	{
		hr = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, ppBodies);
	}

	int nTracked = 0;
	int tempController = -1;
	//对于每一个IBody
	for (int i = 0; i < nBodyCount; ++i)
	{
		IBody* pBody = ppBodies[i];
		if (pBody)
		{
			BOOLEAN bTracked = false;
			hr = pBody->get_IsTracked(&bTracked);

			if (SUCCEEDED(hr) && bTracked)
			{
				nTracked++;
				tempController = i;
				Joint joints[JointType_Count];//存储关节点类
				static HandState preHandState = HandState_Unknown;
				HandState leftHandState = HandState_Unknown;//左手状态
				HandState rightHandState = HandState_Unknown;//右手状态
				TrackingState leftHandTrackingState = TrackingState_NotTracked;
				TrackingState rightHandTrackingState = TrackingState_NotTracked;

				//获取左右手状态
				pBody->get_HandLeftState(&leftHandState);
				pBody->get_HandRightState(&rightHandState);

				short nLeftHandState = htons(leftHandState);
				short nRightHandState = htons(rightHandState);

				memcpy(m_Output + Message_LeftHandState, &nLeftHandState, 2);
				memcpy(m_Output + Message_RightHandState, &nRightHandState, 2);

				//存储彩色坐标系中的关节点位置
				//ColorSpacePoint *colorSpacePosition = new ColorSpacePoint[_countof(joints)];
				DepthSpacePoint *depthSpacePosition = new DepthSpacePoint[_countof(joints)];

				//获得关节点类
				hr = pBody->GetJoints(_countof(joints), joints);
				if (SUCCEEDED(hr))
				{
					if (getControl(joints))
						m_Controller = i;
					if (i != m_Controller)
						continue;
					if (StartControl(joints, leftHandState, rightHandState))
						m_ControlStatus = true;
					if (FinishControl(joints, leftHandState, rightHandState))
						m_ControlStatus = false;
					ofstream fout;
					fout.open("./hwey.txt", ios::out | ios::app);
					fout << joints[JointType_HandRight].Position.Y << ' ' << joints[JointType_WristRight].Position.Y << ' ' << joints[JointType_ElbowRight].Position.Y << endl;
						//<< ' ' << joints[JointType_ThumbRight].Position.Y << ' ' << joints[JointType_HandTipRight].Position.Y << endl;
					fout.close();
					fout.open("./hwex.txt", ios::out | ios::app);
					fout << joints[JointType_HandRight].Position.X << ' ' << joints[JointType_WristRight].Position.X << ' ' << joints[JointType_ElbowRight].Position.X << endl;
					//<< ' ' << joints[JointType_ThumbRight].Position.Y << ' ' << joints[JointType_HandTipRight].Position.Y << endl;
					fout.close();
					leftHandTrackingState = joints[JointType_WristLeft].TrackingState;
					rightHandTrackingState = joints[JointType_WristRight].TrackingState;

					if (leftHandTrackingState == TrackingState_Tracked)
						m_Detect |= 1 << DetectBit_LeftHand;
					else
						m_Detect &= ~(1 << DetectBit_LeftHand);

					if (rightHandTrackingState == TrackingState_Tracked)
						m_Detect |= 1 << DetectBit_RightHand;
					else
						m_Detect &= ~(1 << DetectBit_RightHand);

					//MappingGestureToKeyboard(pBody);
					//for (int j = 0; j < _countof(joints); ++j)
					//{
					//	//将关节点坐标从摄像机坐标系（-1~1）转到彩色图像坐标系（1920*1080）
					//	m_pCoordinateMapper->MapCameraPointToColorSpace(joints[j].Position, &colorSpacePosition[j]);
					//}
					for (int j = 0; j < _countof(joints); ++j)
					{
						//将关节点坐标从摄像机坐标系（-1~1）转到深度图像坐标系（512*424）
						m_pCoordinateMapper->MapCameraPointToDepthSpace(joints[j].Position, &depthSpacePosition[j]);
					}
					static float prex = 0;
					static float prey = 0;
					char buff[128] = { 0 };
					/*int cx = colorSpacePosition[JointType_HandRight].X;
					int cy = colorSpacePosition[JointType_HandRight].Y;*/
					int cx = depthSpacePosition[JointType_WristRight].X;
					int cy = depthSpacePosition[JointType_WristRight].Y;
					/*sprintf_s(buff, "(%d,%d)", cx, cy);
					putText(m_ColorImg, buff, Point(cx, cy), CV_FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 0, 0), 2, 8);*/
					//int x = colorSpacePosition[JointType_WristRight].X * MAX_MOUSE_X / cColorWidth;
					//int y = colorSpacePosition[JointType_WristRight].Y * MAX_MOUSE_Y / cColorHeight;
					
					//ProcessHandPosition(dx, dy);
					//FilterDoubleExponential filter;
					//ofstream fout;
					//fout.open("./rawdata.txt", ios::app | ios::out);
					//fout << joints[JointType_WristRight].Position.X << endl;
					//fout.close();

					TRANSFORM_SMOOTH_PARAMETERS smoothingParameters(0.5, 0.5, 0.5, 0.03, 0.04);// 0.05f, 0.02f, 0.05f, 0.02f, 0.02f);
					//time_t start = clock();
					filter.MidFilter(joints[JointType_WristRight], joints[JointType_WristRight]);
					//time_t end = clock();
					//cout << "midfilter time: " << end - start << endl;
					filter.Update(joints, (UINT)JointType_WristRight, smoothingParameters);
					filter.Update(joints, (UINT)JointType_WristLeft, smoothingParameters);
					const DirectX::XMVECTOR* FilteredJoint = filter.GetFilterJoints();

					CameraSpacePoint RightHandPos,LeftHandPos;
					DepthSpacePoint depthPos;

					RightHandPos.X = DirectX::XMVectorGetX(FilteredJoint[JointType_WristRight]);
					RightHandPos.Y = DirectX::XMVectorGetY(FilteredJoint[JointType_WristRight]);
					RightHandPos.Z = DirectX::XMVectorGetZ(FilteredJoint[JointType_WristRight]);

					LeftHandPos.X = DirectX::XMVectorGetX(FilteredJoint[JointType_WristLeft]);
					LeftHandPos.Y = DirectX::XMVectorGetY(FilteredJoint[JointType_WristLeft]);
					LeftHandPos.Z = DirectX::XMVectorGetZ(FilteredJoint[JointType_WristLeft]);

					m_pCoordinateMapper->MapCameraPointToDepthSpace(RightHandPos, &depthPos);

					int nRightHandPos[3];
					nRightHandPos[0] = htonl(RightHandPos.X * 1000);
					nRightHandPos[1] = htonl(RightHandPos.Y * 1000);
					nRightHandPos[2] = htonl(RightHandPos.Z * 1000);

					int nLeftHandPos[3];
					nLeftHandPos[0] = htonl(LeftHandPos.X * 1000);
					nLeftHandPos[1] = htonl(LeftHandPos.Y * 1000);
					nLeftHandPos[2] = htonl(LeftHandPos.Z * 1000);

					memcpy(m_Output + Message_RightHand, nRightHandPos, 12);
					memcpy(m_Output + Message_LeftHand, nLeftHandPos, 12);

					float x = RightHandPos.X;// *ScaleFactor;
					float y = RightHandPos.Y;// *ScaleFactor;
					//int factor = 3;

					//int x = colorPos.X * MAX_MOUSE_X * factor / cColorWidth;
					//int y = colorPos.Y * MAX_MOUSE_Y * factor/ cColorHeight;
					//sprintf_s(buff, "(%.2lf,%.2lf,%.2lf),(%d,%d)", position.X, position.Y, position.Z, colorPos.X, colorPos.Y);
					sprintf_s(buff, "(%.3lf,%.3lf,%.3lf)/m", RightHandPos.X, RightHandPos.Y, RightHandPos.Z);
					putText(m_SkeletonImg, buff, Point(cx, cy), CV_FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2, 8); 
					float dx = x - prex;
					float dy = y - prey;
					ProcessHandPosition(dx, dy);
					//fout.open("./filtered.txt", ios::app | ios::out);
					//fout << RightHandPos.X << endl;
					//fout.close();
					int dxout = dx * m_ScaleFactor;
					int dyout = dy * m_ScaleFactor;
					Xout += dxout;
					Yout += dyout;
					CheckROI(Xout, Yout);
					if (m_ControlStatus)// && ValidControlPosition(joints))
					{
						static bool mousePressed = false;
						if (rightHandState == HandState_Open && mousePressed)
						{
							cout << "----------------------------------" << endl;
							mousePressed = false;
							mouse_event(MOUSEEVENTF_LEFTUP, 0, 0, 0, 0);//
						}
						if (rightHandState == HandState_Closed && !mousePressed)
						{
							cout << "----------------" << endl;
							mousePressed = true;
							mouse_event(MOUSEEVENTF_LEFTDOWN, 0, 0, 0, 0);//
						}
						if (rightHandTrackingState == TrackingState_Tracked)
						{
							mouse_event(MOUSEEVENTF_MOVE, dxout, -dyout, 0, 0);
						}
						else
							cout << "XXXXXXXXXXXXXXXX" << endl;
					}
					prex = x;
					prey = y;
					//------------------------hand state------------------------------
					if (rightHandState == HandState_Unknown)
						rightHandState = preHandState;

					DrawHandState(depthSpacePosition[JointType_HandLeft], leftHandState);
					DrawHandState(depthSpacePosition[JointType_HandRight], rightHandState);

					//cout << "HandState: " << rightHandState << endl;
					//fout << rightHandState << endl;
					//fout.close();

					//---------------------------body-------------------------------
					DrawBone(joints, depthSpacePosition, JointType_Head, JointType_Neck);
					DrawBone(joints, depthSpacePosition, JointType_Neck, JointType_SpineShoulder);
					DrawBone(joints, depthSpacePosition, JointType_SpineShoulder, JointType_SpineMid);
					DrawBone(joints, depthSpacePosition, JointType_SpineMid, JointType_SpineBase);
					DrawBone(joints, depthSpacePosition, JointType_SpineShoulder, JointType_ShoulderRight);
					DrawBone(joints, depthSpacePosition, JointType_SpineShoulder, JointType_ShoulderLeft);
					DrawBone(joints, depthSpacePosition, JointType_SpineBase, JointType_HipRight);
					DrawBone(joints, depthSpacePosition, JointType_SpineBase, JointType_HipLeft);

					// -----------------------Right Arm ------------------------------------ 
					DrawBone(joints, depthSpacePosition, JointType_ShoulderRight, JointType_ElbowRight);
					DrawBone(joints, depthSpacePosition, JointType_ElbowRight, JointType_WristRight);
					DrawBone(joints, depthSpacePosition, JointType_WristRight, JointType_HandRight);
					DrawBone(joints, depthSpacePosition, JointType_HandRight, JointType_HandTipRight);
					DrawBone(joints, depthSpacePosition, JointType_WristRight, JointType_ThumbRight);

					//----------------------------------- Left Arm--------------------------
					DrawBone(joints, depthSpacePosition, JointType_ShoulderLeft, JointType_ElbowLeft);
					DrawBone(joints, depthSpacePosition, JointType_ElbowLeft, JointType_WristLeft);
					DrawBone(joints, depthSpacePosition, JointType_WristLeft, JointType_HandLeft);
					DrawBone(joints, depthSpacePosition, JointType_HandLeft, JointType_HandTipLeft);
					DrawBone(joints, depthSpacePosition, JointType_WristLeft, JointType_ThumbLeft);

					// ----------------------------------Right Leg--------------------------------
					DrawBone(joints, depthSpacePosition, JointType_HipRight, JointType_KneeRight);
					DrawBone(joints, depthSpacePosition, JointType_KneeRight, JointType_AnkleRight);
					DrawBone(joints, depthSpacePosition, JointType_AnkleRight, JointType_FootRight);

					// -----------------------------------Left Leg---------------------------------
					DrawBone(joints, depthSpacePosition, JointType_HipLeft, JointType_KneeLeft);
					DrawBone(joints, depthSpacePosition, JointType_KneeLeft, JointType_AnkleLeft);
					DrawBone(joints, depthSpacePosition, JointType_AnkleLeft, JointType_FootLeft);

					/*if (IsTwoColorSpacePointOverlapping(colorSpacePosition[JointType_Head], colorSpacePosition[JointType_HandLeft])
						|| IsTwoColorSpacePointOverlapping(colorSpacePosition[JointType_Head], colorSpacePosition[JointType_HandRight]))
					{
						if (leave)
						{
							maskType = (maskType + 1) % 4;
							leave = 0;
						}
					}
					else
						leave = 1;
					if (maskType)
					{
						cv::Point maskcenter(colorSpacePosition[JointType_Head].X, colorSpacePosition[JointType_Head].Y);
						cv::Scalar maskcolor;
						switch (maskType)
						{
						case 1:
							maskcolor = cv::Scalar(255, 0, 0);
							break;
						case 2:
							maskcolor = cv::Scalar(0, 255, 0);
							break;
						case 3:
							maskcolor = cv::Scalar(0, 0, 255);
							break;
						}
						cv::circle(m_ColorImg, maskcenter, 200, maskcolor, -1);
					}*/
				}
				delete[] depthSpacePosition;
			}
		}
	}

	//如果只跟踪到一个人，那么以这个人作为控制者
	if (nTracked == 1)
	{
		m_Controller = tempController;
	}

	SafeRelease(pBodyFrame);

	return hr;
	/*Mat resizedImg;
	cv::resize(m_ColorImg, resizedImg, cv::Size(cColorWidth / 2, cColorHeight / 2));
	cv::imshow("color image", resizedImg);
	cv::waitKey(5);*/

}

HRESULT CBodyBasics::ProcessFaces()
{
	HRESULT hr;
	bool bHaveBodyData;
	IBody* ppBodies[BODY_COUNT] = { 0 };

	hr = ProcessBody(BODY_COUNT, ppBodies);
	if (SUCCEEDED(hr))
		bHaveBodyData = true;
	else
		return E_FAIL;

	IInfraredFrame* pInfraredFrame = nullptr;
	UINT16 *infraredData = new UINT16[cDepthHeight*cDepthWidth];
	hr = m_pInfraredFrameReader->AcquireLatestFrame(&pInfraredFrame);
	if (SUCCEEDED(hr))
	{
		hr = pInfraredFrame->CopyFrameDataToArray(cDepthHeight*cDepthWidth, infraredData);
	}
	delete[] infraredData;
	SafeRelease(pInfraredFrame);

	IDepthFrame* pDepthFrame = nullptr;
	UINT16 *depthData = new UINT16[cDepthHeight*cDepthWidth];
	hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
	if (SUCCEEDED(hr))
	{
		hr = pDepthFrame->CopyFrameDataToArray(cDepthHeight*cDepthWidth, depthData);
	}
	SafeRelease(pDepthFrame);

	// iterate through each face reader
	for (int iFace = 0; iFace < BODY_COUNT; ++iFace)
	{
		if (iFace != m_Controller)
			continue;
		// retrieve the latest face frame from this reader
		IFaceFrame* pFaceFrame = nullptr;
		//cout << "acquiring face frame " << iFace << endl;
		//log("acquiring face frame %d", iFace);
		hr = m_pFaceFrameReaders[iFace]->AcquireLatestFrame(&pFaceFrame);

		static float PreEyesMidx = 0.f;
		static float PreEyesMidy = 0.f;
		static float PreFaceCenterx = 0.f;
		static float PreFaceCentery = 0.f;

		float EyesMidx = 0.f;
		float EyesMidy = 0.f;
		float FaceCenterx = 0.f;
		float FaceCentery = 0.f;

		int cEyesMidx = 0;
		int cEyesMidy = 0;
		int cFaceCenterx = 0;
		int cFaceCentery = 0;

		BOOLEAN bFaceTracked = false;
		if (!SUCCEEDED(hr))
		{
			//cout << "acquire face frame failed" << endl;
		}
		if (nullptr == pFaceFrame)
		{
			//cout << "face frame pointer is null" << endl;
		}
		if (SUCCEEDED(hr) && nullptr != pFaceFrame)
		{
			//cout << "face tracking valid ?" << endl;
			// check if a valid face is tracked in this face frame
			hr = pFaceFrame->get_IsTrackingIdValid(&bFaceTracked);
		}

		if (SUCCEEDED(hr))
		{
			if (bFaceTracked)
			{
				m_Detect |= 1 << DetectBit_Face;
				//cout << "face tracked " << iFace << endl;
				IFaceFrameResult* pFaceFrameResult = nullptr;
				RectI faceBox = { 0 };//in color space
				RectI faceBoxf = { 0 };//in infrared space
				PointF facePointsf[FacePointType::FacePointType_Count] = { 0 };//in infrared space
				PointF facePoints[FacePointType::FacePointType_Count] = { 0 };//in color space
				Vector4 faceRotation;
				DetectionResult faceProperties[FaceProperty::FaceProperty_Count];
				//D2D1_POINT_2F faceTextLayout;

				hr = pFaceFrame->get_FaceFrameResult(&pFaceFrameResult);

				// need to verify if pFaceFrameResult contains data before trying to access it
				if (SUCCEEDED(hr) && pFaceFrameResult != nullptr)
				{
					hr = pFaceFrameResult->get_FaceBoundingBoxInInfraredSpace(&faceBoxf);
					//hr = pFaceFrameResult->get_FaceBoundingBoxInColorSpace(&faceBox);

					if (SUCCEEDED(hr))
					{
						Rect rect(faceBoxf.Left, faceBoxf.Top, faceBoxf.Right - faceBoxf.Left, faceBoxf.Bottom - faceBoxf.Top);
						rectangle(m_SkeletonImg, rect, Scalar(0, 0, 255), 2);
						hr = pFaceFrameResult->GetFacePointsInInfraredSpace(FacePointType::FacePointType_Count, facePointsf);
						//hr = pFaceFrameResult->GetFacePointsInColorSpace(FacePointType::FacePointType_Count, facePoints);
					}

					if (SUCCEEDED(hr))
					{
						if (facePointsf[FacePointType_EyeLeft].X == 0 && facePointsf[FacePointType_EyeRight].X == 0)
						{
							continue;
						}
						else
						{
							m_Detect |= ((1 << DetectBit_LeftEye) | (1 << DetectBit_RightEye));
						}

						Point EyeLeftPoint(facePointsf[FacePointType_EyeLeft].X, facePointsf[FacePointType_EyeLeft].Y);
						Point EyeRightPoint(facePointsf[FacePointType_EyeRight].X, facePointsf[FacePointType_EyeRight].Y);						
						int r = (EyeRightPoint.x - EyeLeftPoint.x) >> 2;

						circle(m_SkeletonImg, EyeLeftPoint, r, Scalar(0, 0, 255), 2);
						circle(m_SkeletonImg, EyeRightPoint, r, Scalar(0, 0, 255), 2);

						cEyesMidx = (EyeLeftPoint.x + EyeRightPoint.x) / 2;
						cEyesMidy = (EyeLeftPoint.y + EyeRightPoint.y) / 2;
						cFaceCenterx = (faceBox.Left + faceBox.Right) / 2;
						cFaceCentery = (faceBox.Bottom + faceBox.Top) / 2;

						if (iFace == m_Controller)
						{
							string text = "controller";
							Point mid((EyeLeftPoint.x + EyeRightPoint.x) / 2 - 20, (EyeLeftPoint.y + EyeRightPoint.y) / 2 - 50);
							putText(m_ColorImg, text, mid, CV_FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 0, 255), 2);
						}

						//将鼻子的位置用人脸中心坐标代替
						facePointsf[FacePointType_Nose].X = (faceBoxf.Left + faceBoxf.Right) / 2;
						facePointsf[FacePointType_Nose].Y = (faceBoxf.Top + faceBoxf.Bottom) / 2;

						UINT16 depths[FacePointType_Count];
						DepthSpacePoint depthPoints[FacePointType_Count];
						CameraSpacePoint cameraPoints[FacePointType_Count];
						for (int i = 0; i < FacePointType_Count; i++)
						{
							depthPoints[i].X = facePointsf[i].X;
							depthPoints[i].Y = facePointsf[i].Y;
							int idx = (int)facePointsf[i].Y * cDepthWidth + (int)facePointsf[i].X;
							depths[i] = depthData[idx];
						}
						m_pCoordinateMapper->MapDepthPointsToCameraSpace(FacePointType_Count, depthPoints, FacePointType_Count, depths, FacePointType_Count, cameraPoints);
						
						EyesMidx = (cameraPoints[FacePointType_EyeLeft].X + cameraPoints[FacePointType_EyeRight].X) / 2;
						EyesMidy = (cameraPoints[FacePointType_EyeLeft].Y + cameraPoints[FacePointType_EyeRight].Y) / 2;
						
						FaceCenterx = cameraPoints[FacePointType_Nose].X;
						FaceCentery = cameraPoints[FacePointType_Nose].Y;

						int nEyeLeft[3];
						nEyeLeft[0] = htonl(cameraPoints[FacePointType_EyeLeft].X * 1000);
						nEyeLeft[1] = htonl(cameraPoints[FacePointType_EyeLeft].Y * 1000);
						nEyeLeft[2] = htonl(cameraPoints[FacePointType_EyeLeft].Z * 1000);

						int nEyeRight[3];
						nEyeRight[0] = htonl(cameraPoints[FacePointType_EyeRight].X * 1000);
						nEyeRight[1] = htonl(cameraPoints[FacePointType_EyeRight].Y * 1000);
						nEyeRight[2] = htonl(cameraPoints[FacePointType_EyeRight].Z * 1000);

						int nEyesMid[3];
						nEyesMid[0] = htonl((cameraPoints[FacePointType_EyeLeft].X + cameraPoints[FacePointType_EyeRight].X) * 500);
						nEyesMid[1] = htonl((cameraPoints[FacePointType_EyeLeft].Y + cameraPoints[FacePointType_EyeRight].Y) * 500);
						nEyesMid[2] = htonl((cameraPoints[FacePointType_EyeLeft].Z + cameraPoints[FacePointType_EyeRight].Z) * 500);

						int nFaceMid[3];
						nFaceMid[0] = htonl(cameraPoints[FacePointType_Nose].X * 1000);
						nFaceMid[1] = htonl(cameraPoints[FacePointType_Nose].Y * 1000);
						nFaceMid[2] = htonl(cameraPoints[FacePointType_Nose].Z * 1000);

						float MeterPerPixelOnFace = (cameraPoints[FacePointType_EyeRight].X - cameraPoints[FacePointType_EyeLeft].X) / (facePointsf[FacePointType_EyeRight].X - facePointsf[FacePointType_EyeLeft].X);
						int dFaceHeight = (faceBoxf.Bottom - faceBoxf.Top) * 1000 * MeterPerPixelOnFace;
						int dFaceWidth = (faceBoxf.Right - faceBoxf.Left) * 1000 * MeterPerPixelOnFace;
						int nFaceHeight = htons(dFaceHeight);
						int nFaceWidth = htons(dFaceWidth);

						memcpy(m_Output + Message_LeftEye, nEyeLeft, 12);
						memcpy(m_Output + Message_RightEye, nEyeRight, 12);
						memcpy(m_Output + Message_EyesMid, nEyesMid, 12);
						memcpy(m_Output + Message_FaceMid, nFaceMid, 12);
						memcpy(m_Output + Message_FaceHeight, &nFaceHeight, 2);
						memcpy(m_Output + Message_FaceWidth, &nFaceWidth, 2);

						hr = pFaceFrameResult->get_FaceRotationQuaternion(&faceRotation);
						/*static int outputcount = 0;
						if (outputcount++%10 == 0)
							cout << faceRotation.x << ' ' << faceRotation.y << ' ' << faceRotation.z << ' ' << faceRotation.w << endl;
						*/
						float s2 = faceRotation.y * faceRotation.y + faceRotation.z * faceRotation.z + faceRotation.w * faceRotation.w;
						float factor = 10000 / sqrt(s2);

						int faceNormal[3];
						faceNormal[0] = factor * faceRotation.y;
						faceNormal[1] = factor * faceRotation.z;
						faceNormal[2] = factor * faceRotation.w;

						int nFaceNormal[3];
						nFaceNormal[0] = htonl(faceNormal[0]);
						nFaceNormal[1] = htonl(faceNormal[1]);
						nFaceNormal[2] = htonl(faceNormal[2]);

						memcpy(m_Output + Message_FaceNormal, nFaceNormal, 12);
					}					
				}

				SafeRelease(pFaceFrameResult);
			}
			else
			{
				m_Detect &= ~(1 << DetectBit_Face);
				// face tracking is not valid - attempt to fix the issue
				// a valid body is required to perform this step
				if (bHaveBodyData)
				{
					// check if the corresponding body is tracked 
					// if this is true then update the face frame source to track this body
					IBody* pBody = ppBodies[iFace];
					if (pBody != nullptr)
					{
						BOOLEAN bTracked = false;
						hr = pBody->get_IsTracked(&bTracked);

						UINT64 bodyTId;
						if (SUCCEEDED(hr) && bTracked)
						{
							// get the tracking ID of this body
							hr = pBody->get_TrackingId(&bodyTId);
							if (SUCCEEDED(hr))
							{
								// update the face frame source with the tracking ID
								m_pFaceFrameSources[iFace]->put_TrackingId(bodyTId);
							}
						}
					}
				}
			}
		}
		SafeRelease(pFaceFrame);

		/*if (iFace == m_Controller)
		{
			static int lostTimesFace = 0;
			static int errorTimesEyes = 0;
			static int errorTimesFace = 0;
			static int totalTimes = 0;
			ofstream fout;
			if (!bFaceTracked)
			{
				lostTimesFace++;
			}
			else if (abs(EyesMidx - PreEyesMidx) > 0.1 || abs(EyesMidy - PreEyesMidy) > 0.1)
			{
				errorTimesEyes++;
			}
			else
			{
				fout.open("./EyePosition.txt", ios::out | ios::app);
				fout << EyesMidx << ' ' << EyesMidy << endl;
				fout.close();
			}

			PreEyesMidx = EyesMidx;
			PreEyesMidy = EyesMidy;

			if (abs(FaceCenterx - PreFaceCenterx) > 0.1 || abs(FaceCentery - PreFaceCentery) > 0.1)
			{
				errorTimesFace++;
			}
			else
			{
				fout.open("./FaceCenter.txt", ios::out | ios::app);
				fout << FaceCenterx << ' ' << FaceCentery << endl;
				fout.close();
			}

			PreFaceCenterx = FaceCenterx;
			PreFaceCentery = FaceCentery;

			fout.open("./cEyePosition.txt", ios::out | ios::app);
			fout << cEyesMidx << ' ' << cEyesMidy << endl;
			fout.close();

			fout.open("./cFaceCenter.txt", ios::out | ios::app);
			fout << cFaceCenterx << ' ' << cFaceCentery << endl;
			fout.close();

			totalTimes++;
		}*/
	}

	delete[] depthData;

	if (bHaveBodyData)
	{
		for (int i = 0; i < _countof(ppBodies); ++i)
		{
			//cout << "releasing ppbodies" << endl;
			SafeRelease(ppBodies[i]);
		}
	}
}

void CBodyBasics::ProcessFace(IFaceFrameResult *pFaceFrameResult)
{
	HRESULT hr;
	BOOLEAN bFaceTracked = false;
	RectI boundingBox;
	if (pFaceFrameResult == nullptr)
		return;
	//pFaceFrameResult
	hr = pFaceFrameResult->get_FaceBoundingBoxInColorSpace(&boundingBox);
	if (SUCCEEDED(hr))
	{
		rectangle(m_ColorImg, Rect(boundingBox.Left, boundingBox.Bottom, boundingBox.Right - boundingBox.Left, boundingBox.Top - boundingBox.Bottom), Scalar(0, 0, 255));
	}
}

HRESULT CBodyBasics::GetFaceTextPositionInColorSpace(IBody* pBody, DetectionResult faceProperties[])
{
	string expression[FaceProperty_Count] = {
		"Happy",
		"Engaged",
		"WearingGlasses",
		"LeftEyeClosed",
		"RightEyeClosed",
		"MouthOpen",
		"MouthMoved",
		"LookingAway"
	};
	string dResult[4] = {
		"Unknown",
		"No",
		"Maybe",
		"Yes"
	};
	Point org(500, 500);
	for (int i = 0; i < FaceProperty_Count; i++)
	{
		string result = expression[i] + ":" + dResult[faceProperties[i]];
		putText(m_ColorImg, result, Point(500, 500 + 50 * i), CV_FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0), 2);
	}

	return S_OK;
}

//画手的状态
void CBodyBasics::DrawHandState(const ColorSpacePoint depthSpacePosition, HandState handState)
{
	//给不同的手势分配不同颜色
	CvScalar color;
	switch (handState){
	case HandState_Open:
		color = cvScalar(255, 0, 0);
		break;
	case HandState_Closed:
		color = cvScalar(0, 255, 0);
		break;
	case HandState_Lasso:
		color = cvScalar(0, 0, 255);
		break;
	default://如果没有确定的手势，就不要画
		return;
	}

	circle(m_ColorImg,
		cvPoint(depthSpacePosition.X, depthSpacePosition.Y),
		20, color, -1);
}

void CBodyBasics::DrawHandState(const DepthSpacePoint depthSpacePosition, HandState handState)
{
	//给不同的手势分配不同颜色
	CvScalar color;
	switch (handState){
	case HandState_Open:
		color = cvScalar(255, 0, 0);
		break;
	case HandState_Closed:
		color = cvScalar(0, 255, 0);
		break;
	case HandState_Lasso:
		color = cvScalar(0, 0, 255);
		break;
	default://如果没有确定的手势，就不要画
		return;
	}

	circle(m_SkeletonImg,
		cvPoint(depthSpacePosition.X, depthSpacePosition.Y),
		20, color, -1);
}
/// Draws one bone of a body (joint to joint)
void CBodyBasics::DrawBone(const Joint* pJoints, const ColorSpacePoint* depthSpacePosition, JointType joint0, JointType joint1)
{
	TrackingState joint0State = pJoints[joint0].TrackingState;
	TrackingState joint1State = pJoints[joint1].TrackingState;

	// If we can't find either of these joints, exit
	if ((joint0State == TrackingState_NotTracked) || (joint1State == TrackingState_NotTracked))
	{
		return;
	}

	// Don't draw if both points are inferred
	if ((joint0State == TrackingState_Inferred) && (joint1State == TrackingState_Inferred))
	{
		return;
	}

	CvPoint p1 = cvPoint(depthSpacePosition[joint0].X, depthSpacePosition[joint0].Y),
		p2 = cvPoint(depthSpacePosition[joint1].X, depthSpacePosition[joint1].Y);

	// We assume all drawn bones are inferred unless BOTH joints are tracked
	if ((joint0State == TrackingState_Tracked) && (joint1State == TrackingState_Tracked))
	{
		//非常确定的骨架，用白色直线
	    line(m_ColorImg, p1, p2, cvScalar(255, 255, 255));
	}
	else
	{
		//不确定的骨架，用红色直线
		line(m_ColorImg, p1, p2, cvScalar(0, 0, 255));
	}
}

void CBodyBasics::DrawBone(const Joint* pJoints, const DepthSpacePoint* depthSpacePosition, JointType joint0, JointType joint1)
{
	TrackingState joint0State = pJoints[joint0].TrackingState;
	TrackingState joint1State = pJoints[joint1].TrackingState;

	// If we can't find either of these joints, exit
	if ((joint0State == TrackingState_NotTracked) || (joint1State == TrackingState_NotTracked))
	{
		return;
	}

	// Don't draw if both points are inferred
	if ((joint0State == TrackingState_Inferred) && (joint1State == TrackingState_Inferred))
	{
		return;
	}

	CvPoint p1 = cvPoint(depthSpacePosition[joint0].X, depthSpacePosition[joint0].Y),
		p2 = cvPoint(depthSpacePosition[joint1].X, depthSpacePosition[joint1].Y);

	// We assume all drawn bones are inferred unless BOTH joints are tracked
	if ((joint0State == TrackingState_Tracked) && (joint1State == TrackingState_Tracked))
	{
		//非常确定的骨架，用白色直线
		line(m_SkeletonImg, p1, p2, cvScalar(255, 255, 255));
	}
	else
	{
		//不确定的骨架，用红色直线
		line(m_SkeletonImg, p1, p2, cvScalar(0, 0, 255));
	}
}

void CBodyBasics::ProcessHandPosition(float &x, float &y)
{
	static int selectCounts = 0;
	static float moveThresh = m_MoveThresh;

	float dx = abs(x);
	float dy = abs(y);

	switch (m_MoveMode)
	{
	case MoveMode_Select:
		moveThresh = m_SelectThresh;
		if (dx > m_QuickThresh || dy > m_QuickThresh)
		{
			selectCounts = 0;
			moveThresh = m_QuickThresh;
			m_MoveMode = MoveMode_Quick;
			//std::cout << "leave select" << std::endl;
		}
		break;

	case MoveMode_Move:
		moveThresh = m_MoveThresh;
		if (dx > m_SelectThresh || dy > m_SelectThresh)
		{
			selectCounts = 0;
		}
		else
		{
			//selectCounts++;
			//std::cout << '.';
			if (selectCounts > m_SelectCounts)//2s
			{
				moveThresh = m_SelectThresh;
				m_MoveMode = MoveMode_Select;
				//std::cout << "selected" << std::endl;
			}
		}
		break;

	case MoveMode_Quick:
		moveThresh = m_QuickThresh;
		if (dx < m_QuickThresh || dy < m_QuickThresh)
		{
			moveThresh = m_MoveThresh;
			m_MoveMode = MoveMode_Move;
		}
		break;

	default:
		m_MoveMode = MoveMode_Move;
		//cout << "invalid move mode" << endl;
	}
	////cout << "move mode: " << m_MoveMode << endl;
	////位置未发生明显移动（悬停），2s后锁定坐标
	//if (dx < m_SelectThresh && dy < m_SelectThresh)
	//{
	//	selectCounts++;
	//	std::cout << '.';
	//	if (selectCounts > m_SelectCounts)//2s
	//	{
	//		moveThresh = m_SelectThresh;
	//		m_MoveMode = MoveMode_Select;
	//		std::cout << "selected" << std::endl;
	//	}
	//}
	////位置快速变化，取消锁定
	//else if (dx > m_QuickThresh || dy > m_QuickThresh)
	//{
	//	selectCounts = 0;
	//	moveThresh = m_QuickThresh;
	//	m_MoveMode = MoveMode_Quick;
	//	std::cout << "leave select" << std::endl;
	//}
	//else
	//{
	//	selectCounts = 0;
	//	moveThresh = m_MoveThresh;
	//	m_MoveMode = MoveMode_Move;
	//}
	//位置大于移动阈值，传感器数据发生抖动，滤除该数据
	if (dx > moveThresh || dy > moveThresh)
	{
		x = 0;
		y = 0;
	}

	return;
}

void CBodyBasics::PressKey(vk key)
{
	keybd_event(key, 0, 0, 0);
	Sleep(100);
	keybd_event(key, 0, KEYEVENTF_KEYUP, 0);
}

bool CBodyBasics::IsTwoColorSpacePointOverlapping(const ColorSpacePoint point1, const ColorSpacePoint point2)
{
	return abs(point1.X - point2.X) < overlapThres && abs(point1.Y - point2.Y) < overlapThres;
}

bool CBodyBasics::FinishControl(Joint *joints, HandState LeftHandState, HandState RightHandState)
{
	if (LeftHandState != HandState_Closed || RightHandState != HandState_Closed)
		return false;

	float basePosition = joints[JointType_SpineMid].Position.Y;

	//左右手腕均在脊柱中间之上
	if (joints[JointType_WristLeft].Position.Y > basePosition && joints[JointType_WristRight].Position.Y > basePosition)
		return true;
	else
		return false;
}

bool CBodyBasics::StartControl(Joint *joints, HandState LeftHandState, HandState RightHandState)
{
	if (LeftHandState != HandState_Open || RightHandState != HandState_Open)
		return false;

	float basePosition = joints[JointType_SpineMid].Position.Y;

	//左右手腕均在脊柱中间之上
	if (joints[JointType_WristLeft].Position.Y > basePosition && joints[JointType_WristRight].Position.Y > basePosition)
		return true;
	else
		return false;
}

bool CBodyBasics::ValidControlPosition(Joint *joints)
{
	return joints[JointType_WristRight].Position.Y > joints[JointType_ElbowRight].Position.Y &&
		joints[JointType_WristRight].Position.X > joints[JointType_Head].Position.X;
}

bool CBodyBasics::getControl(Joint *joints)
{
	Joint shoulderleft = joints[JointType_ShoulderLeft];
	Joint shoulderright = joints[JointType_ShoulderRight];
	Joint handleft = joints[JointType_HandLeft];
	Joint handright = joints[JointType_HandRight];

	bool IsleftArmRaised = handleft.Position.Y - shoulderleft.Position.Y > ArmRaisedThreshold;
	bool IsRightArmRaised = handright.Position.Y - shoulderright.Position.Y > ArmRaisedThreshold;

	if (IsleftArmRaised && IsRightArmRaised)
		return true;
	else
		return false;
}

void CBodyBasics::MappingGestureToKeyboard(IBody *pBody)
{
	BOOLEAN bTracked = false;
	HRESULT hr = pBody->get_IsTracked(&bTracked);
	if (!SUCCEEDED(hr) || !bTracked)
		return;

	Joint joints[JointType_Count];
	hr = pBody->GetJoints(_countof(joints), joints);

	Joint head = joints[JointType_Head];
	Joint shoulderleft = joints[JointType_ShoulderLeft];
	Joint shoulderright = joints[JointType_ShoulderRight];
	Joint handleft = joints[JointType_HandLeft];
	Joint handright = joints[JointType_HandRight];

	bool IsleftArmRaised = handleft.Position.Y - shoulderleft.Position.Y > ArmRaisedThreshold;
	bool IsRightArmRaised = handright.Position.Y - shoulderright.Position.Y > ArmRaisedThreshold;

	bool IsleftArmStretched = shoulderleft.Position.X - handleft.Position.X > ArmStretchedThreshold;
	bool IsRightArmStretched = handright.Position.X - shoulderright.Position.X > ArmStretchedThreshold;

	bool IsJump = head.Position.Y - HeadPreviousPosition > JumpDiffThreshold;
	HeadPreviousPosition = head.Position.Y;

	//判断头的位置升高，触发“Up”键
	if (IsJump)
		PressKey(K);

	//判断双手举起，触发“Down”键
	if (IsleftArmRaised && IsRightArmRaised)
		PressKey(S);

	//判断左手展开，触发“Left”键
	if (IsleftArmStretched)
		mouse_event(MOUSEEVENTF_MOVE, -10, 0, 0, 0);
		//PressKey(A);

	//判断右手展开，触发“Right”键
	if (IsRightArmStretched)
		mouse_event(MOUSEEVENTF_MOVE, 10, 0, 0, 0);
		//PressKey(D);
}

HRESULT CBodyBasics::AddROI(Rect rect)
{
	m_roi.push_back(rect);
	return S_OK;
}

HRESULT CBodyBasics::RemoveROI(Rect rect)
{
	if (m_roi.empty())
		return E_FAIL;
	vector<Rect>::iterator iter;
	for (iter = m_roi.begin(); iter != m_roi.end(); ++iter)
	{
		Rect erect = *iter;
		if (erect == rect)
		{
			m_roi.erase(iter);
			return S_OK;
		}
	}
	return E_FAIL;
}

bool CBodyBasics::CheckROI(int &x, int &y)
{
	int size = m_roi.size();
	Point point(x, y);
	if (m_MoveMode == MoveMode_Move)
	{
		for (int i = 0; i < size; i++)
		{
			Rect rect = m_roi[i];
			if (rect.contains(point))
			{
				x = rect.x + rect.width / 2;
				y = rect.y + rect.height / 2;
				return true;
			}
		}
	}
	x = max(0, x);
	x = min(m_ControlSpaceSize.width, x);
	y = max(0, y);
	y = min(m_ControlSpaceSize.height, y);
	return false;
}

char* CBodyBasics::data()
{
	return m_Output;
}

/// Constructor
CBodyBasics::CBodyBasics() :
m_pKinectSensor(nullptr),
m_pCoordinateMapper(nullptr),
m_pBodyFrameReader(nullptr),
m_pColorFrameReader(nullptr),
m_pDepthFrameReader(nullptr),
m_pBodyIndexFrameReader(nullptr),
m_ControlStatus(false),
m_Detect(0),
m_Controller(-1),
m_ScaleFactor(1500)
{
	m_StartTimeStamp = clock();
	short MessageHeader = 0x6c01;
	short MessageLength = 0x6c;
	short nMessageHeader = htons(MessageHeader);
	short nMessageLength = htons(MessageLength);
	memcpy(m_Output + Message_Header, &nMessageHeader, 2);
	memcpy(m_Output + Message_Length, &nMessageLength, 2);
	for (int i = 0; i < BODY_COUNT; i++)
	{
		m_pFaceFrameSources[i] = nullptr;
		m_pFaceFrameReaders[i] = nullptr;
	}
}

/// Destructor
CBodyBasics::~CBodyBasics()
{
	SafeRelease(m_pBodyFrameReader);
	SafeRelease(m_pCoordinateMapper);
	SafeRelease(m_pColorFrameReader);
	SafeRelease(m_pDepthFrameReader);
	SafeRelease(m_pBodyIndexFrameReader);

	for (int i = 0; i < BODY_COUNT; i++)
	{
		SafeRelease(m_pFaceFrameSources[i]);
		SafeRelease(m_pFaceFrameReaders[i]);
	}

	if (m_pKinectSensor)
	{
		m_pKinectSensor->Close();
	}
	SafeRelease(m_pKinectSensor);
}
