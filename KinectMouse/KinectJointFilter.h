#pragma once
//#define DLL_EXPORT
//#ifdef DLL_EXPORT
//#define DLL_API __declspec(dllexport)
//#else
//#define DLL_API __declspec(dllimport)
//#endif
#include <Windows.h>
#include <Kinect.h>
#include <DirectXMath.h>
#include <queue>
#include <algorithm>

struct TRANSFORM_SMOOTH_PARAMETERS
{
	float fSmoothing;
	float fCorrection;
	float fPrediction;
	float fJitterRadius;
	float fMaxDeviationRadius;
	TRANSFORM_SMOOTH_PARAMETERS();
	TRANSFORM_SMOOTH_PARAMETERS(float Smoothing, float Correction, float Prediction, float JitterRadius, float MaxDeviationRadius);
	//~TRANSFORM_SMOOTH_PARAMETERS();
};

//Holt Double Exponential Smoothing Filter
class FilterDoubleExponentialData
{
public:
	DirectX::XMVECTOR m_vRawPosition;
	DirectX::XMVECTOR m_vFilteredPosition;
	DirectX::XMVECTOR m_vTrend;
	DWORD    m_dwFrameCount;
};

class FilterDoubleExponential
{
public:
	FilterDoubleExponential(){ Init(); }
	~FilterDoubleExponential(){ Shutdown(); }

	void Init(float fSmoothing = 0.25f,
		float fCorrection = 0.25f,
		float fPrediction = 0.25f,
		float fJitterRadius = 0.03f,
		float fMaxDeviationRadius = 0.05f)
	{
		Reset(fSmoothing, fCorrection, fPrediction, fJitterRadius, fMaxDeviationRadius);
	}

	void Shutdown()
	{

	}

	//reset filter when a skeleton is lost
	void Reset(float fSmoothing = 0.25f,
		float fCorrection = 0.25f,
		float fPrediction = 0.25f,
		float fJitterRadius = 0.03f,
		float fMaxDeviationRadius = 0.05f)
	{
		assert(m_pFilteredJoints);
		assert(m_pHistory);

		m_fSmoothing = fSmoothing;
		m_fCorrection = fCorrection;
		m_fJitterRadius = fJitterRadius;
		m_fMaxDeviationRadius = fMaxDeviationRadius;

		memset(m_pFilteredJoints, 0, sizeof(DirectX::XMVECTOR) * JointType_Count);
		memset(m_pHistory, 0, sizeof(FilterDoubleExponentialData) * JointType_Count);

	}

	void Update(IBody* const pBody);
	void Update(Joint joints[]);
	void Update(Joint joints[], UINT JointID, TRANSFORM_SMOOTH_PARAMETERS smoothingParams);
	void MidFilter(Joint &Input, Joint &Output);

	inline const DirectX::XMVECTOR* GetFilterJoints()const{ return &m_pFilteredJoints[0]; }

private:
	DirectX::XMVECTOR m_pFilteredJoints[JointType_Count];
	FilterDoubleExponentialData m_pHistory[JointType_Count];
	FLOAT m_fSmoothing;
	FLOAT m_fCorrection;
	FLOAT m_fPrediction;

	FLOAT m_fJitterRadius;
	FLOAT m_fMaxDeviationRadius;

};