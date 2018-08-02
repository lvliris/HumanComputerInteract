#include "stdafx.h"
#include "KinectJointFilter.h"

using namespace DirectX;

//Linear interpolation between two float
inline float Lerp(float f1, float f2, float fBlend)
{
	return f1 + (f2 - f1)*fBlend;
}

//if joint is 0 it is not valid
inline bool JointPositionIsValid(XMVECTOR vJointPosition)
{
	return (XMVectorGetX(vJointPosition) != 0.0f ||
		XMVectorGetY(vJointPosition) != 0.0f ||
		XMVectorGetZ(vJointPosition) != 0.0f);
}

TRANSFORM_SMOOTH_PARAMETERS::TRANSFORM_SMOOTH_PARAMETERS():
	fSmoothing(0.25f),
	fCorrection(0.25f),
	fPrediction(0.25f),
	fJitterRadius(0.03f),
	fMaxDeviationRadius(0.05f)
{

}

TRANSFORM_SMOOTH_PARAMETERS::TRANSFORM_SMOOTH_PARAMETERS(float Smoothing, float Correction, float Prediction, float JitterRadius, float MaxDeviationRadius)
{
	fSmoothing = Smoothing;
	fCorrection = Correction;
	fPrediction = Prediction;
	fJitterRadius = JitterRadius;
	fMaxDeviationRadius = MaxDeviationRadius;
}


//--------------------------------------------------------------------------------------
// Implementation of a Holt Double Exponential Smoothing filter. The double exponential
// smooths the curve and predicts.  There is also noise jitter removal. And maximum
// prediction bounds.  The paramaters are commented in the init function.
//--------------------------------------------------------------------------------------
void FilterDoubleExponential::Update(IBody* const pBody)
{
	assert(pBody);

	// Check for divide by zero. Use an epsilon of a 10th of a millimeter
	m_fJitterRadius = XMMax(0.0001f, m_fJitterRadius); //Returns the larger of the two input objects

	TRANSFORM_SMOOTH_PARAMETERS SmoothingParams;

	Joint joints[JointType_Count];

	pBody->GetJoints(_countof(joints), joints);
	for (INT i = 0; i < JointType_Count; i++)
	{
		SmoothingParams.fSmoothing = m_fSmoothing;
		SmoothingParams.fCorrection = m_fCorrection;
		SmoothingParams.fPrediction = m_fPrediction;
		SmoothingParams.fJitterRadius = m_fJitterRadius;
		SmoothingParams.fMaxDeviationRadius = m_fMaxDeviationRadius;

		// If inferred, we smooth a bit more by using a bigger jitter radius
		Joint joint = joints[i];
		if (joint.TrackingState == TrackingState::TrackingState_Inferred)
		{
			SmoothingParams.fJitterRadius *= 2.0f;
			SmoothingParams.fMaxDeviationRadius *= 2.0f;
		}

		Update(joints, i, SmoothingParams);
	}
}

void FilterDoubleExponential::Update(Joint joints[])
{
	// Check for divide by zero. Use an epsilon of a 10th of a millimeter
	m_fJitterRadius = XMMax(0.0001f, m_fJitterRadius);

	TRANSFORM_SMOOTH_PARAMETERS SmoothingParams;
	for (INT i = 0; i < JointType_Count; i++)
	{
		SmoothingParams.fSmoothing = m_fSmoothing;
		SmoothingParams.fCorrection = m_fCorrection;
		SmoothingParams.fPrediction = m_fPrediction;
		SmoothingParams.fJitterRadius = m_fJitterRadius;
		SmoothingParams.fMaxDeviationRadius = m_fMaxDeviationRadius;

		// If inferred, we smooth a bit more by using a bigger jitter radius
		Joint joint = joints[i];
		if (joint.TrackingState == TrackingState::TrackingState_Inferred)
		{
			SmoothingParams.fJitterRadius *= 2.0f;
			SmoothingParams.fMaxDeviationRadius *= 2.0f;
		}

		Update(joints, i, SmoothingParams); // 对每个关节数据分别进行平滑滤波
	}
}

void FilterDoubleExponential::Update(Joint joints[], UINT JointID, TRANSFORM_SMOOTH_PARAMETERS smoothingParams)
{
	XMVECTOR vPrevRawPosition;       // x(t-1)
	XMVECTOR vPrevFilteredPosition;  // 前一期平滑值S(t-1)
	XMVECTOR vPrevTrend;             // 前一期趋势值b(t-1)

	XMVECTOR vRawPosition;           // 实际值x(t)
	XMVECTOR vFilteredPosition;      // 平滑值S(t)
	XMVECTOR vPredictedPosition;     // 预测值F(t+T)
	XMVECTOR vTrend;                 // 趋势值b(t)
	XMVECTOR vDiff;
	XMVECTOR vLength;
	FLOAT fDiff;
	BOOL bJointIsValid;

	const Joint joint = joints[JointID];

	vRawPosition = XMVectorSet(joint.Position.X, joint.Position.Y, joint.Position.Z, 0.0f);

	vPrevFilteredPosition = m_pHistory[JointID].m_vFilteredPosition;    // 前一期平滑值S(t-1)
	vPrevTrend = m_pHistory[JointID].m_vTrend;                          // 前一期趋势值b(t-1)
	vPrevRawPosition = m_pHistory[JointID].m_vRawPosition;              // x(t-1)

	bJointIsValid = JointPositionIsValid(vRawPosition);

	// If joint is invalid, reset the filter
	if (!bJointIsValid)
	{
		m_pHistory[JointID].m_dwFrameCount = 0;
	}

	// Initial start values
	if (m_pHistory[JointID].m_dwFrameCount == 0)
	{
		vFilteredPosition = vRawPosition;
		vTrend = XMVectorZero();
		m_pHistory[JointID].m_dwFrameCount++;
	}
	else if (m_pHistory[JointID].m_dwFrameCount == 1)
	{
		vFilteredPosition = XMVectorScale(XMVectorAdd(vRawPosition, vPrevRawPosition), 0.5f);  //XMVectorScale: Scalar multiplies a vector by a floating-point value
		vDiff = XMVectorSubtract(vFilteredPosition, vPrevFilteredPosition);
		vTrend = XMVectorAdd(XMVectorScale(vDiff, smoothingParams.fCorrection), XMVectorScale(vPrevTrend, 1.0f - smoothingParams.fCorrection));
		m_pHistory[JointID].m_dwFrameCount++;
	}
	else
	{

		// A good filtering solution is usually a combination of various filtering techniques, which may include applying 
		// a jitter removal filter to remove spike noise, a smoothing filter, and a forecasting filter to reduce latency,
		// and then adjusting the outputs based on person kinematics and anatomy to avoid awkward cases caused by overshoot.

		// First apply jitter filter
		vDiff = XMVectorSubtract(vRawPosition, vPrevFilteredPosition);
		vLength = XMVector3Length(vDiff); //Returns a vector. The length of vDiff is replicated into each component
		fDiff = fabs(XMVectorGetX(vLength));
		//fDiff = fabs(XMVectorGetX(vDiff));

		if (fDiff <= smoothingParams.fJitterRadius)
		{
			vFilteredPosition = XMVectorAdd(XMVectorScale(vRawPosition, fDiff / smoothingParams.fJitterRadius),
				XMVectorScale(vPrevFilteredPosition, 1.0f - fDiff / smoothingParams.fJitterRadius));
		}
		else // It should be determined not to be a jitter when the diff value is exceed radius threshold of parameter. In this case, It adopt raw value.
		{
			vFilteredPosition = vRawPosition;
		}

		//if (fDiff >= smoothingParams.fJitterRadius)
		//{
		//	vFilteredPosition = XMVectorAdd(XMVectorScale(vRawPosition, smoothingParams.fJitterRadius),
		//		XMVectorScale(vPrevFilteredPosition, 1.0f - smoothingParams.fJitterRadius));
		//}
		//else // It should be determined not to be a jitter when the diff value is exceed radius threshold of parameter. In this case, It adopt raw value.
		//{
		//	vFilteredPosition = vRawPosition;
		//}


		//////////////////////////////////////////// Now the double exponential smoothing filter:

		// 1. S(t) = α*x(t) + (1-α)*(S(t-1)+b(t-1))     0≤α≤1
		//    S(t) = (1-α)*x(t) + α*(S(t-1)+b(t-1))     0≤α≤1
		// The first smoothing equation adjusts St. This helps to eliminate the lag and brings St to the appropriate base of the current value.
		vFilteredPosition = XMVectorAdd(XMVectorScale(vFilteredPosition, 1.0f - smoothingParams.fSmoothing),
			XMVectorScale(XMVectorAdd(vPrevFilteredPosition, vPrevTrend), smoothingParams.fSmoothing));

		vDiff = XMVectorSubtract(vFilteredPosition, vPrevFilteredPosition);  // S(t)-S(t-1)

		// 2. b(t)= γ * (S(t)-S(t-1)) + (1-γ) * b(t-1)   0≤γ≤1
		// The second smoothing equation then updates the trend, which is expressed as the difference between the last two values.
		vTrend = XMVectorAdd(XMVectorScale(vDiff, smoothingParams.fCorrection), XMVectorScale(vPrevTrend, 1.0f - smoothingParams.fCorrection)); // 修正趋势值
	}

	// 3. F(t+T) = S(t) + b(t)*T    
	vPredictedPosition = XMVectorAdd(vFilteredPosition, XMVectorScale(vTrend, smoothingParams.fPrediction)); // Predict into the future to reduce latency


	// Check that we are not too far away from raw data
	vDiff = XMVectorSubtract(vPredictedPosition, vRawPosition);
	vLength = XMVector3Length(vDiff);
	fDiff = fabs(XMVectorGetX(vLength));

	if (fDiff > smoothingParams.fMaxDeviationRadius)
	{
		vPredictedPosition = XMVectorAdd(XMVectorScale(vPredictedPosition, smoothingParams.fMaxDeviationRadius / fDiff),
			XMVectorScale(vRawPosition, 1.0f - smoothingParams.fMaxDeviationRadius / fDiff));
	}

	// Save the data from this frame
	m_pHistory[JointID].m_vRawPosition = vRawPosition;
	m_pHistory[JointID].m_vFilteredPosition = vFilteredPosition;
	m_pHistory[JointID].m_vTrend = vTrend;

	// Output the data
	m_pFilteredJoints[JointID] = vPredictedPosition;
	m_pFilteredJoints[JointID] = XMVectorSetW(m_pFilteredJoints[JointID], 1.0f);
}

void FilterDoubleExponential::MidFilter(Joint &Input, Joint &Output)
{
	const int MidFilterStride = 5;
	static Joint Joints[MidFilterStride];
	static int InputSeq = 0;

	Joints[InputSeq] = Input;
	InputSeq = InputSeq++ % MidFilterStride;

	float Jointx[MidFilterStride];
	float Jointy[MidFilterStride];
	float Jointz[MidFilterStride];
	for (int i = 0; i < MidFilterStride; i++)
	{
		Jointx[i] = Joints[i].Position.X;
		Jointy[i] = Joints[i].Position.Y;
		Jointz[i] = Joints[i].Position.Z;
	}

	std::sort(Jointx, Jointx + MidFilterStride);
	std::sort(Jointy, Jointy + MidFilterStride);
	std::sort(Jointz, Jointz + MidFilterStride);

	Output.Position.X = Jointx[MidFilterStride / 2];
	Output.Position.Y = Jointy[MidFilterStride / 2];
	Output.Position.Z = Jointz[MidFilterStride / 2];
}