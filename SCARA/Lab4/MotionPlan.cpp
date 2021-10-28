#include <iostream>
#include <fstream>
#include "MotionPlan.h"
#include "HLrobotconfig.h"
#include <algorithm>
#include <Windows.h>
#include "eigen3/Eigen/Dense"

using namespace std;
using namespace HLRobot;
using namespace Eigen;

/********************************************************************
ABSTRACT:	构造函数

INPUTS:		<none>

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/

CHLMotionPlan::CHLMotionPlan()
{
	for (int i = 0; i < 6; i++)
	{
		mJointAngleBegin[i] = 0;
		mJointAngleEnd[i] = 0;
	}

	for (int i = 0; i < 16; i++)
	{
		mStartMatrixData[i] = 0;
		mEndMatrixData[i] = 0;
	}

	mSampleTime = 0.001;
	mVel = 0;
	mAcc = 0;
	mDec = 0;
}

/********************************************************************
ABSTRACT:	析构函数

INPUTS:		<none>

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
CHLMotionPlan::~CHLMotionPlan()
{

}

/********************************************************************
ABSTRACT:	设置采样时间

INPUTS:		sampleTime			采样时间，单位S

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
void CHLMotionPlan::SetSampleTime(double sampleTime)
{
	if (sampleTime < 0.001)
	{
		mSampleTime = 0.001;
	}
	else
	{
		mSampleTime = sampleTime;
	}
}

/********************************************************************
ABSTRACT:	设置运动参数

INPUTS:		vel			速度，单位m/s
			acc			加速度，单位m/s/s
			dec			减速度，单位m / s / s

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
void CHLMotionPlan::SetProfile(double vel, double acc, double dec)
{
	mVel = vel;
	mAcc = acc;
	mDec = dec;
}

/********************************************************************
ABSTRACT:	设置规划的起始单位和技术点位

INPUTS:		startPos			起始点位笛卡尔坐标
			endPos				结束点位笛卡尔坐标

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
void CHLMotionPlan::SetPlanPoints(PosStruct startPos, PosStruct endPos)
{
	double startAngle[3], endAngle[3];

	startAngle[0] = startPos.yaw * PI / 180;
	startAngle[1] = startPos.pitch * PI / 180;
	startAngle[2] = startPos.roll * PI / 180;

	endAngle[0] = endPos.yaw * PI / 180;
	endAngle[1] = endPos.pitch * PI / 180;
	endAngle[2] = endPos.roll * PI / 180;

	mStartMatrixData[0] = cos(startAngle[0])*cos(startAngle[1])*cos(startAngle[2]) - sin(startAngle[0])*sin(startAngle[2]);
	mStartMatrixData[1] = -cos(startAngle[0])*cos(startAngle[1])*sin(startAngle[2]) - sin(startAngle[0])*cos(startAngle[2]);
	mStartMatrixData[2] = cos(startAngle[0])*sin(startAngle[1]);
	mStartMatrixData[3] = startPos.x / 1000;

	mStartMatrixData[4] = sin(startAngle[0])*cos(startAngle[1])*cos(startAngle[2]) + cos(startAngle[0])*sin(startAngle[2]);
	mStartMatrixData[5] = -sin(startAngle[0])*cos(startAngle[1])*sin(startAngle[2]) + cos(startAngle[0])*cos(startAngle[2]);
	mStartMatrixData[6] = sin(startAngle[0])*sin(startAngle[1]);
	mStartMatrixData[7] = startPos.y / 1000;

	mStartMatrixData[8] = -sin(startAngle[1])*cos(startAngle[2]);
	mStartMatrixData[9] = sin(startAngle[1])*sin(startAngle[2]);
	mStartMatrixData[10] = cos(startAngle[1]);
	mStartMatrixData[11] = startPos.z / 1000;

	mStartMatrixData[12] = 0;
	mStartMatrixData[13] = 0;
	mStartMatrixData[14] = 0;
	mStartMatrixData[15] = 1;

	mEndMatrixData[0] = cos(endAngle[0])*cos(endAngle[1])*cos(endAngle[2]) - sin(endAngle[0])*sin(endAngle[2]);
	mEndMatrixData[1] = -cos(endAngle[0])*cos(endAngle[1])*sin(endAngle[2]) - sin(endAngle[0])*cos(endAngle[2]);
	mEndMatrixData[2] = cos(endAngle[0])*sin(endAngle[1]);
	mEndMatrixData[3] = endPos.x / 1000;

	mEndMatrixData[4] = sin(endAngle[0])*cos(endAngle[1])*cos(endAngle[2]) + cos(endAngle[0])*sin(endAngle[2]);
	mEndMatrixData[5] = -sin(endAngle[0])*cos(endAngle[1])*sin(endAngle[2]) + cos(endAngle[0])*cos(endAngle[2]);
	mEndMatrixData[6] = sin(endAngle[0])*sin(endAngle[1]);
	mEndMatrixData[7] = endPos.y / 1000;

	mEndMatrixData[8] = -sin(endAngle[1])*cos(endAngle[2]);
	mEndMatrixData[9] = sin(endAngle[1])*sin(endAngle[2]);
	mEndMatrixData[10] = cos(endAngle[1]);
	mEndMatrixData[11] = endPos.z / 1000;

	mEndMatrixData[12] = 0;
	mEndMatrixData[13] = 0;
	mEndMatrixData[14] = 0;
	mEndMatrixData[15] = 1;

	double angle1, angle2, angle3, angle4, angle5, angle6;
	HLRobot::SetRobotEndPos(startPos.x, startPos.y, startPos.z, startPos.yaw, startPos.pitch, startPos.roll, startPos.config);
	HLRobot::GetJointAngles(angle1, angle2, angle3, angle4, angle5, angle6);

	mJointAngleBegin[0] = angle1;
	mJointAngleBegin[1] = angle2;
	mJointAngleBegin[2] = angle3;
	mJointAngleBegin[3] = angle4;
	mJointAngleBegin[4] = angle5;
	mJointAngleBegin[5] = angle6;

	HLRobot::SetRobotEndPos(endPos.x, endPos.y, endPos.z, endPos.yaw, endPos.pitch, endPos.roll, endPos.config);
	HLRobot::GetJointAngles(angle1, angle2, angle3, angle4, angle5, angle6);
	mJointAngleEnd[0] = angle1;
	mJointAngleEnd[1] = angle2;
	mJointAngleEnd[2] = angle3;
	mJointAngleEnd[3] = angle4;
	mJointAngleEnd[4] = angle5;
	mJointAngleEnd[5] = angle6;

}

/********************************************************************
ABSTRACT:	运动轨迹规划部分（以关节空间为例）

INPUTS:		pos						二维位置向量

OUTPUTS:	pos						二维位置向量（第一维：位置个数，第二维：每个轴的关节角度，单位弧度）

RETURN:		<none>
***********************************************************************/

/******
 * 参考步骤
 * 步骤1：创建文件并写入初始角度
 * 步骤2：计算每个轴旋转的角度
 * 步骤3：计算每个轴移动到终止点所需要时间
 * 步骤4：根据采样时间计算离散点位数
 * 步骤5：根据离散点位数得到每刻的关节角数值
 * 步骤6：将每时刻的关节角数值写入到文件
 */
void CHLMotionPlan::GetPlanPoints()
{
	ofstream outfile;               			//创建文件
	outfile.open("data.txt");
	outfile << mJointAngleBegin[0] << "  "
			<< mJointAngleBegin[1] << "  "
			<< mJointAngleBegin[2] << "  "
			<< mJointAngleBegin[3] << "  "
			<< mJointAngleBegin[4] << "  "
			<< mJointAngleBegin[5] << "  ";
	outfile << endl;//保存初始的时间、六个关节角度

	//完成代码

}

void CHLMotionPlan::GetPlanPoints_line()
{
	//完成代码
}