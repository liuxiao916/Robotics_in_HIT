#include <iostream>
#include <fstream>
#include "MotionPlan.h"
#include "Srobotconfig.h"
#include <algorithm>
#include "eigen3/Eigen/Dense"
#include <iomanip>

using namespace std;
using namespace SRobot;
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

	double angle1, angle2, angle3, angle4;
	SRobot::SetRobotEndPos(startPos.x, startPos.y, startPos.z, startPos.yaw, startPos.pitch,  startPos.roll);
	SRobot::GetJointAngles(angle1, angle2, angle3, angle4);

	mJointAngleBegin[0] = angle1;
	mJointAngleBegin[1] = angle2;
	mJointAngleBegin[2] = angle3;
	mJointAngleBegin[3] = angle4;

	SRobot::SetRobotEndPos(endPos.x, endPos.y, endPos.z, endPos.yaw, endPos.pitch, endPos.roll);
	SRobot::GetJointAngles(angle1, angle2, angle3, angle4);
	mJointAngleEnd[0] = angle1;
	mJointAngleEnd[1] = angle2;
	mJointAngleEnd[2] = angle3;
	mJointAngleEnd[3] = angle4;

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
	outfile <<setiosflags(ios::fixed)<<setprecision(4)<< mJointAngleBegin[0] << "  "
			<< mJointAngleBegin[1] << "  "
			<< mJointAngleBegin[2] << "  "
			<< mJointAngleBegin[3] << "  "
			<< 0 << "  " << 0 << "  ";
	outfile << endl;	// 保存初始的时间4个关节角度

	double RotateAngle[4];	// 定义关节旋转角
	// 定义总时间,加速度时间,匀速时间,减速时间
	double time[4], time1[4], time2[4], time3[4];
	int N[4];			// 定义采样点数
	int Nmax = 0;	// 定义最大采样点数 

	for (int i = 0; i < 4; i++){
		double vtemp;
		RotateAngle[i] = mJointAngleEnd[i] - mJointAngleBegin[i];
		// cout << RotateAngle[i] << endl;

		if (RotateAngle[i] > 0){
			if (RotateAngle[i] > ((mVel * mVel) / (2 * mAcc) + (mVel * mVel) / (2 * mDec))){
				time1[i] = mVel / mAcc;
				time3[i] = mVel / mDec;
				time2[i] = (RotateAngle[i] - ((mVel * mVel) / (2 * mAcc) + (mVel * mVel) / (2 * mDec))) / mVel;
				time[i] = time1[i] + time2[i] + time3[i];
			}
			else{
				vtemp = abs(sqrt((2 * mAcc * mDec * RotateAngle[i]) / (mAcc + mDec)));
				time1[i] = vtemp / mAcc;
				time3[i] = vtemp / mDec;
				time2[i] = 0;
				time[i] = time1[i] + time2[i] + time3[i];
			}
		}
		else if (RotateAngle[i] < 0){
			if (-RotateAngle[i] > ((mVel * mVel) / (2 * mAcc) + (mVel * mVel) / (2 * mDec))){
				time1[i] = mVel / mDec;
				time3[i] = mVel / mAcc;
				time2[i] = (-RotateAngle[i] - ((mVel * mVel) / (2 * mAcc) + (mVel * mVel) / (2 * mDec))) / mVel;
				time[i] = time1[i] + time2[i] + time3[i];
			}
			else{
				vtemp = abs(sqrt((2 * mAcc * mDec * -RotateAngle[i]) / (mAcc + mDec)));
				time1[i] = vtemp / mDec;
				time3[i] = vtemp / mAcc;
				time2[i] = 0;
				time[i] = time1[i] + time2[i] + time3[i];
			}
		}
		else if (0 == RotateAngle[i]){
			time1[i] = 0;
			time2[i] = 0;
			time3[i] = 0;
			time[i] = 0;
		}

		N[i] = (int)(time[i] / mSampleTime);
		if (N[i] > Nmax){
			Nmax = N[i];
		}
	}

	/*cout << time[0] << "   " << time[1] << "   " << time[2] << "   " << time[3] << "   " << endl;
	cout << N[0]<<"   "<<N[1]<<"   "<<N[2]<<"   "<<N[3]<<"   "<<Nmax << endl;*/

	double lastAngle[4];
	double nowAngle[4];
	double nowV[4] = { 0 };
	double lastV[4] = { 0 };

	for (int i = 0; i < 4; i++) {
		lastAngle[i] = mJointAngleBegin[i];
	}

	for (int j = 1; j <= Nmax; j++){
		for (int i = 0; i < 4; i++){
			if (j <= N[i]){
				if (RotateAngle[i] > 0){
					if (time2[i] > 0){
						if (j <= (time1[i] / mSampleTime)){
							nowV[i] = lastV[i] + mAcc * mSampleTime;
							nowAngle[i] = lastAngle[i] + (nowV[i] + lastV[i]) * mSampleTime / 2;
						}
						else if (j > (time1[i] / mSampleTime) && j < ((time1[i] + time2[i]) / mSampleTime)){
							nowV[i] = lastV[i];
							nowAngle[i] = lastAngle[i] + nowV[i] * mSampleTime;
						}
						else if (j > ((time1[i] + time2[i]) / mSampleTime) && j <= (time[i] / mSampleTime)){
							nowV[i] = lastV[i] - mDec * mSampleTime;
							nowAngle[i] = lastAngle[i] + nowV[i] * mSampleTime;
						}
						else if (j > time[i] / mSampleTime){
							nowAngle[i] = lastAngle[i];
						}
					}
					else if (time2[i] == 0){
						if (j <= (time1[i] / mSampleTime)){
							nowV[i] = lastV[i] + mAcc * mSampleTime;
							nowAngle[i] = lastAngle[i] + (nowV[i] + lastV[i]) * mSampleTime / 2;
						}
						else if (j > (time1[i] / mSampleTime) && j <= (time[i] / mSampleTime)){
							nowV[i] = lastV[i] - mDec * mSampleTime;
							nowAngle[i] = lastAngle[i] + (nowV[i] + lastV[i]) * mSampleTime / 2;
						}
						else if (j > time[i] / mSampleTime){
							nowAngle[i] = lastAngle[i];
						}
					}
				}
				else if (RotateAngle[i] < 0){
					if (time2[i] > 0){
						if (j <= (time1[i] / mSampleTime)){
							nowV[i] = lastV[i] - mDec * mSampleTime;
							nowAngle[i] = lastAngle[i] + (nowV[i] + lastV[i]) * mSampleTime / 2;
						}
						else if (j > (time1[i] / mSampleTime) && j < ((time1[i] + time2[i]) / mSampleTime)){
							nowV[i] = lastV[i];
							nowAngle[i] = lastAngle[i] + nowV[i] * mSampleTime;
						}
						else if (j > ((time1[i] + time2[i]) / mSampleTime) && j <= (time[i] / mSampleTime)){
							nowV[i] = lastV[i] + mAcc * mSampleTime;
							nowAngle[i] = lastAngle[i] + (nowV[i] + lastV[i]) * mSampleTime / 2;
						}
						else if (j > time[i] / mSampleTime){
							nowAngle[i] = lastAngle[i];
						}
					}
					else if (time2[i] == 0){
						if (j <= (time1[i] / mSampleTime)){
							nowV[i] = lastV[i] - mDec * mSampleTime;
							nowAngle[i] = lastAngle[i] + (nowV[i] + lastV[i]) * mSampleTime / 2;
						}
						else if (j > (time1[i] / mSampleTime) && j <= (time[i] / mSampleTime)){
							nowV[i] = lastV[i] + mAcc * mSampleTime;
							nowAngle[i] = lastAngle[i] + (nowV[i] + lastV[i]) * mSampleTime / 2;
						}
						else if (j > time[i] / mSampleTime){
							nowAngle[i] = lastAngle[i];
						}
					}
				}
				else if (RotateAngle[i] == 0){
					nowAngle[i] = lastAngle[i];
				}
			}
			else if (j > N[i]){
				nowAngle[i] = lastAngle[i];
			}
			lastV[i] = nowV[i];
			lastAngle[i] = nowAngle[i];
		}

		// cout << nowAngle[0] << "    " << nowAngle[1] << "  " << nowAngle[2] << "   " << nowAngle[3] << endl;
		outfile << setiosflags(ios::fixed) << setprecision(4) << nowAngle[0] << "  "
			<< nowAngle[1] << "  "
			<< nowAngle[2] << "  "
			<< nowAngle[3] << "  "
			<< 0 << "  " << 0 << "  ";
		outfile << endl;
	}

}

void CHLMotionPlan::GetPlanPoints_line()
{
	//完成代码
    double length;
	double startX, startY, startZ, startYaw, startPitch, startRoll;
	double endX, endY, endZ, endYaw, endPitch, endRoll;

	startX = mStartMatrixData[3];
	startY = mStartMatrixData[7];
	startZ = mStartMatrixData[11];
	startYaw = 0;
	startPitch = 180;
	startRoll = acos(-1 * mStartMatrixData[0]) * 180 / PI;
	// cout << startX << "   " << startY << "   " << startZ << "   " << startYaw << "   " << startPitch << "   " << startRoll << endl;
	
	endX = mEndMatrixData[3];
	endY = mEndMatrixData[7];
	endZ = mEndMatrixData[11];
	endYaw = 0;
	endPitch = 180;
	endRoll = acos(-1 * mEndMatrixData[0]) * 180 / PI;
	//cout << endX << "   " << endY << "   " << endZ << "   " << endYaw << "    " << endPitch << "    " << endRoll << endl;

	double deltaX, deltaY, deltaZ;
	deltaX = abs(startX - endX);
	deltaY = abs(startY - endY);
	deltaZ = abs(startZ - endZ);
	length = abs(sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ));
	//cout << length << endl;

	ofstream outfile;		// 创建文件
	outfile.open("mydata.txt");
	outfile << setiosflags(ios::fixed) << setprecision(4) << startX << "  "
		<< startY << "  "
		<< startZ << "  "
		<< startYaw << "  "
		<< startPitch << "  "
		<< startRoll << "  ";
	outfile << endl;

	double time1, time2, time3, time;	// 定义加速时间, 匀速时间, 减速时间, 总时间
	if (length > ((mVel * mVel) / (2 * mAcc) + (mVel * mVel) / (2 * mDec))){
		time1 = mVel / mAcc;
		time3 = mVel / mDec;
		time2 = (length - ((mVel * mVel) / (2 * mAcc) + (mVel * mVel) / (2 * mDec))) / mVel;
		time = time1 + time2 + time3;
	}
	else if (length <= ((mVel * mVel) / (2 * mAcc) + (mVel * mVel) / (2 * mDec))){
		double vtemp;
		vtemp = abs(sqrt((2 * mAcc * mDec * length) / (mAcc + mDec)));
		time1 = vtemp / mAcc;
		time2 = 0;
		time3 = vtemp / mDec;
		time = time1 + time2 + time3;
	}
	else if (length == 0){
		time1 = 0;
		time2 = 0;
		time3 = 0;
		time = 0;
	}
	// cout << time << endl;

	int N = (int)(time / mSampleTime);	// 定义需要点位的个数
	double nowX, nowY, nowZ, nowYaw, nowPitch, nowRoll;
	double lastX, lastY, lastZ, lastYaw, lastPitch, lastRoll;
	double nowV, lastV, tempLength;
	// 初始化设置
	nowX = lastX = startX;
	nowY = lastY = startY;
	nowZ = lastZ = startZ;
	nowYaw = lastYaw = startYaw;
	nowPitch = lastPitch = startPitch;
	nowRoll = lastRoll = startRoll;
	nowV = 0;
	lastV = 0;

	for (int i = 1; i <= N; i++){
		if (time2 > 0){			// 匀速段大于0
			if (i <= time1 / mSampleTime){
				nowV = lastV + mAcc * mSampleTime;
				tempLength = (nowV + lastV) * mSampleTime / 2;
			}
			else if (i > time1 / mSampleTime && i < (time1 + time2) / mSampleTime){
				nowV = lastV;
				tempLength = nowV * mSampleTime;
			}
			else if (i > (time1 + time2) / mSampleTime && i <= time / mSampleTime){
				nowV = lastV - mDec * mSampleTime;
				tempLength = (nowV + lastV) * mSampleTime / 2;
			}
			else if (i > time / mSampleTime){
				tempLength = 0;
			}
		}
		else if (0 == time2){	// 匀速段等于0
			if (i <= time1 / mSampleTime){
				nowV = lastV + mAcc * mSampleTime;
				tempLength = (nowV + lastV) * mSampleTime / 2;
			}
			else if (i > time1 / mSampleTime && i < time / mSampleTime){
				nowV = lastV - mDec * mSampleTime;
				tempLength = (nowV + lastV) * mSampleTime / 2;
			}
			else if (i > time / mSampleTime){
				tempLength = 0;
			}
		}

		nowX = lastX + tempLength / length * (endX - startX);
		nowY = lastY + tempLength / length * (endY - startY);
		nowZ = lastZ + tempLength / length * (endZ - startZ);
		nowYaw = lastYaw + tempLength / length * (endYaw - startYaw);
		nowPitch = lastPitch + tempLength / length * (endPitch - startPitch);
		nowRoll = lastRoll + tempLength / length * (endRoll - startRoll);

		outfile << setiosflags(ios::fixed) << setprecision(4) << nowX << "  "
				<< nowY << "  "
				<< nowZ << "  "
				<< nowYaw << "  "
				<< nowPitch << "  "
				<< nowRoll << "  ";
		outfile << endl;

		lastV = nowV;
		lastX = nowX;
		lastY = nowY;
		lastZ = nowZ;
		lastYaw = nowYaw;
		lastPitch = nowPitch;
		lastRoll = nowRoll;
	}
}