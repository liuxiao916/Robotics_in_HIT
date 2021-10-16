﻿
#include "Srobotconfig.h"
#include <Eigen/Dense>
#include <iostream>

using namespace std;

namespace SRobot
{
    SCARA Scara(250,250);
	//初始化TransMatrix
	double mTransMatrix[16]={0};
    double mangle[4]={0};
    double mpose[6]={0};

	//只使用一种姿态
	bool mConfig = 1;

    Matrix3d skew(Vector3d w)
    {   
        Matrix3d so3;
        so3 << 0, -w(2), w(1),
            w(2), 0, -w(0),
            -w(1), w(0), 0;
        return so3;
    }

    Matrix4d R2SE3(Vector3d v, Vector3d w, int i, double theta)
    {
        Matrix4d SE3 = Matrix4d::Identity();
        if (i == 2)
        {
            SE3(2,3) = -theta;
        }
        else
        {
            theta = PI * theta / 180;
            Matrix3d so3 = skew(w);
            Matrix3d SO3 = Matrix3d::Identity() + so3 * sin(theta) + so3 * so3 * (1 - cos(theta));  //Rodrigues Formula
            Vector3d p = (Matrix3d::Identity() - SO3) * skew(w) * v + w * w.transpose() * v * theta;
            SE3.block<3,3>(0,0) = SO3;
            SE3.block<3,1>(0,3) = p;
        }
        return SE3;
    }
	
	
	
	void SetRobotEndPos(double x, double y, double z, double yaw, double pitch, double roll)
	{
        mpose[0] = x;
        mpose[1] = y;
        mpose[2] = z;
        mpose[3] = yaw;
        mpose[4] = pitch;
        mpose[5] = roll;
	}

	void GetJointAngles(double &angle1, double &angle2, double &angle3, double &angle4)
	{
	}

	void SetRobotJoint(double angle1, double angle2, double angle3, double angle4)
	{
        mangle[0] = angle1;
        mangle[1] = angle2;
        mangle[2] = angle3;
        mangle[3] = angle4;
        Matrix3d R,R1,R2,R4;
        R << cos((angle1+angle2+angle4)/180*PI), -sin((angle1+angle2+angle4)/180*PI), 0,
             sin((angle1+angle2+angle4)/180*PI),cos((angle1+angle2+angle4)/180*PI),0,
             0,0,1;
        // R1 << cos(angle1/180*PI), -sin(angle1/180*PI), 0,
        //     sin(angle1/180*PI),cos(angle1/180*PI),0,
        //     0,0,1;
        // R2 << cos(angle2/180*PI), -sin(angle2/180*PI), 0,
        //     sin(angle2/180*PI),cos(angle2/180*PI),0,
        //     0,0,1;
        // R4 << cos(angle4/180*PI), -sin(angle4/180*PI), 0,
        //     sin(angle4/180*PI),cos(angle4/180*PI),0,
        //     0,0,1;
        cout<<R;
        cout<<endl;
        // cout<<R2;
        // cout<<endl;
        // cout<<R4;
        // cout<<endl;
	}

	void GetJointEndPos(double &x, double &y, double &z, double &yaw, double &pitch, double &roll)
	{
        Matrix4d T;
        Matrix3d R;
        Vector3d Euler;
        robotForward(mangle, mTransMatrix, mConfig);
        for (int i=0; i<4; i++){
            for(int j=0; j<4; j++){
                T(i,j) = mTransMatrix[4*i+j];
            }
        }
        x = T(0,3);
        y = T(1,3);
        z = T(2,3);
        R = T.block<3,3>(0,0);
        Euler = R.eulerAngles(2,1,2);

        double tmp = sqrt(R(2, 0) * R(2, 0) + R(2, 1) * R(2, 1));
		yaw = atan2(R(1, 2), R(0, 2)) * 180 / PI;
		pitch = atan2(tmp, R(2, 2)) * 180 / PI;
		roll = atan2(R(2, 1), -R(2, 0)) * 180 / PI;

        // std::cout << x;
        // std::cout << std::endl;
        // std::cout << y;
        // std::cout << std::endl;
        // std::cout << z;
        std::cout<<R;
        std::cout << std::endl;
        std::cout << Euler/3.14*180; 
        // std::cout << std::endl;
        // std::cout << yaw;
        // std::cout << std::endl;
        // std::cout << pitch;
        // std::cout << std::endl;
        // std::cout << roll;
        // std::cout << std::endl;

	}


	/********************************************************************
	ABSTRACT:	机器人逆运动学

	INPUTS:		T[16]:	位姿矩阵，其中长度距离为米

				config：姿态，六轴机器人对应有8种姿态（即对应的逆运动学8个解），
				Scara机器人有2个解，Delta机器人有一个解，
				为了安全，实验室中我们只计算一种即可。config用来作为选解的标志数。

	OUTPUTS:    theta[6] 6个关节角, 单位为弧度

	RETURN:		<none>
	***********************************************************************/
	void robotBackward(const double* TransVector, bool mconfig, double* theta)
	{
		
	}

	/********************************************************************
	ABSTRACT:	机器人正运动学
	
	INPUTS:		q[6]: 6个关节角, 单位为弧度
	
	OUTPUTS:	config用来作为选解的标志数。

				TransVector[16] : 刚体变换矩阵，也就是末端的位姿描述，其中长度距离为米
	
	RETURN:		<none>
	***********************************************************************/
	void robotForward(const double* q, double* TransVector, bool mconfig)
	{	
        Matrix4d T=Matrix4d::Identity();
        cout<<Scara.G0;
        cout<<endl;
		for(int i=0;i<4;i++){
            T = T * R2SE3(Scara.v.col(i),Scara.w.col(i),i,q[i]);
            //cout<<R2SE3(Scara.v.col(i),Scara.w.col(i),i,q[i])<<endl;
        }
        T = T*Scara.G0;
        for (int i=0; i<4; i++){
            for(int j=0; j<4; j++){
                TransVector[4*i+j] = T(i,j);
            }
        }
	}
}
