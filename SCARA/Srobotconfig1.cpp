#include <iostream>
#include "Srobotconfig.h"
#include "eigen3/Eigen/Dense"
#include <vector>
using namespace std;
using namespace Eigen;
namespace SRobot
{
	//初始化TransMatrix
	double mTransMatrix[16] {0};
	double endpose[6] {500,0,0,0,180,180};
	double angles[4] {0};
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

	Matrix4d R6toSE3(Vector3d v, Vector3d w, double theta)
	{
		// turn deg to rad
		theta = PI * theta / 180;

		Matrix3d so3 = skew(w);
		Matrix3d SO3 = Matrix3d::Identity() + sin(theta) * so3 + (1 - cos(theta)) * so3 * so3;

		// (eye(3) - SO3)* cross(w, v) + (w' * v * theta) * w;

		Vector3d p = (Matrix3d::Identity() - SO3) * (w.cross(v)) + theta * w.transpose() * v * w;
		
		Matrix4d res = Matrix4d::Identity();
		res.block<3, 3>(0, 0) = SO3;
		res.block<3, 1>(0, 3) = p;

		return res;
	}

	Matrix4d inverseSE3(Matrix4d g)
	{
		Matrix4d g_inv = Matrix4d::Identity();
		Vector3d p = g.block<3,1>(0, 3);
		Matrix3d rot = g.block<3, 3>(0, 0);
		g_inv.block<3, 3>(0, 0) = rot.transpose();
		g_inv.block<3, 1>(0, 3) = -rot.transpose() * p;

		return g_inv;
	}

	void subproblem1(Vector3d r, Vector3d p, Vector3d q, Vector3d w, double& theta)
		{
			Vector3d u, v, u_, v_;
			u = p - r;
			v = q - r;

			u_ = u - w * w.transpose() * u;
			v_ = v - w * w.transpose() * v;
			theta = 180 / PI * atan2(w.transpose() * (u_.cross(v_)), u_.transpose() * v_);
		}

	void subproblem3(Vector3d r, Vector3d p, Vector3d q, Vector3d w, double delta, vector<double>& theta)
	{
		Vector3d u = p - r;
		Vector3d v = q - r;
		Vector3d u_ = u - w * w.transpose() * u;
		Vector3d v_ = v - w * w.transpose() * v;
		double delta_2 = pow(delta, 2) - pow(w.transpose() * (p - q), 2) ;  
		double theta0 = atan2(w.transpose() * (u_.cross(v_)), u_.transpose() * v_);
		double u_2 = u_.dot(u_);
		double v_2 = v_.dot(v_);
		double theta_1 = theta0 + acos((u_2 + v_2 - delta_2)/sqrt(4 * u_2 * v_2));
		double theta_2 = theta0 - acos((u_2 + v_2 - delta_2)/sqrt(4 * u_2 * v_2));
		theta.push_back(theta_1 * PI /180);
		theta.push_back(theta_2 * PI /180);
	}
	
	void SetRobotEndPos(double x, double y, double z, double yaw, double pitch, double roll)
	{
		Matrix4d T;
		Matrix3d R;
		Vector3d p;
		p << x, y, z;
		yaw = yaw * PI / 180;
		pitch = pitch * PI / 180;
		roll = roll * PI /180;
		R = AngleAxisd(yaw, Vector3d::UnitZ()).matrix() *
			AngleAxisd(pitch, Vector3d::UnitY()).matrix() *
			AngleAxisd(roll, Vector3d::UnitZ()).matrix();
		T.block<3, 3>(0, 0) = R;
		T.block<3, 1>(0, 3) = p;
		for (int i =1; i < 16; i++)
		{
			mTransMatrix[i] = T(i / 4, i % 4);
		}
		robotBackward(mTransMatrix, mConfig, angles);
		endpose[0] = x;
		endpose[1] = y;
		endpose[2] = z;
		endpose[3] = yaw;
		endpose[4] = pitch;
		endpose[5] = roll;	
	}
	

	void GetJointAngles(double &angle1, double &angle2, double &angle3, double &angle4)
	{
		angle1 = angles[0];
		angle2 = angles[1];
		angle3 = angles[2];
		angle4 = angles[3];
	}
	
	void SetRobotJoint(double angle1, double angle2, double angle3, double angle4)
	{
		angles[0] = angle1;
		angles[1] = angle2;
		angles[2] = angle3;
		angles[3] = angle4;
		robotForward(angles, mTransMatrix, mConfig);
		Matrix4d T;
		T = Matrix4d::Identity();
		for (int i = 0; i < 16; i++)
		{
			T(i/4, i%4) = mTransMatrix[i];
			cout << "test: " << mTransMatrix[i] <<endl;
		}
		Vector3d eulerAngle;
		Matrix3d R;
		R = T.block<3, 3>(0, 0);
		eulerAngle = R.eulerAngles(2,1,2);
		endpose[0] = T(0, 3);
		endpose[1] = T(1, 3);
		endpose[2] = T(2, 3);
		endpose[3] = eulerAngle[0] * 180 / PI;
		endpose[4] = eulerAngle[1] * 180 / PI;
		endpose[5] = eulerAngle[2] * 180 / PI;
	}
	

	void GetJointEndPos(double &x, double &y, double &z, double &yaw, double &pitch, double &roll)
	{
		x = endpose[0];
		y = endpose[1];
		z = endpose[2];
		yaw = endpose[3];
		pitch = endpose[4];
		roll = endpose[5];
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
		Matrix4d gd;
		vector<double> th1, th2, th3, th4;
		th3.push_back(-1 * TransVector[11]);
		for (int i = 0; i < 16; i++)
		{
			gd(i / 4, i % 4) = TransVector[i];
		}
		Matrix4d g0;
		g0 << 1, 0, 0, 500, 
		      0, -1, 0, 0, 
			  0, 0, -1, 0,
			  0, 0, 0, 1; 
		Vector4d p(500, 0, 0, 1);
		Vector4d p_(250, 0, 0, 1);
		Vector4d q(0, 0, 0, 1);
		Vector4d temp_p;
		Vector4d temp_q(0, 0, 0, 1);
		Vector4d t1;
		double delta;
		Matrix4d g3 = Matrix4d::Identity();
		g3(2,3) = gd(2,3);
		temp_p = g3 * p;
		t1 = gd * g0.inverse() * p - q;
		delta = t1.norm();
		Vector3d r(250, 0, 0);
		Vector3d w(0, 0, 1);
		subproblem3(r, temp_p.head(3), q.head(3), w, delta, th2);
		Matrix4d g2;
		g2 = R6toSE3(r, w, th2[0]);
		temp_p = g2 * g3 * p;
		temp_q = gd * g0.inverse() * p;
		double theta0;
		subproblem1(q.head(3), temp_p.head(3), temp_q.head(3), w, theta0);
		th1.emplace_back(theta0);
		Matrix4d g1;
		g1 = R6toSE3(q.head(3), w, th1[0]);
		t1 = g3.inverse() * g2.inverse() * g1.inverse() * gd * g0.inverse() *q;
		double theta4;
		subproblem1(p.head(3), q.head(3), t1.head(3), w, theta4);
		th4.push_back(theta4);
		theta[0] = th1[0];
		theta[1] = th2[0];
		theta[2] = th3[0];
		theta[3] = th4[0];
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
		mConfig = mconfig;
		Matrix4d T;
		Matrix4d g0;
		Matrix4d g1;
		Matrix4d g2;
		Matrix4d g3;
		Matrix4d g4;
		g0 << 1, 0, 0, 500, 
		      0, -1, 0, 0, 
			  0, 0, -1, 0,
			  0, 0, 0, 1; 
		Vector3d w(0, 0, 1);
		Vector3d v1(0,0,0);
		v1 = -1 * w.cross(v1);
		Vector3d v2(250,0,0);
		v2 = -1 * w.cross(v2);
		Vector3d v3(500, 0, 0);
		v3 = w.cross(v3);
		g1 = R6toSE3(v1, w, q[0]);
		g2 = R6toSE3(v2, w, q[1]);
		g3 = Matrix4d::Identity();
		g3(2,3) = -1 * q[2];
		g4 = R6toSE3(v3, -1 * w, q[3]);
        cout << g1;
        cout <<endl;
        cout << g2;
        cout <<endl;        
        cout << g3;
        cout <<endl;        
        cout << g4;
        cout <<endl;
		T = g1 * g2 * g3 * g4 * g0;
		for (int i = 0; i < 4; i++)
		{
			for (int j =0; j < 4; j++)
			{
				TransVector[4 * i + j] = T(i,j);
			}
		}
	}
}
