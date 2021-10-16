
#include "Srobotconfig.h"
#include "eigen3/Eigen/Dense"

using namespace std;

namespace SRobot
{
    SCARA Scara(250,250);
	//初始化TransMatrix
	double mTransMatrix[16] {0};
    double angle[4]{0};
    double pose[6]{0};

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

    Matrix4d R2SE3(Vector3d v, Vector3d w, double theta)
    {
        theta = PI * theta / 180;
        Matrix3d so3 = skew(w);
        Matrix3d SO3 = Matrix3d::Identity() + so3 * sin(theta) + so3 * so3 * (1 - cos(theta));  //Rodrigues Formula
        Vector3d p = (Matrix3d::Identity() - SO3) * skew(w) * v + w * w.transpose() * v * theta;

        Matrix4d SE3 = Matrix4d::Identity();
        SE3.block<3,3>(0,0) = SO3;
        SE3.block<3,1>(0,3) = p;
        return SE3;
    }
	
	
	
	void SetRobotEndPos(double x, double y, double z, double yaw, double pitch, double roll)
	{
        pose[0] = x;
        pose[1] = y;
        pose[2] = z;
        pose[3] = yaw;
        pose[4] = pitch;
        pose[5] = roll;
	}

	void GetJointAngles(double &angle1, double &angle2, double &angle3, double &angle4)
	{
	}

	void SetRobotJoint(double angle1, double angle2, double angle3, double angle4)
	{
        angle[0] = angle1;
        angle[1] = angle2;
        angle[2] = angle3;
        angle[3] = angle4;
	}

	void GetJointEndPos(double &x, double &y, double &z, double &yaw, double &pitch, double &roll)
	{

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
		for(int i=0;i<4;i++){
            T = T * R2SE3(Scara.v.col(i),Scara.w.col(i),q[i]);
        }
        T = T*Scara.G0;
        for (int i=0; i<4; i++){
            for(int j=0; j<4; j++){
                TransVector[4*i+j] = T(i,j);
            }
        }
	}
}
