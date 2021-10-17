
#include "Srobotconfig.h"
#include <Eigen/Dense>
#include <iostream>
#include <vector>

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
	
    void Subproblem1 (Vector3d r, Vector3d p, Vector3d q, Vector3d w, double &theta)
    {
        Vector3d u,v,u_dot,v_dot;
        u = p - r;
        v = q - r;

        u_dot = u - w * w.transpose() * u;
        v_dot = v - w * w.transpose() * v;

        theta = atan2(w.transpose()*(u_dot.cross(v_dot)),u_dot.transpose()*v_dot) * 180 / PI;
    }

    //SCRAR没有用 写一下当练习
    void Subproblem2 (Vector3d r, Vector3d p, Vector3d q, Vector3d w1, Vector3d w2, vector<double> &theta1, vector<double> &theta2 )
    {
        Vector3d u,v;
        u = p - r;
        v = q - r;

        double w12 = w1.transpose()*w2;
        double w2u = w2.transpose()*u;
        double w1v = w1.transpose()*v;
        double alpha = (w12*w2u - w1v) / (w12*w12 -1);
        double beta = (w12*w1v-w2u) / (w12*w12 -1);

        double u_norm_2 = u.transpose()*u;
        Vector3d w1_cross_w2 = w1.cross(w2);
        double Gamma_2 = (u_norm_2 - alpha*alpha -beta*beta -2*alpha*beta*w12) / (w1_cross_w2).dot(w1_cross_w2);

        if(Gamma_2>=0){
            double Gamma = sqrt(Gamma_2);
            Vector3d z1 = alpha * w1 + beta * w2 + Gamma * w1_cross_w2;
            Vector3d z2 = alpha * w1 + beta * w2 - Gamma * w1_cross_w2;
            Vector3d c1 = z1 + r;
            Vector3d c2 = z2 + r;

            double theta1_temp, theta2_temp;
            Subproblem1(r, q, c1, w1, theta1_temp);
            theta1.push_back(-theta1_temp);
            Subproblem1(r, q, c2, w1, theta1_temp);
            theta1.push_back(-theta1_temp);

            Subproblem1(r, p, c1, w2, theta2_temp);
            theta2.push_back(theta2_temp);
            Subproblem1(r, p, c2, w2, theta2_temp);
            theta2.push_back(theta2_temp);
        }
    }
   void Subproblem3 (Vector3d r, Vector3d p, Vector3d q, Vector3d w, double delta, vector<double> &theta)
    {
        Vector3d u,v,u_dot,v_dot;
        u = p - r;
        v = q - r;
        u_dot = u - w * w.transpose() * u;
        v_dot = v - w * w.transpose() * v;
        double delta_dot_2 = delta*delta - (w.dot(p-q))*(w.dot(p-q));   //maybe bug

        double theta0 = atan2(w.dot(u_dot.cross(v_dot)),u_dot.dot(v_dot));

        double Phi = (u_dot.dot(u_dot) + v_dot.dot(v_dot) -delta_dot_2)/sqrt(4*u_dot.dot(u_dot)*v_dot.dot(v_dot));
        if (abs(Phi) <= 1){
            theta.push_back((theta0+acos(Phi))/PI*180);
            theta.push_back((theta0-acos(Phi))/PI*180);
        }
    }	
	
	void SetRobotEndPos(double x, double y, double z, double yaw, double pitch, double roll)
	{
        mpose[0] = x;
        mpose[1] = y;
        mpose[2] = z;
        mpose[3] = yaw/180*PI;
        mpose[4] = pitch/180*PI;
        mpose[5] = roll/180*PI;
        
        Eigen::AngleAxisd yawAngle(AngleAxisd(mpose[3],Vector3d::UnitZ()));
        Eigen::AngleAxisd pitchAngle(AngleAxisd(mpose[4],Vector3d::UnitY()));
        Eigen::AngleAxisd rollAngle(AngleAxisd(mpose[5],Vector3d::UnitZ()));
        Matrix3d rotate;
        rotate = yawAngle*pitchAngle*rollAngle;
        Matrix4d T = Matrix4d::Identity();
        T.block<3,3>(0,0) = rotate;
        T(0,3) = x;
        T(1,3) = y;
        T(2,3) = z;

        for (int i=0; i<4; i++){
            for(int j=0; j<4; j++){
                mTransMatrix[4*i+j] = T(i,j);
            }
        }
	}

	void GetJointAngles(double &angle1, double &angle2, double &angle3, double &angle4)
	{
        robotBackward(mTransMatrix, mConfig, mangle);
	}

	void SetRobotJoint(double angle1, double angle2, double angle3, double angle4)
	{
        mangle[0] = angle1;
        mangle[1] = angle2;
        mangle[2] = angle3;
        mangle[3] = angle4;
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
        yaw = Euler(0)/PI*180;
        pitch = Euler(1)/PI*180;
        roll = Euler(2)/PI*180;
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
        cout<<endl;
		for(int i=0;i<4;i++){
            T = T * R2SE3(Scara.v.col(i),Scara.w.col(i),i,q[i]);
        }
        T = T*Scara.G0;
        for (int i=0; i<4; i++){
            for(int j=0; j<4; j++){
                TransVector[4*i+j] = T(i,j);
            }
        }
	}
}
