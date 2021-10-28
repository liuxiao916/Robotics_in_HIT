#ifndef ROBOTCONFIG_H_
#define ROBOTCONFIG_H_
const double PI = 3.1415926;
#include <Eigen/Dense>
using namespace Eigen;
namespace SRobot
{ 
	//Inverse kinematics solution
	void SetRobotEndPos(double x, double y, double z, double yaw, double pitch, double roll);
	void GetJointAngles(double &angle1, double &angle2, double &angle3, double &angle4);
	
	//Forward kinematics solution
	void SetRobotJoint(double angle1, double angle2, double angle3, double angle4);
	void GetJointEndPos(double &x, double &y, double &z, double &yaw, double &pitch, double &roll);

	//Inverse kinematics and Forward kinematics method function
	void robotBackward(const double* TransVector, bool config, double* theta);
	void robotForward(const double* q, double* TransVector, bool config);

	class SCARA {
	public:
		Matrix<double, 3, 4> q;
        Matrix<double, 3, 4> w;
        Matrix<double, 3, 4> v;
        Matrix4d G0;
        double a1,a2;
		SCARA(double a1, double a2):a1(a1),a2(a2) {
            q.block<3,1>(0,0) << 0, 0, 0; 
            q.block<3,1>(0,1) << a1, 0, 0; 
            q.block<3,1>(0,2) << 0, 0, 0;
            q.block<3,1>(0,3) << a1+a2, 0, 0;  
            w.block<3,1>(0,0) << 0, 0, 1; 
            w.block<3,1>(0,1) << 0, 0, 1; 
            w.block<3,1>(0,2) << 0, 0, 0;
            w.block<3,1>(0,3) << 0, 0, -1;  
            for(int i =0; i<4; i++){
                v.col(i) = -w.col(i).cross(q.col(i));
            }
            v.col(2) << 0, 0 ,1;  //no rotation
            G0 << 1,0,0,a1+a2,
                0,-1,0,0,
                0,0,-1,0,
                0,0,0,1; 
        };
		~SCARA() {};

	private:


	};
}

#endif
