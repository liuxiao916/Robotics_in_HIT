#include <iostream>
#include <Eigen/Dense>
#include <Srobotconfig.h>
using namespace Eigen;
using namespace std;

int main(int, char**) {
    double x, y ,z ,roll ,p, yaw;
    double joint[4]={0};
    Matrix<double, 3, 4> q;
    //Robot::SetRobotEndPos(415.402,196.478,-17.354,0,180,-149.424);
    SRobot::SetRobotEndPos(213.430,-0.136,-17.354,0,180,-165.577);
    //SRobot::SetRobotEndPos(475.693, -189.407, 802.044, 4.221, 169.075, -131.743); (error point)
    SRobot::GetJointAngles(joint[0],joint[1],joint[2],joint[3]);
    cout << joint[0] <<" " << joint[1] <<" " <<joint[2]<<" "<< joint[3]<<endl ;

    SRobot::SetRobotJoint(2.101, 46.426, 17.354, 79.111);
    SRobot::GetJointEndPos(x,y,z,roll,p,yaw);
    cout << x <<" " << y <<" " <<z<<" "<< roll<<" "<<p<<" "<<yaw <<endl;
    SRobot::SetRobotJoint(-49.965, 55.391, 17.354, 79.116);
    SRobot::GetJointEndPos(x,y,z,roll,p,yaw);
    cout << x <<" " << y <<" " <<z<<" "<< roll<<" "<<p<<" "<<yaw <<endl;
    SRobot::SetRobotJoint(-64.768, 129.463, 17.354, 79.118);
    SRobot::GetJointEndPos(x,y,z,roll,p,yaw);
    cout << x <<" " << y <<" " <<z<<" "<< roll<<" "<<p<<" "<<yaw <<endl;
}
