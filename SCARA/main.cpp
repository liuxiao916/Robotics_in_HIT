#include <iostream>
#include <Eigen/Dense>
#include <Srobotconfig.h>
using namespace Eigen;
using namespace std;

int main(int, char**) {
    double x, y ,z ,roll ,p, yaw;
    double joint[4]={0};
    Matrix<double, 3, 4> q;
    SRobot::SetRobotEndPos(415.402,196.478,-17.354,0,180,-149.424);
    //SRobot::SetRobotEndPos(213.430,-0.136,-17.354,0,180,-165.577);
    SRobot::GetJointAngles(joint[0],joint[1],joint[2],joint[3]);
    //cout << x <<" " << y<<" " <<z;

}
