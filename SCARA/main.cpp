#include <iostream>
#include <Eigen/Dense>
#include <Srobotconfig.h>
using namespace Eigen;
using namespace std;

int main(int, char**) {
    double x, y ,z ,r ,p, yaw;
    double joint[6]={0};
    Matrix<double, 3, 4> q;
    SRobot::SetRobotJoint(-49.965, 55.391, 17.354, 79.116);
    SRobot::GetJointEndPos(x, y, z, r, p, yaw);


}
