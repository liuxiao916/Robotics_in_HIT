#include <iostream>
#include "MotionPlan.h"

using namespace std;

/****
 * 实验四: 轨迹规划
 * 要 求：使用C/C++完成梯型速度规划，生成data.txt文件
 * 规划类型：关节空间、笛卡尔空间（直线）
 * 点位数量：起始点和终止点
 * 给定条件：Vel，Acc，Dec
 * 
 */

int main()
{   //起始点
    PosStruct Start;
    Start.x = 415.402; Start.y = 196.478; Start.z = -17.354;
    Start.yaw = 0; Start.pitch = 180; Start.roll = -149.424;

    //终止点
    PosStruct End;
    End.x = 409.694; End.y = -167.773; End.z = -17.354;
    End.yaw = 0; End.pitch = 180; End.roll = -106.310;

    //梯型速度规划
    CHLMotionPlan trajectory1;
    trajectory1.SetPlanPoints(Start, End);  
    trajectory1.SetProfile(10, 10, 10);    //vel °/s， acc °/s.s, dec °/s.s
    trajectory1.SetSampleTime(0.001);      //s
    trajectory1.GetPlanPoints();           //关节空间梯形速度规划
    trajectory1.GetPlanPoints_line();      //笛卡尔空间直线轨迹梯形速度规划
}
