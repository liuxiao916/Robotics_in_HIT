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
    Start.x = 475.693; Start.y = -189.407; Start.z = 802.044;
    Start.yaw = 4.221; Start.pitch = 169.075; Start.roll = -131.743;

    //终止点
    PosStruct End;
    End.x = 441.78; End.y = 86.9; End.z = 877.035;
    End.yaw = 103.388; End.pitch = 170.195; End.roll = -90.995;

    //梯型速度规划
    CHLMotionPlan trajectory1;
    trajectory1.SetPlanPoints(Start, End);  
    trajectory1.SetProfile(10, 10, 10);    //vel °/s， acc °/s.s, dec °/s.s
    trajectory1.SetSampleTime(0.001);      //s
    trajectory1.GetPlanPoints();           //关节空间梯形速度规划
    trajectory1.GetPlanPoints_line();      //笛卡尔空间直线轨迹梯形速度规划
}
