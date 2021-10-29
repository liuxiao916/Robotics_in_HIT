// ConsoleApplication1.cpp : 定义控制台应用程序的入口点。

#include <iostream>
#include <winsock.h>
#include <conio.h>
#include "AProbotconfig.h"
#include "MotionPlan.h"
#include "eigen3/Eigen/Dense"
#include "FtpControl.h"

#pragma comment(lib, "ws2_32.lib")

using namespace std;
using namespace APRobot;
using namespace Eigen;

void initialization();
#pragma comment(lib, "ws2_32.lib")

int main(){
    PosStruct Point[8];
    Point[0].x = -0.413; Point[0].y = 328.104; Point[0].z = -704.883;
    Point[0].yaw = 0; Point[0].pitch = 180; Point[0].roll = -161.995;

    Point[1].x = -80.413; Point[1].y = 278.104; Point[1].z = -704.883;
    Point[1].yaw = 0; Point[1].pitch = 180; Point[1].roll = -161.995;

    Point[2].x = -160.413; Point[2].y = 238.104; Point[2].z = -704.883;
    Point[2].yaw = 0; Point[2].pitch = 180; Point[2].roll = -161.995;

    Point[3].x = -250.413; Point[3].y = 193.104; Point[3].z = -704.883;
    Point[3].yaw = 0; Point[3].pitch = 180; Point[3].roll = -161.995;

    Point[4].x = -65.413; Point[4].y = 438.104; Point[4].z = -704.883;
    Point[4].yaw = 0; Point[4].pitch = 180; Point[4].roll = -161.995;

    Point[5].x = -145.413; Point[5].y = 388.104; Point[5].z = -704.883;
    Point[5].yaw = 0; Point[5].pitch = 180; Point[5].roll = -161.995;

    Point[6].x = -230.414; Point[6].y = 343.104; Point[6].z = -704.883;
    Point[6].yaw = 0; Point[6].pitch = 180; Point[6].roll = -161.995;

    Point[7].x = -310.139; Point[7].y = 302.610; Point[7].z = -704.883;
    Point[7].yaw = 0; Point[7].pitch = 180; Point[7].roll = -161.995;

    int flag1 = 0;
    int flag2 = 0;
    
    cout << "请选择抓取点：";
    cin >> flag1;
    cout << "已选择第" << flag1 << "个抓取点" << endl;

    cout << "请选择放置点：";
    cin >> flag2;
    cout << "已选择第" << flag2 << "个放置点" << endl;

    PosStruct Start1;
    PosStruct Start2;
    PosStruct End1;
    PosStruct End2;

    Start1.x = Point[flag1].x; Start1.y = Point[flag1].y; Start1.z = Point[flag1].z + 100;
    Start1.yaw = 0; Start1.pitch = 180; Start1.roll = Point[flag1].roll;
    Start2.x = Point[flag1].x; Start2.y = Point[flag1].y; Start2.z = Point[flag1].z;
    Start2.yaw = 0; Start2.pitch = 180; Start2.roll = Point[flag1].roll;
    End1.x = Point[flag2].x; End1.y = Point[flag2].y; End1.z = Point[flag2].z + 100;
    End1.yaw = 0; End1.pitch = 180; End1.roll = Point[flag2].roll;
    End2.x = Point[flag2].x; End2.y = Point[flag2].y; End2.z = Point[flag2].z;
    End2.yaw = 0; End2.pitch = 180; End2.roll = Point[flag2].roll;

    // 梯型速度规划
    CHLMotionPlan trajectory1;
    trajectory1.SetProfile(10, 10, 10, 10, 2, 2);
    trajectory1.SetSampleTime(0.001);
    trajectory1.SetPlanPoints(Start1, Start2);
    trajectory1.GetPlanPoints_line("line1.txt");
    FtpControl::Upload("192.168.10.101", "data", "line1.txt", "line1.txt");
    trajectory1.SetPlanPoints(Start2, Start1);
    trajectory1.GetPlanPoints_line("line2.txt");
    FtpControl::Upload("192.168.10.101", "data", "line2.txt", "line2.txt");
    trajectory1.SetPlanPoints(Start1, End1);
    trajectory1.GetPlanPoints_line("line3.txt");
    FtpControl::Upload("192.168.10.101", "data", "line3.txt", "line3.txt");
    trajectory1.SetPlanPoints(End1, End2);
    trajectory1.GetPlanPoints_line("line4.txt");
    FtpControl::Upload("192.168.10.101", "data", "line4.txt", "line4.txt");
    trajectory1.SetPlanPoints(End2, End1);
    trajectory1.GetPlanPoints_line("line5.txt");
    FtpControl::Upload("192.168.10.101", "data", "line5.txt", "line5.txt");


    /***********************
    *
    *该部分为与机器人之间的通讯，不需要更改
    *
    ************************/
    //定义长度变量
    int send_len = 0;
    int recv_len = 0;
    //定义发送缓冲区和接受缓冲区
    char send_buf[100] = {};
    char recv_buf[200] = {};
    string recvstr;
    //定义服务端套接字，接受请求套接字
    SOCKET s_server;
    //服务端地址客户端地址
    SOCKADDR_IN server_addr;
    initialization();
    //填充服务端信息
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.S_un.S_addr = inet_addr("192.168.10.120");
    server_addr.sin_port = htons(2090);
    //创建套接字
    s_server = socket(AF_INET, SOCK_STREAM, 0);
    if (connect(s_server, (SOCKADDR *)&server_addr, sizeof(SOCKADDR)) == SOCKET_ERROR){
        cout << "服务器连接失败！" << endl;
        WSACleanup();
    }
    else{
        cout << "服务器连接成功！" << endl;
    }


    /***********************
    *
    *该部分是机器人宏指令的使用方法
    *
    ************************/
    // 登录
    send_len = send(s_server, "[1# System.Login 0]", 100, 0);
    recv_len = recv(s_server, recv_buf, 100, 0);
    cout << "[Login]" << "\t" << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(500);
    // 使能
    send_len = send(s_server, "[2# Robot.PowerEnable 1,1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << "[PowerEnable]" << "\t" << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(2000);
    // 系统停止
    send_len = send(s_server, "[3# System.Abort 1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << "[Abort]" << "\t" << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(100);
    // 启动
    send_len = send(s_server, "[4# System.Start ]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << "[Start]" << "\t" << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(100);
    // Home
    send_len = send(s_server, "[5# Robot.Home 1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << "[Home]" << "\t" << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(100);
    // 自动模式
    send_len = send(s_server, "[6# System.Auto 1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << "[AutoMode]" << "\t" << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(200);
    // PPB使能
    send_len = send(s_server, "[7# PPB.Enable 1,1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 100, 0);
    cout << "[PPB_Enable]" << "\t" << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(200);
    // 笛卡尔坐标系
    send_len = send(s_server, "[8# Robot.Frame 1,2]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << "[Frame2]" << "\t" << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(200);

    send_len = send(s_server, "[8# IO.Set DOUT(20103),0]", 100, 0);
    recv_len = recv(s_server, recv_buf, 100, 0);
    cout << "[IO_OFF]" << '\t' << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1000);
    send_len = send(s_server, "[8# IO.Set DOUT(20104),0]", 100, 0);
    recv_len = recv(s_server, recv_buf, 100, 0);
    cout << "[IO_OFF]" << '\t' << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1000);


    for (int i = 1; i <= 5; i++){
        string temp, flag;
        const char* Command;
        flag = to_string(i);
        temp = "[0# PPB.ReadFile 1,/data/line" + flag + ".txt]";
        Command = temp.data();
        send_len = send(s_server, Command, 100, 0);
        recv_len = recv(s_server, recv_buf, 100, 0);
        cout << "[ReadFile]" << '\t' << recv_buf << endl;
        memset(recv_buf, '\0', sizeof(recv_buf));
        Sleep(1000);

        //笛卡尔到达起始点
        send_len = send(s_server, "[0# PPB.J2StartPoint 1,0,0]", 100, 0);
        recv_len = recv(s_server, recv_buf, 100, 0);
        cout << "[ToStartPoint]" << '\t' << recv_buf << endl;
        memset(recv_buf, '\0', sizeof(recv_buf));
        Sleep(1000);

        //PPB运行
        send_len = send(s_server, "[0# PPB.Run 1]", 100, 0);
        recv_len = recv(s_server, recv_buf, 100, 0);
        cout << "[Run]" << '\t' << recv_buf << endl;
        memset(recv_buf, '\0', sizeof(recv_buf));
        Sleep(1000);

        if (i == 1){
            send_len = send(s_server, "[7# WaitTime 2000]", 100, 0);
            recv_len = recv(s_server, recv_buf, 100, 0);
            cout << "[WAIT]" << '\t' << recv_buf << endl;
            memset(recv_buf, '\0', sizeof(recv_buf));
            Sleep(1000);

            send_len = send(s_server, "[7# IO.Set DOUT(20103),1]", 100, 0);
            recv_len = recv(s_server, recv_buf, 100, 0);
            cout << "[IO_ON]" << '\t' << recv_buf << endl;
            memset(recv_buf, '\0', sizeof(recv_buf));
            Sleep(1000);

            send_len = send(s_server, "[7# IO.Set DOUT(20104),0]", 100, 0);
            recv_len = recv(s_server, recv_buf, 100, 0);
            cout << "[IO_ON]" << '\t' << recv_buf << endl;
            memset(recv_buf, '\0', sizeof(recv_buf));
            Sleep(1000);
        }
        else if (i == 4){
            send_len = send(s_server, "[7# WaitTime 2000]", 100, 0);
            recv_len = recv(s_server, recv_buf, 100, 0);
            cout << "[WAIT]" << '\t' << recv_buf << endl;
            memset(recv_buf, '\0', sizeof(recv_buf));
            Sleep(1000);

            send_len = send(s_server, "[7# IO.Set DOUT(20103),0]", 100, 0);
            recv_len = recv(s_server, recv_buf, 100, 0);
            cout << "[IO_ON]" << '\t' << recv_buf << endl;
            memset(recv_buf, '\0', sizeof(recv_buf));
            Sleep(1000);

            send_len = send(s_server, "[7# IO.Set DOUT(20104),1]", 100, 0);
            recv_len = recv(s_server, recv_buf, 100, 0);
            cout << "[IO_ON]" << '\t' << recv_buf << endl;
            memset(recv_buf, '\0', sizeof(recv_buf));
            Sleep(1000);

            send_len = send(s_server, "[7# WaitTime 2000]", 100, 0);
            recv_len = recv(s_server, recv_buf, 100, 0);
            cout << "[WAIT]" << '\t' << recv_buf << endl;
            memset(recv_buf, '\0', sizeof(recv_buf));
            Sleep(1000);

            send_len = send(s_server, "[7# IO.Set DOUT(20103),0]", 100, 0);
            recv_len = recv(s_server, recv_buf, 100, 0);
            cout << "[IO_ON]" << '\t' << recv_buf << endl;
            memset(recv_buf, '\0', sizeof(recv_buf));
            Sleep(1000);

            send_len = send(s_server, "[7# IO.Set DOUT(20104),0]", 100, 0);
            recv_len = recv(s_server, recv_buf, 100, 0);
            cout << "[IO_ON]" << '\t' << recv_buf << endl;
            memset(recv_buf, '\0', sizeof(recv_buf));
            Sleep(1000);
        }
    }

    
    // 关闭套接字
    closesocket(s_server);
    // 释放DLL资源
    WSACleanup();
    return 0;
}

void initialization(){
    // 初始化套接字库
    WORD w_req = MAKEWORD(2, 2); //版本号
    WSADATA wsadata;
    int err;
    err = WSAStartup(w_req, &wsadata);
    if (err != 0){
        cout << "初始化套接字库失败！" << endl;
    }
    else{
        cout << "初始化套接字库成功！" << endl;
    }
    // 检测版本号
    if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2){
        cout << "套接字库版本号不符！" << endl;
        WSACleanup();
    }
    else{
        cout << "套接字库版本正确！" << endl;
    }
}
