// ConsoleApplication1.cpp : 定义控制台应用程序的入口点。
//

#include<iostream>
#include<winsock.h>
#include <conio.h>
#pragma comment(lib,"ws2_32.lib")
using namespace std;
using namespace Eigen;
using namespace HLRobot;

void sendCmd(SOCKET& ser, Vector6d j) {

	char cmd[128] = {};
	sprintf_s(cmd, sizeof(cmd), "[4# p=%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]", j(0), j(1), j(2), j(3), j(4), j(5));

	int send_len, recv_len;
	char recv_buf[200] = {};
	send_len = send(ser, cmd, 200, 0);
	recv_len = recv(ser, recv_buf, 200, 0);
	cout << recv_buf << endl;

	memset(cmd, 0, sizeof(cmd));
	memset(recv_buf, 0, sizeof(recv_buf));
	sprintf_s(cmd, sizeof(cmd), "[5# Move.Joint p]");
	send_len = send(ser, cmd, 200, 0);
	recv_len = recv(ser, recv_buf, 200, 0);
	cout << recv_buf << endl;
}

void Task1(SOCKET& ser, const Vector6d& p1, const Vector6d& p2, const Vector6d& p3) {
	Vector6d j;
	SetRobotEndPos(p1(0), p1(1), p1(2), p1(3), p1(4), p1(5));
	GetJointAngles(j(0), j(1), j(2), j(3), j(4), j(5));
	sendCmd(ser, j);
	Sleep(500);
	SetRobotEndPos(p2(0), p2(1), p2(2), p2(3), p2(4), p2(5));
	GetJointAngles(j(0), j(1), j(2), j(3), j(4), j(5));
	sendCmd(ser, j);
	Sleep(500);
	SetRobotEndPos(p3(0), p3(1), p3(2), p3(3), p3(4), p3(5));
	GetJointAngles(j(0), j(1), j(2), j(3), j(4), j(5));
	sendCmd(ser, j);
	Sleep(500);
}

void drawTriangle(SOCKET& ser, const Vector6d& p1, const Vector6d& p2, const Vector6d& p3)
{
	vector<Vector6d> p;

	// p1 -> p2
	for (int i = 0; i < 6; i++) {
		Vector6d tmp = p1 + i / 6 * (p2 - p1);
		p.emplace_back(tmp);
	}
	for (int i = 0; i < 6; i++) {
		Vector6d tmp = p2 + i / 6 * (p3 - p2);
		p.emplace_back(tmp);
	}
	for (int i = 0; i < 6; i++) {
		Vector6d tmp = p3 + i / 6 * (p1 - p3);
		p.emplace_back(tmp);
	}

	for (int i = 0; i < p.size(); i++) {
		Vector6d p_ = p[i];
		SetRobotEndPos(p_(0), p_(1), p_(2), p_(3), p_(4), p_(5));
		Vector6d j;
		GetJointAngles(j(0), j(1), j(2), j(3), j(4), j(5));

		sendCmd(ser, j);
		Sleep(500);
	}
}

void drawCircle(SOCKET& ser, Vector6d center, double r)
{
	vector<Vector6d> p;
	for (int i = 0; i < 36; i++) {
		Vector6d tmp = center;
		tmp(0) += r * cos(i / 36 * PI);
		tmp(1) += r * sin(i / 36 * PI);
		p.emplace_back(tmp);
	}

	for (int i = 1; i < 36; i++) {
		Vector6d p_ = p[i];
		SetRobotEndPos(p_(0), p_(1), p_(2), p_(3), p_(4), p_(5));
		Vector6d j;
		GetJointAngles(j(0), j(1), j(2), j(3), j(4), j(5));
		sendCmd(ser, j);
		Sleep(500);
	}

}

void initialization();
#pragma comment(lib, "WS2_32.lib")
int main()
{   //
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
	if (connect(s_server, (SOCKADDR *)&server_addr, sizeof(SOCKADDR)) == SOCKET_ERROR) {
		cout << "服务器连接失败！" << endl;
		WSACleanup();
	}
	else {
		cout << "服务器连接成功！" << endl;
	}

	//登录
    send_len = send(s_server, "[1# System.Login 0]", 100, 0);
    recv_len = recv(s_server, recv_buf, 100, 0);
    cout << recv_buf << endl;
    memset(recv_buf,'\0',sizeof(recv_buf));
    Sleep(1200);
	//使能
    send_len = send(s_server, "[2# Robot.PowerEnable 1,1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

	//在此添加程序
	send_len = send(s_server, "[3# Location p]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1200);

	Vector6d p1, p2, p3, center;
	p1 << 415.402, 196.478, -17.354, 0, 180, -149.424;
	p2 << 409.694, -167.773, -17.354, 0, 180, -106.310;
	p3 << 213.430, -0.136, -17.354, 0, 180, -165.577;
	center << 337.865, 23.742, -17.354, 0, 180, -152.259;
	double r = 120;

	closesocket(s_server);
	//释放DLL资源
	WSACleanup();
	return 0;
}
void initialization() {
	//初始化套接字库
	WORD w_req = MAKEWORD(2, 2);//版本号
	WSADATA wsadata;
	int err;
	err = WSAStartup(w_req, &wsadata);
	if (err != 0) {
		cout << "初始化套接字库失败！" << endl;
	}
	else {
		cout << "初始化套接字库成功！" << endl;
	}
	//检测版本号
	if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2) {
		cout << "套接字库版本号不符！" << endl;
		WSACleanup();
	}
	else {
		cout << "套接字库版本正确！" << endl;
	}
}
