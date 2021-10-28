#include "stdafx.h"
#include "FtpControl.h"

#include "Poco/Net/FTPClientSession.h"
#include "Poco/Net/DialogSocket.h"
#include "Poco/Net/SocketAddress.h"
#include "Poco/Net/NetException.h"
#include <iostream>
#include <string>
#include <fstream>

using namespace std;

bool FtpControl::Upload(std::string ip, std::string workingDirectory, std::string srcfile, std::string dstfile)
{
    try
    {
        //����FTP������
        Poco::Net::FTPClientSession session(ip);//����FTP�Ự����
        if (!session.isOpen())
        {
            return false;
        }

        //��¼(Ĭ���˻���������)
        session.login("", "");
        if (!session.isLoggedIn())
        {
            session.close();
            return false;
        }

        //���ù����ռ估�ļ�����
        session.setWorkingDirectory(workingDirectory);
        session.setFileType(Poco::Net::FTPClientSession::FileType::TYPE_TEXT);

        //���ļ�
        fstream fp(srcfile, ios::in);
        if (!fp)
        {
            session.close();
            return false;
        }

        //ɾ����������ԭ�е��ļ�
        std::istream &ftpin = session.beginList("");
        string list;
        vector<string> filelist;
        while (ftpin >> list)
        {
            if (list == dstfile)
            {
                //ɾ�������������ļ�
                session.remove(dstfile);
            }
        }
        session.endList();

        //�ϴ��ļ�(�Ƴ�����)
        ostream& os = session.beginUpload(dstfile);//ָ��Զ���ļ�����ʼ�ϴ�
        string line;
        bool isFirst = true;
        while (getline(fp, line))
        {
            if (line != "")
            {
                if (!isFirst)
                {
                    os << "\r\n" << line;
                }
                else
                {
                    os << line;
                    isFirst = false;
                }
            }
        }
        session.endUpload();

        //�ر��˳�
        fp.close();
        session.close();

        return true;
    }
    catch (const Poco::Net::FTPException& e)
    {
        std::cout << e.message() << std::endl;
    }

    return false;
}

bool FtpControl::CheckServer(std::string ip)
{
    try
    {
        //����FTP������
        Poco::Net::FTPClientSession session(ip);//����FTP�Ự����
        if (!session.isOpen())
        {
            return false;
        }

        //��¼(Ĭ���˻���������)
        session.login("", "");
        if (!session.isLoggedIn())
        {
            session.close();
            return false;
        }

        session.close();
        return true;
    }
    catch (const std::exception&)
    {

    }

    return false;
}
