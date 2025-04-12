#include "MocapApi.h"
#include "Rotqua.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iomanip>
#include <Eigen/Dense>
#include <cmath>
#include <WS2tcpip.h>
#include <WinSock2.h>
#include <Windows.h>
#include <stdio.h>  
#include <stdlib.h>  
#include <ctime>
#include "win_client_cpp.h"

#define SOCKET_FLAG1

#pragma comment(lib, "MocapApi.lib")
#pragma comment(lib, "ws2_32.lib") 

const uint16_t JointNum = 6;
const uint16_t JointList[2][JointNum] = { { 0,9,36,37,38,39 }, //左臂涉及的动捕数据节点编号
                                            {0, 9, 13, 14, 15, 16} };//右臂s涉及的动捕数据节点编号
const char *ErrorType[23] = {"Error_None", "Error_MoreEvent",
    "Error_InsufficientBuffer", "Error_InvalidObject", "Error_InvalidHandle",
    "Error_InvalidParameter", "Error_NotSupported", "Error_IgnoreUDPSetting",
    "Error_IgnoreTCPSetting", "Error_IgnoreBvhSetting", "Error_JointNotFound",
    "Error_WithoutTransformation", "Error_NoneMessage", "Error_NoneParent",
    "Error_NoneChild", "Error_AddressInUse", "Error_ServerNotReady",
    "Error_ClientNotReady", "Error_IncompleteCommand", "Error_UDP",
    "Error_TCP", "Error_QueuedCommandFaild", "Error_InterfaceIncompatible"};//MocapApi反馈错误的编号

MocapApi::EMCPError error;

void ErrorHandling(const char* message)
{
    fputs(message, stderr);
    fputc('\n', stderr);
    WSACleanup();
    exit(1);
}

void CheckError(const char* Todo) {
    if (error != MocapApi::Error_None) {
        std::cerr << Todo << " failed " << ErrorType[error] << std::endl;
        exit(1);
    }
}

int main() {
    std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(3);//小数显示3位
    double theta[5], thetaTemp[5], formerTheta[2][5], SetTimeOut = 180,      //SetTimeOut为整个流程的限时，单位为秒
        FPS = 4, SpeedLimit = 180, threshold = SpeedLimit/FPS;       //FPS为采集数据的帧率，SpeedLimit为机械臂每秒转速限制
    const char* serv_ip = "127.0.0.1";      //工控机IP地址
    //const char* serv_port = "9280";
    const char serv_port[2][5] = {
        "9680", // 左臂端口为 "9280"
        "9681"  // 右臂端口为 "9281"
    };
    bool starterFlag = true;
    bool State[2] = { true, false };       //左右臂的工作状态
    
    //setting_init();
        // 初始化WinSock
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);

    // 创建Socket
    SOCKET clientSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (clientSocket == INVALID_SOCKET) {
        std::cerr << "Socket creation failed" << std::endl;
        return -1;
    }

    // 设置服务器地址
    sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(12345); // 端口号

    // 使用inet_pton替换inet_addr
    if (inet_pton(AF_INET, "192.168.25.36", &serverAddr.sin_addr) <= 0) {
        std::cerr << "Invalid IP address" << std::endl;
        closesocket(clientSocket);
        WSACleanup();
        return -1;
    }

    // 连接服务器
    if (connect(clientSocket, (sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
        std::cerr << "Connection to server failed" << std::endl;
        closesocket(clientSocket);
        WSACleanup();
        return -1;
    }

    std::cout << "Connected to ROS server" << std::endl;






    //各个关节角的修正值
    double thetaCor[2][5] = {
        { -90, 0, 0, 180, 0 },
        { 90, 0, 0, 0, 0 }
    };

    //以下为使用MocapApi的准备工作，具体的设置参见Axis Studio中关于BVH数据广播的设置
    
    // 创建句柄
    MocapApi::MCPApplicationHandle_t applicationHandle = 0;
    MocapApi::MCPSettingsHandle_t mcpSettingsHandle = 0;
    float x, y, z, w; // 用于存储旋转数据

    // 获取Application接口指针，给句柄赋值
    MocapApi::IMCPApplication* application = nullptr;
    MocapApi::MCPGetGenericInterface(MocapApi::IMCPApplication_Version,
        reinterpret_cast<void**>(&application));
    error = application->CreateApplication(&applicationHandle);
    CheckError("creat application");

    //获取RenderSettings接口指针，给句柄赋值
    MocapApi::IMCPRenderSettings* mcpRenderSettings = nullptr;
    MocapApi::MCPGetGenericInterface(MocapApi::IMCPRenderSettings_Version,
        reinterpret_cast<void**>(&mcpRenderSettings));
    MocapApi::MCPRenderSettingsHandle_t mcpRenderSettingsHandle = 0;
    mcpRenderSettings->CreateRenderSettings(&mcpRenderSettingsHandle);

    //设置为默认RenderSettings
    MocapApi::EMCPPreDefinedRenderSettings preDefinedRenderSettings
        = MocapApi::PreDefinedRenderSettings_Default;
    error = mcpRenderSettings->GetPreDefRenderSettings(preDefinedRenderSettings,
        &mcpRenderSettingsHandle);
    CheckError("Set default RenderSettings");

    //设置RenderSettings
    error = application->SetApplicationRenderSettings(mcpRenderSettingsHandle,
        applicationHandle);
    CheckError("Set RenderSettings");

    //获取设置接口指针以及给句柄赋值
    MocapApi::IMCPSettings* mcpSettings = nullptr;
    MocapApi::MCPGetGenericInterface(MocapApi::IMCPSettings_Version,
        reinterpret_cast<void**>(&mcpSettings));
    error = mcpSettings->CreateSettings(&mcpSettingsHandle);
    CheckError("create settings");

    //设置TCP
    char serverIp[] = "127.0.0.1";
    uint16_t serverPort = 7003;
    error = mcpSettings->SetSettingsTCP(serverIp, serverPort, mcpSettingsHandle);
    CheckError("set TCP");

    //设置BVH旋转（Rotation)
    MocapApi::EMCPBvhRotation bvhRotation = MocapApi::BvhRotation_YXZ;
    error = mcpSettings->SetSettingsBvhRotation(bvhRotation, mcpSettingsHandle);
    CheckError("set BvhRotation");

    //设置BVH变换(Transformation)
    MocapApi::EMCPBvhTransformation bvhTransformation = MocapApi::BvhTransformation_Enable;
    error = mcpSettings->SetSettingsBvhTransformation(bvhTransformation, mcpSettingsHandle);
    CheckError("set BvhTransformation");

    //设置BVH数据类型
    MocapApi::EMCPBvhData bvhData = MocapApi::BvhDataType_Binary;
    error = mcpSettings->SetSettingsBvhData(bvhData, mcpSettingsHandle);
    CheckError("set BvhDataType");

    //确认设置
    error = application->SetApplicationSettings(mcpSettingsHandle, applicationHandle);
    CheckError("set Application Settings");

    // 打开应用
    error = application->OpenApplication(applicationHandle);
    CheckError("open application");
    if (error != MocapApi::Error_None) {
        application->DestroyApplication(applicationHandle);
        return -1;
    }

    // 获取IMCPAvatar接口指针
    MocapApi::IMCPAvatar* mcpAvatar = nullptr;
    MocapApi::MCPGetGenericInterface(MocapApi::IMCPAvatar_Version,
        reinterpret_cast<void**>(&mcpAvatar));

    //获取Joint接口指针
    MocapApi::IMCPJoint* mcpJoint = nullptr;
    MocapApi::MCPGetGenericInterface(MocapApi::IMCPJoint_Version,
        reinterpret_cast<void**>(&mcpJoint));

    //获取RigidBody接口指针
    MocapApi::IMCPRigidBody* mcpRigidBody = nullptr;
    MocapApi::MCPGetGenericInterface(MocapApi::IMCPRigidBody_Version,
        reinterpret_cast<void**>(&mcpRigidBody));

    //以上为使用MocapApi的准备工作
    


    //校准，用来确定动作基准，如果采取T-pose，可以不用校准
    std::cout << "Calibration begins\nCalibration posture, please" << std::endl;
    Sleep(3000);
    std::cout << "Please hold still" << std::endl;


    uint32_t sizeEvent, maxSizeEvent = 100;
    MocapApi::MCPEvent_t* events;
    uint32_t count = 0, CaliTimeOut = 20, TimeOut = SetTimeOut*FPS;
    bool checkflag = false;
    Eigen::Matrix3d CtMat[2][4], UtMat, Mat[2][4], StdMat[2][4];
    //UtMat是单位矩阵，Mat是肩到手4个关节的绝对旋转矩阵
    //CtMat是校准Mat的矩阵，StdMat是校准后的矩阵，如前所述，如果采取T-pose为姿势基准，不需要整个校准的过程
    //StdMat的第一列，也就是x轴的方向，指示手臂或者手指的朝向
    UtMat << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    CtMat[0][0] = CtMat[1][0] = UtMat;
    Mat[0][0] = Mat[1][0] = UtMat;

    do {
        count++;
        sizeEvent = 0;
        error = application->PollApplicationNextEvent(nullptr, &sizeEvent,
            applicationHandle);
        if (error != MocapApi::Error_None) break;
        events = new MocapApi::MCPEvent_t[maxSizeEvent];
        error = application->PollApplicationNextEvent(events, &sizeEvent,
            applicationHandle);
        CheckError("Poll Next Event");
        Sleep(200);
        uint32_t JointSize = 100;
        MocapApi::MCPJointHandle_t* pJointHandle
            = new MocapApi::MCPJointHandle_t[JointSize];
        if (error == MocapApi::Error_None && sizeEvent != 0) { 
            for (int i = 0; i < sizeEvent && i < maxSizeEvent; i++) {
                events[i].size = sizeof(events[i]);
                MocapApi::MCPAvatarHandle_t avatarHandle = events[i].eventData.motionData.avatarHandle;

                error = mcpAvatar->GetAvatarJoints(pJointHandle, &JointSize, avatarHandle);
                CheckError("Get Avatar Joints");
                if (error != MocapApi::Error_None) {
                    continue;
                }
                else
                    checkflag = true;
                for (int mp = 0; mp < 2; mp++) {        //mp表示左右臂
                    for (int j = 1; j < 4; j++) {       //1 2 3分别为大臂、小臂、手
                        error = mcpJoint->GetJointLocalRotation(&x, &y, &z, &w, pJointHandle[JointList[mp][j+2]]);
                        CheckError("Get Joint Local Rotation");
                        if (error == MocapApi::Error_None) {
                            //std::cout << '(' << x << ", " << y << ", " << z << ", " << w << ")" << std::endl;
                            Rotation rot;
                            rot.quaternion(x, y, z, w);
                            //rot.display();
                            Eigen::Matrix3d RotMat;
                            RotMat << rot.data[0], rot.data[1], rot.data[2],
                                rot.data[3], rot.data[4], rot.data[5],
                                rot.data[6], rot.data[7], rot.data[8];
                            Mat[mp][j] = Mat[mp][j - 1] * RotMat;
                            //RotMat为相对旋转矩阵，累积相乘后得到绝对旋转矩阵
                        }

                    }
                }
            }
        }
        delete[] pJointHandle;
        delete[] events;
    } while (error != MocapApi::Error_InsufficientBuffer && count < CaliTimeOut && !checkflag);
    
    if (!checkflag || error != MocapApi::Error_None) {
        std::cout << "failed to calibrate, exiting program..." << std::endl;
    }
    else {      //完成校准后，正式开始遥操作
        // 获取当前时间
        std::time_t t = std::time(nullptr);
        struct tm timeinfo;
        localtime_s(&timeinfo, &t); // 使用 localtime_s 安全函数

        // 构建文件名，使用当前日期和时间
        char filename[200];
        std::strftime(filename, sizeof(filename), "Tomato_Harvester_Mocap_%Y%m%d_%H%M%S.txt", &timeinfo);

        //将数据写入文件
        //std::ofstream file("D:\\NeuronMocapOutput\\sequence.txt");
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "无法打开文件进行写入" << std::endl;
            return 1;
        }
        file << FPS << std::endl;

        std::cout << "calibration succeeded\nstarting posture please" << std::endl; 
        Sleep(3000);
        std::cout << "free to move now" << std::endl;

        //轮询事件
        count = 0;

#ifdef SOCKET_FLAG
        //准备发送数据（通过Socket)
        WSADATA wsaData;
        SOCKET hSocket[2];
        SOCKADDR_IN servAddr;

        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
            ErrorHandling("WSAStartup() error!");
#endif

        for(int mp = 0; mp < 2; mp++) {
            for (int i = 1; i < 4; i++) {
                CtMat[mp][i] = UtMat * Mat[mp][i].inverse();
            }
#ifdef SOCKET_FLAG
            hSocket[mp] = socket(PF_INET, SOCK_STREAM, 0);
            if (hSocket[mp] == INVALID_SOCKET) {
                ErrorHandling("socket() error");
                State[mp] = false;
                continue;
            }
            memset(&servAddr, 0, sizeof(servAddr));
            servAddr.sin_family = AF_INET;
            servAddr.sin_port = htons(atoi(serv_port[mp]));
            inet_pton(AF_INET, serv_ip, &servAddr.sin_addr);
            //servAddr.sin_addr.s_addr = inet_addr("192.168.24.68");
            //servAddr.sin_port = htons(atoi("9120"));

            if (connect(hSocket[mp], (SOCKADDR*)&servAddr, sizeof(servAddr)) == SOCKET_ERROR) {
                ErrorHandling("connect() error");
                State[mp] = false;
            }
            else {
                std::cout << mp << " connect success" << std::endl;
                State[mp] = true;
            }
#endif
        }

        //发送数据的容器
        double data[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        int data_size = sizeof(data);

        //循环采集动捕数据
        do {
            count++;
            sizeEvent = 0;
            error = application->PollApplicationNextEvent(nullptr, &sizeEvent,
                applicationHandle);
            if (error != MocapApi::Error_None) break;
            events = new MocapApi::MCPEvent_t[maxSizeEvent];
            error = application->PollApplicationNextEvent(events, &sizeEvent,
                applicationHandle);
            CheckError("Poll Next Event");
            Sleep(1000/FPS);        //控制频率
            uint32_t JointSize = 100;
            MocapApi::MCPJointHandle_t* pJointHandle
                = new MocapApi::MCPJointHandle_t[JointSize];
            if (error == MocapApi::Error_None && sizeEvent != 0) {
                for (int i = 0; i < sizeEvent && i < maxSizeEvent; i++) {
                    events[i].size = sizeof(events[i]);
                    std::cout << "Time Stamp: " << events[i].fTimestamp << std::endl;
                    MocapApi::MCPAvatarHandle_t avatarHandle = events[i].eventData.motionData.avatarHandle;

                    error = mcpAvatar->GetAvatarJoints(pJointHandle, &JointSize, avatarHandle);
                    CheckError("Get Avatar Joints");
                    std::cout << "Joint Size: " << JointSize << std::endl;
                    //uint32_t StartJoint = 5, EndJoint = 6;
                    for (int mp = 0; mp < 2; mp++) {
                        if (!State[mp])
                            continue;       //如果此臂未连接，则跳过
                        for (int j = 1; j < 4; j++) {
                            std::cout << " *********************" << std::endl;
                            error = mcpJoint->GetJointLocalRotation(&x, &y, &z, &w, pJointHandle[JointList[mp][j + 2]]);
                            CheckError("Get Joint Local Rotation");
                            if (error == MocapApi::Error_None) {

                                std::cout << " LocalRotation" << '(' << x << ", " << y << ", " << z << ", " << w << ")" << std::endl;
                                Rotation rot;
                                rot.quaternion(x, y, z, w);
                                //rot.display();
                                Eigen::Matrix3d RotMat;
                                RotMat << rot.data[0], rot.data[1], rot.data[2],
                                    rot.data[3], rot.data[4], rot.data[5],
                                    rot.data[6], rot.data[7], rot.data[8];
                                Mat[mp][j] = Mat[mp][j - 1] * RotMat;
                                StdMat[mp][j] = Mat[mp][j] * CtMat[mp][j];
                                /*if (j == 3) {
                                    std::cout << StdMat[mp][j] << std::endl;
                                }*/


                            }

                        }
                    }
                }
            }
            delete[] pJointHandle;
            delete[] events;

            //求解机械臂关节角
            for (int mp = 0; mp < 2; mp++) {
                if (State[mp]) {
                    if (mp == 0) {
                        //左臂(0,0)向左，(1,0)向上，(2,0)向前（T-pose下左上前坐标系）
                        thetaTemp[0] = atan2(StdMat[mp][2](0, 0), StdMat[mp][2](2, 0));     //小臂偏航决定第一关节角
                        thetaTemp[1] = atan2(StdMat[mp][1](0, 0), StdMat[mp][1](2, 0));       //大臂偏航决定第二关节角
                        thetaTemp[2] = asin(StdMat[mp][2](1, 0));       //小臂俯仰决定第三关节角
                        thetaTemp[3] = asin(StdMat[mp][3](1, 0));       //手掌俯仰决定第四关节角
                        thetaTemp[4] = atan2(StdMat[mp][3](0, 0), StdMat[mp][3](2, 0));        //手掌偏航决定第五关节角
                        for (int i = 0; i < 5; i++) {
                            thetaTemp[i] *= 180 / 3.14159;
                            thetaTemp[i] += thetaCor[mp][i]; //校正
                        }
                    }
                    else {
                        //(0,0)向右，(1,0)向下，(2,0)向后 （其实还是左上前，但反过来更好理解）
                        thetaTemp[0] = atan2(-StdMat[mp][2](0, 0), -StdMat[mp][2](2, 0));     //小臂偏航决定第一关节角
                        thetaTemp[1] = 3.14159 - atan2(StdMat[mp][1](0, 0), -StdMat[mp][1](2, 0));       //大臂偏航决定第二关节角
                        thetaTemp[2] = 3.14159 - asin(-StdMat[mp][2](1, 0));       //小臂俯仰决定第三关节角
                        thetaTemp[3] = 3.14159 - asin(-StdMat[mp][3](1, 0));       //手掌俯仰决定第四关节角
                        thetaTemp[4] = atan2(-StdMat[mp][3](0, 0), -StdMat[mp][3](2, 0));        //手掌偏航决定第五关节角
                        for (int i = 0; i < 5; i++) {
                            thetaTemp[i] *= 180 / 3.14159;
                            thetaTemp[i] += thetaCor[mp][i]; //校正
                        }
                    }

                    //限定角度范围
                    for (int i = 0; i < 5; i++) {
                        switch (i) {
                        case 0:
                            theta[i] = thetaTemp[i];
                            if (theta[i] < -360) theta[i] = -360;
                            else if (theta[i] > 360) theta[i] = 360;
                            break;
                        case 1:
                            theta[i] = thetaTemp[i];
                            if (theta[i] < -85) theta[i] = -85;
                            else if (theta[i] > 265) theta[i] = 265;
                            break;
                        case 2:
                            theta[i] = thetaTemp[i] - thetaTemp[i - 1];
                            if (theta[i] < -175) theta[i] = -175;
                            else if (theta[i] > 175) theta[i] = 175;
                            break;
                        case 3:
                            theta[i] = thetaTemp[i] - thetaTemp[i - 1];
                            if (theta[i] < -85) theta[i] = -85;
                            else if (theta[i] > 265) theta[i] = 265;
                            break;
                        case 4:
                            theta[i] = thetaTemp[i] - theta[0];
                            if (theta[i] < -360) theta[i] = -360;
                            else if (theta[i] > 360) theta[i] = 360;
                            break;

                        }

                        //限速
                        if (!starterFlag) {
                            if (theta[i] - formerTheta[mp][i] > threshold) {
                                theta[i] = formerTheta[mp][i] + threshold;
                            }
                            else if (theta[i] - formerTheta[mp][i] < -threshold) {
                                theta[i] = formerTheta[mp][i] - threshold;
                            }
                        }
                        formerTheta[mp][i] = theta[i];
                        //std::cout << "joint" << i + 1 << " : " << theta[i] << std::endl;
                        file << theta[i];
                        if (i != 4) file << ' ';
                        else file << std::endl;
                    }

#ifdef SOCKET_FLAG1
                    //准备数据
                    data[5] = 0;    //最后一个角度不需要控制
                    for (int i = 0; i < 5; i++) {
                        data[i] = theta[i]*3.14/180;
                    }
                    std::vector<double> data_vector(data, data + sizeof(data) / sizeof(data[0]));

                    //int res1 = send2ros(data_vector);
                    // 模拟关节角数据发送
        //std::vector<double> joint_angles = { 0.5, -0.8, 1.0, -0.5, 1.3, -1.2 }; // 模拟关节角数据

                    std::string message = serialize_joint_angles(data_vector);

                    message += "\n";
                    // 发送数据
                    send(clientSocket, message.c_str(), message.length(), 0);

                    std::cout << "Sent joint angles: " << message << std::endl;

                    // 等待1秒
                    Sleep(100);
#endif

                    for (int i = 0; i < 5; i++) {
                        std::cout << "arm" << mp << "joint" << i + 1 << " : " << data[i] << std::endl;
                    }
                }
                if (starterFlag && mp) starterFlag = false;
            }
        } while (error != MocapApi::Error_InsufficientBuffer && count < TimeOut);
        if (error != MocapApi::Error_None) {
            // 处理该错误
        }
        file.close();
        std::cout <<"file sequence.txt successfully generated" << std::endl;


    }

    mcpSettings->DestroySettings(mcpSettingsHandle);
    mcpRenderSettings->DestroyRenderSettings(mcpRenderSettingsHandle);
    application->CloseApplication(applicationHandle);
    application->DestroyApplication(applicationHandle);
    
    return 0;
}