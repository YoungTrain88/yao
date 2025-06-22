// =================================================================
//      Mocap to Hand Pose Teleoperation (Position + Orientation)
//
//   - VERSION: CORRECTED FOR SENDING HAND POSE
//   - This code CAPTURES AND SENDS the hand's position and orientation.
//   - It DOES NOT perform any Inverse Kinematics (IK).
//   - This is the recommended, more flexible approach.
// =================================================================

#include "MocapApi.h"
#include <iostream>
#include <string>
#include <vector>
#include <iomanip>
#include <Eigen/Dense>
#include <WinSock2.h>
#include <WS2tcpip.h>
#include <Windows.h>

#pragma comment(lib, "MocapApi.lib")
#pragma comment(lib, "ws2_32.lib")

#define SOCKET_FLAG1

// =================================================================
//                   1. CONFIGURATION PARAMETERS
// =================================================================

// --- 网络配置 ---
const std::string ROS_SERVER_IP = "10.181.137.104"; // 你的ROS主机IP
const int ROS_SERVER_PORT = 12345;

// --- 动捕软件 (Axis Studio) 配置 ---
const char* MOCAP_SERVER_IP = "127.0.0.1";
const uint16_t MOCAP_SERVER_PORT = 7003;

// --- 控制参数 ---
const double FPS = 30.0; // 建议使用更高的帧率以获得更平滑的追踪

// --- 动捕节点配置 ---
// 在Axis Studio中确认你的右手腕关节ID
const uint16_t RIGHT_HAND_JOINT_ID = 16;

// =================================================================
//                      2. HELPER FUNCTIONS
// =================================================================
// (Helper functions for error checking and serialization)
// MocapApi错误类型定义
MocapApi::EMCPError g_mocap_error;
const char* ErrorType[23] = {
    "Error_None", "Error_MoreEvent", "Error_InsufficientBuffer", "Error_InvalidObject",
    "Error_InvalidHandle", "Error_InvalidParameter", "Error_NotSupported", "Error_IgnoreUDPSetting",
    "Error_IgnoreTCPSetting", "Error_IgnoreBvhSetting", "Error_JointNotFound", "Error_WithoutTransformation",
    "Error_NoneMessage", "Error_NoneParent", "Error_NoneChild", "Error_AddressInUse",
    "Error_ServerNotReady", "Error_ClientNotReady", "Error_IncompleteCommand", "Error_UDP",
    "Error_TCP", "Error_QueuedCommandFaild", "Error_InterfaceIncompatible"
};

// 检查并报告MocapApi的错误
void CheckError(const char* action) {
    if (g_mocap_error != MocapApi::Error_None) {
        std::cerr << "MocapAPI Error during '" << action << "': " << ErrorType[g_mocap_error] << std::endl;
        exit(1);
    }
}

// 将计算出的关节角度（弧度）序列化为字符串
std::string serialize_joint_angles(const std::vector<double>& joint_angles_rad) {
    std::ostringstream oss;
    for (size_t i = 0; i < joint_angles_rad.size(); ++i) {
        oss << joint_angles_rad[i];
        if (i < joint_angles_rad.size() - 1) {
            oss << ",";
        }
    }
    return oss.str();

}
// 【已添加】用于序列化位置和姿态的函数
std::string serialize_pose_data(const Eigen::Vector3d& pos, const Eigen::Quaterniond& rot) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4);
    oss << pos.x() << "," << pos.y() << "," << pos.z() << ",";
    oss << rot.w() << "," << rot.x() << "," << rot.y() << "," << rot.z();
    return oss.str();
}
// =================================================================
//       3. NEW CORE FUNCTION: GET GLOBAL POSE BY CHAINING
// =================================================================

// =================================================================
//       3. CORRECTED CORE FUNCTION: GET GLOBAL POSE BY CHAINING
// =================================================================

bool getGlobalPoseForJoint(
    MocapApi::IMCPJoint* mcpJoint,
    MocapApi::MCPJointHandle_t* pJointHandles, // 【修正】传入关节句柄数组
    uint16_t targetJointId,
    Eigen::Vector3d& outGlobalPos,
    Eigen::Quaterniond& outGlobalRot)
{
    // 您需要根据Axis Studio中的骨骼结构确认从根到手腕的完整链条
    const std::vector<uint16_t> right_arm_chain = { 0, 1, 2, 8, 13, 14, 15, 16 };

    outGlobalPos = Eigen::Vector3d(0, 0, 0);
    outGlobalRot = Eigen::Quaterniond::Identity();

    float qw, qx, qy, qz;
    float px, py, pz;

    for (uint16_t jointId : right_arm_chain) {
        MocapApi::MCPJointHandle_t currentJointHandle = pJointHandles[jointId];

        // 【修正】使用正确的函数签名，传入关节句柄
        if (mcpJoint->GetJointLocalRotation(&qw, &qx, &qy, &qz, currentJointHandle) != MocapApi::Error_None) {
            return false;
        }
        Eigen::Quaterniond localRot(qw, qx, qy, qz);
        localRot.normalize();

        // 【修正】使用正确的函数签名，传入关节句柄
        if (mcpJoint->GetJointLocalPosition(&px, &py, &pz, currentJointHandle) != MocapApi::Error_None) {
            return false;
        }
        Eigen::Vector3d localPos(px, py, pz);

        outGlobalPos = outGlobalPos + (outGlobalRot * localPos);
        outGlobalRot = outGlobalRot * localRot;
        outGlobalRot.normalize();

        if (jointId == targetJointId) {
            break;
        }
    }

    return true;
}



// =================================================================
//                     4 . MAIN APPLICATION
// =================================================================

int main() {
    std::cout << std::fixed << std::setprecision(4);

    // --- BLOCK 0: NETWORK INITIALIZATION ---
    // (与之前版本完全相同，此处省略)
    #ifdef SOCKET_FLAG1
    // ... connect to ROS server ...
    SOCKET clientSocket;
    std::cout << "--- BLOCK 0: Initializing Network ---" << std::endl;
    {
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
            std::cerr << "WSAStartup failed." << std::endl;
            return 1;
        }

        clientSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (clientSocket == INVALID_SOCKET) {
            std::cerr << "Socket creation failed." << std::endl;
            WSACleanup();
            return 1;
        }

        sockaddr_in serverAddr;
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(ROS_SERVER_PORT);
        inet_pton(AF_INET, ROS_SERVER_IP.c_str(), &serverAddr.sin_addr);

        std::cout << "Connecting to ROS server at " << ROS_SERVER_IP << ":" << ROS_SERVER_PORT << "..." << std::endl;
        if (connect(clientSocket, (sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
            std::cerr << "Connection to ROS server failed. Error code: " << WSAGetLastError() << std::endl;
            closesocket(clientSocket);
            WSACleanup();
            return 1;
        }
        std::cout << "Successfully connected to ROS server." << std::endl;
    }
    #endif


    // --- BLOCK 1: MOCAP API INITIALIZATION ---
    std::cout << "\n--- BLOCK 1: Initializing Mocap API ---" << std::endl;
    MocapApi::MCPApplicationHandle_t applicationHandle = 0;
    MocapApi::IMCPApplication* application = nullptr;
    MocapApi::MCPGetGenericInterface(MocapApi::IMCPApplication_Version, reinterpret_cast<void**>(&application));
    g_mocap_error = application->CreateApplication(&applicationHandle);
    CheckError("Create Application");

    // 获取并创建 RenderSettings
    MocapApi::IMCPRenderSettings* mcpRenderSettings = nullptr;
    MocapApi::MCPGetGenericInterface(MocapApi::IMCPRenderSettings_Version, reinterpret_cast<void**>(&mcpRenderSettings));
    MocapApi::MCPRenderSettingsHandle_t mcpRenderSettingsHandle = 0;
    mcpRenderSettings->CreateRenderSettings(&mcpRenderSettingsHandle);
    MocapApi::EMCPPreDefinedRenderSettings preDefinedRenderSettings = MocapApi::PreDefinedRenderSettings_Default;
    g_mocap_error = mcpRenderSettings->GetPreDefRenderSettings(preDefinedRenderSettings, &mcpRenderSettingsHandle);
    CheckError("Set default RenderSettings");
    g_mocap_error = application->SetApplicationRenderSettings(mcpRenderSettingsHandle, applicationHandle);
    CheckError("Set RenderSettings");

    // 获取并创建 Settings
    MocapApi::IMCPSettings* mcpSettings = nullptr;
    MocapApi::MCPGetGenericInterface(MocapApi::IMCPSettings_Version, reinterpret_cast<void**>(&mcpSettings));
    MocapApi::MCPSettingsHandle_t mcpSettingsHandle = 0;
    g_mocap_error = mcpSettings->CreateSettings(&mcpSettingsHandle);
    CheckError("Create settings");

    // 设置TCP
    g_mocap_error = mcpSettings->SetSettingsTCP(const_cast<char*>(MOCAP_SERVER_IP), MOCAP_SERVER_PORT, mcpSettingsHandle);
    CheckError("Set TCP");

    // 设置BVH旋转
    g_mocap_error = mcpSettings->SetSettingsBvhRotation(MocapApi::BvhRotation_YXZ, mcpSettingsHandle);
    CheckError("Set BvhRotation");

    // 设置BVH变换
    g_mocap_error = mcpSettings->SetSettingsBvhTransformation(MocapApi::BvhTransformation_Enable, mcpSettingsHandle);
    CheckError("Set BvhTransformation");

    // 设置BVH数据类型
    g_mocap_error = mcpSettings->SetSettingsBvhData(MocapApi::BvhDataType_Binary, mcpSettingsHandle);
    CheckError("Set BvhDataType");

    // 确认设置
    g_mocap_error = application->SetApplicationSettings(mcpSettingsHandle, applicationHandle);
    CheckError("Set Application Settings");

    // 打开应用
    g_mocap_error = application->OpenApplication(applicationHandle);
    CheckError("Open Application");
    std::cout << "Mocap application opened successfully." << std::endl;

    MocapApi::IMCPAvatar* mcpAvatar = nullptr;
    MocapApi::MCPGetGenericInterface(MocapApi::IMCPAvatar_Version, reinterpret_cast<void**>(&mcpAvatar));
    MocapApi::IMCPJoint* mcpJoint = nullptr;
    MocapApi::MCPGetGenericInterface(MocapApi::IMCPJoint_Version, reinterpret_cast<void**>(&mcpJoint));

    // --- BLOCK 2: CALIBRATION ---
    std::cout << "\n--- BLOCK 2: Calibration ---" << std::endl;
    std::cout << "Place your hand in the desired STARTING position and orientation." << std::endl;
    Sleep(3000);
    std::cout << "Hold still..." << std::endl;

    Eigen::Vector3d calibPos;
    Eigen::Quaterniond calibRot;
    bool calibration_successful = false;

    for (int i = 0; i < 50; ++i) {
        uint32_t sizeEvent = 0;
        application->PollApplicationNextEvent(nullptr, &sizeEvent, applicationHandle);
        if (sizeEvent > 0) {
            MocapApi::MCPEvent_t* events = new MocapApi::MCPEvent_t[sizeEvent];
            application->PollApplicationNextEvent(events, &sizeEvent, applicationHandle);

            if (sizeEvent > 0) {
                MocapApi::MCPAvatarHandle_t avatarHandle = events[0].eventData.motionData.avatarHandle;

                // 【修正】必须先获取关节句柄数组
                uint32_t jointSize = 256; // 分配足够的空间
                MocapApi::MCPJointHandle_t* pJointHandles = new MocapApi::MCPJointHandle_t[jointSize];
                if (mcpAvatar->GetAvatarJoints(pJointHandles, &jointSize, avatarHandle) == MocapApi::Error_None) {

                    // 【修正】将句柄数组传入核心函数
                    if (getGlobalPoseForJoint(mcpJoint, pJointHandles, RIGHT_HAND_JOINT_ID, calibPos, calibRot)) {
                        calibration_successful = true;
                    }
                }
                delete[] pJointHandles; // 释放内存
            }
            delete[] events;

            if (calibration_successful) break;
        }
        Sleep(50);
    }

    if (!calibration_successful) {
        std::cerr << "Calibration failed! Could not get data from Mocap. Exiting." << std::endl;
        return 1;
    }
    std::cout << "Calibration successful. Starting Pose recorded." << std::endl;

    // --- BLOCK 3: MAIN TELEOPERATION LOOP ---
    std::cout << "\n--- BLOCK 3: Starting Main Loop ---" << std::endl;
    do {
        // 【修正】修正类型转换警告
        Sleep(static_cast<DWORD>(1000.0 / FPS));

        uint32_t sizeEvent = 0;
        application->PollApplicationNextEvent(nullptr, &sizeEvent, applicationHandle);
        if (sizeEvent == 0) continue;

        MocapApi::MCPEvent_t* events = new MocapApi::MCPEvent_t[sizeEvent];
        application->PollApplicationNextEvent(events, &sizeEvent, applicationHandle);

        if (sizeEvent > 0) {
            MocapApi::MCPAvatarHandle_t avatarHandle = events[0].eventData.motionData.avatarHandle;

            // 【修正】在循环中同样需要先获取关节句柄
            uint32_t jointSize = 256;
            MocapApi::MCPJointHandle_t* pJointHandles = new MocapApi::MCPJointHandle_t[jointSize];
            if (mcpAvatar->GetAvatarJoints(pJointHandles, &jointSize, avatarHandle) == MocapApi::Error_None) {

                Eigen::Vector3d currentPos;
                Eigen::Quaterniond currentRot;

                if (getGlobalPoseForJoint(mcpJoint, pJointHandles, RIGHT_HAND_JOINT_ID, currentPos, currentRot)) {
                    Eigen::Vector3d relativePos = calibRot.inverse() * (currentPos - calibPos);
                    Eigen::Quaterniond relativeRot = currentRot * calibRot.inverse();

                    std::string message = serialize_pose_data(relativePos, relativeRot);
                    message += "\n";
                    // 【修正】修正类型转换警告
#ifdef SOCKET_FLAG1
                    send(clientSocket, message.c_str(), static_cast<int>(message.length()), 0);
#endif
                }
            }
            delete[] pJointHandles;
        }
        delete[] events;

    } while (true);

    // --- BLOCK 4: CLEANUP ---
    // ... (清理 MocapAPI 和 Winsock) ...
    std::cout << "\n--- BLOCK 4: Cleaning Up ---" << std::endl;
    mcpSettings->DestroySettings(mcpSettingsHandle);
    mcpRenderSettings->DestroyRenderSettings(mcpRenderSettingsHandle);
    application->CloseApplication(applicationHandle);
    application->DestroyApplication(applicationHandle);
#ifdef SOCKET_FLAG1
    closesocket(clientSocket);
#endif
    WSACleanup();

    std::cout << "Program finished." << std::endl;
    
    return 0;
}
