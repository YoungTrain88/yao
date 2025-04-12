#include <winsock2.h>
#include <ws2tcpip.h> // 包含inet_pton函数头文件
#include <iostream>
#include <vector>
#include <string>
#include <sstream>

#pragma comment(lib, "ws2_32.lib") // 加载Winsock库

//std::string serialize_joint_angles(const std::vector<double>& joint_angles) {
//	std::ostringstream oss;
//	for (size_t i = 0; i < joint_angles.size(); ++i) {
//		oss << joint_angles[i];
//		if (i != joint_angles.size() - 1) {
//			oss << ",";
//		}
//	}
//	return oss.str();
//}

int setting_init() {
	//// 初始化WinSock
	//WSADATA wsaData;
	//WSAStartup(MAKEWORD(2, 2), &wsaData);

	//// 创建Socket
	//SOCKET clientSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	//if (clientSocket == INVALID_SOCKET) {
	//	std::cerr << "Socket creation failed" << std::endl;
	//	return -1;
	//}

	//// 设置服务器地址
	//sockaddr_in serverAddr;
	//serverAddr.sin_family = AF_INET;
	//serverAddr.sin_port = htons(12345); // 端口号

	//// 使用inet_pton替换inet_addr
	//if (inet_pton(AF_INET, "192.168.31.34", &serverAddr.sin_addr) <= 0) {
	//	std::cerr << "Invalid IP address" << std::endl;
	//	closesocket(clientSocket);
	//	WSACleanup();
	//	return -1;
	//}

	//// 连接服务器
	//if (connect(clientSocket, (sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
	//	std::cerr << "Connection to server failed" << std::endl;
	//	closesocket(clientSocket);
	//	WSACleanup();
		return -1;
	//}

	//std::cout << "Connected to ROS server" << std::endl;

}



int send2ros(const std::vector<double>& joint_angles) {


	// 模拟关节角数据发送
		//std::vector<double> joint_angles = { 0.5, -0.8, 1.0, -0.5, 1.3, -1.2 }; // 模拟关节角数据

		//std::string message = serialize_joint_angles(joint_angles);

		//message += "\n";
		//// 发送数据
		////send(clientSocket, message.c_str(), message.length(), 0);

		//std::cout << "Sent joint angles: " << message << std::endl;

		//// 等待1秒
		//Sleep(1000);

	// 关闭Socket
	/*closesocket(clientSocket);
	WSACleanup();*/
		return 0;

}