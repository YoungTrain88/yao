#include <winsock2.h>
#include <ws2tcpip.h> // ����inet_pton����ͷ�ļ�
#include <iostream>
#include <vector>
#include <string>
#include <sstream>

#pragma comment(lib, "ws2_32.lib") // ����Winsock��

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
	//// ��ʼ��WinSock
	//WSADATA wsaData;
	//WSAStartup(MAKEWORD(2, 2), &wsaData);

	//// ����Socket
	//SOCKET clientSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	//if (clientSocket == INVALID_SOCKET) {
	//	std::cerr << "Socket creation failed" << std::endl;
	//	return -1;
	//}

	//// ���÷�������ַ
	//sockaddr_in serverAddr;
	//serverAddr.sin_family = AF_INET;
	//serverAddr.sin_port = htons(12345); // �˿ں�

	//// ʹ��inet_pton�滻inet_addr
	//if (inet_pton(AF_INET, "192.168.31.34", &serverAddr.sin_addr) <= 0) {
	//	std::cerr << "Invalid IP address" << std::endl;
	//	closesocket(clientSocket);
	//	WSACleanup();
	//	return -1;
	//}

	//// ���ӷ�����
	//if (connect(clientSocket, (sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
	//	std::cerr << "Connection to server failed" << std::endl;
	//	closesocket(clientSocket);
	//	WSACleanup();
		return -1;
	//}

	//std::cout << "Connected to ROS server" << std::endl;

}



int send2ros(const std::vector<double>& joint_angles) {


	// ģ��ؽڽ����ݷ���
		//std::vector<double> joint_angles = { 0.5, -0.8, 1.0, -0.5, 1.3, -1.2 }; // ģ��ؽڽ�����

		//std::string message = serialize_joint_angles(joint_angles);

		//message += "\n";
		//// ��������
		////send(clientSocket, message.c_str(), message.length(), 0);

		//std::cout << "Sent joint angles: " << message << std::endl;

		//// �ȴ�1��
		//Sleep(1000);

	// �ر�Socket
	/*closesocket(clientSocket);
	WSACleanup();*/
		return 0;

}