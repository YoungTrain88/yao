#ifndef WIN_CLIENT_CPP_H
#define WIN_CLIENT_CPP_H

#include <winsock2.h>
#include <ws2tcpip.h>
#include <iostream>
#include <vector>
#include <string>

#pragma comment(lib, "ws2_32.lib")

// ÉùÃ÷ send2ros º¯Êý
int send2ros(const std::vector<double>& joint_angles);

int setting_init();

std::string serialize_joint_angles(const std::vector<double>& joint_angles);

#endif // WIN_CLIENT_CPP_H