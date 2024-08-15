#include "TCPClient.hpp"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

TCPClient::TCPClient(const std::string& addr, int prt) : address(addr), port(prt), sock(-1) {
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == -1) {
        std::cerr << "Could not create socket\n";
    }

    server.sin_addr.s_addr = inet_addr(address.c_str());
    server.sin_family = AF_INET;
    server.sin_port = htons(port);
}

TCPClient::~TCPClient() {
    closeConnection();
}

bool TCPClient::connectToServer() {
    if (connect(sock, (struct sockaddr *)&server, sizeof(server)) < 0) {
        std::cerr << "Connect failed. Error\n";
        return false;
    }
    std::cout << "Connected\n";
    return true;
}

void TCPClient::sendData(const std::string& data) {
    if (send(sock, data.c_str(), data.size(), 0) < 0) {
        std::cerr << "Send failed\n";
    }
}

void TCPClient::closeConnection() {
    if (sock != -1) {
        close(sock);
    }
}

