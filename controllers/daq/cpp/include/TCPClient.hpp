#ifndef TCPCLIENT_HPP
#define TCPCLIENT_HPP

#include <string>
#include <netinet/in.h>

class TCPClient {
public:
    TCPClient(const std::string& address, int port);
    ~TCPClient();
    bool connectToServer();
    void sendData(const std::string& data);
    void closeConnection();

private:
    int sock;
    std::string address;
    int port;
    struct sockaddr_in server;
};

#endif // TCPCLIENT_HPP
