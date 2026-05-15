#ifndef HAPLY_CLIENT_H
#define HAPLY_CLIENT_H

#include "haply.h"
#include <nlohmann/json.hpp>
#include <string>
#include <iostream>

#ifdef _WIN32
    #define WIN32_LEAN_AND_MEAN
    #include <winsock2.h>
    #include <ws2tcpip.h>
    typedef SOCKET socket_t;
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <fcntl.h>
    typedef int socket_t;
    #define INVALID_SOCKET -1
#endif

class HaplyClient : public Haply {
public:
    HaplyClient() : connected(false), receiving_data(false),
                    x(0), y(0), z(0), qx(0), qy(0), qz(0), qw(1.0),
                    has_orientation(false), sock(INVALID_SOCKET) {}

    bool isConnected() { return connected; }
    bool hasOrientation() { return has_orientation; }
    bool hasPosition() { return receiving_data; }

    void connect() override {
#ifdef _WIN32
        WSADATA wsaData;
        WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
        sock = ::socket(AF_INET, SOCK_STREAM, 0);
        if (sock == INVALID_SOCKET) {
            std::cerr << "Failed to create socket" << std::endl;
            return;
        }

        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(10002);
        inet_pton(AF_INET, "127.0.0.1", &server_addr.sin_addr);

        if (::connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            std::cerr << "Failed to connect to Haply bridge" << std::endl;
            return;
        }


#ifdef _WIN32
        u_long mode = 1;
        ioctlsocket(sock, FIONBIO, &mode);
#else
        int flags = fcntl(sock, F_GETFL, 0);
        fcntl(sock, F_SETFL, flags | O_NONBLOCK);
#endif
        connected = true;
        std::cout << "Connected to Haply bridge" << std::endl;
    }

    void disconnect() override {
        connected = false;
#ifdef _WIN32
        closesocket(sock);
        WSACleanup();
#else
        ::close(sock);
#endif
        std::cout << "Disconnected from Haply bridge" << std::endl;
    }

    void update() {
        if (!connected) return;

        char buf[65536];
#ifdef _WIN32
        int bytes = recv(sock, buf, sizeof(buf) - 1, 0);
        if (bytes == SOCKET_ERROR) {
            int err = WSAGetLastError();
            if (err == WSAEWOULDBLOCK) return;  // no data available
            connected = false;
            return;
        }
#else
        int bytes = ::recv(sock, buf, sizeof(buf) - 1, 0);
        if (bytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) return;  // no data
            connected = false;
            return;
        }
#endif
        if (bytes == 0) { connected = false; return; }

        buf[bytes] = '\0';
        buffer += buf;

        size_t pos;
        while ((pos = buffer.find('\n')) != std::string::npos) {
            std::string msg = buffer.substr(0, pos);
            buffer = buffer.substr(pos + 1);
            parseMessage(msg);
        }
    }

    void getPosition(double& out_x, double& out_y, double& out_z) override {
        out_x = x; out_y = y; out_z = z;
    }

    void getOrientation(double& out_x, double& out_y, double& out_z, double& out_w) {
        out_x = qx; out_y = qy; out_z = qz; out_w = qw;
    }

    void sendForce(double fx, double fy, double fz) override {
        if (!connected) return;
        nlohmann::json msg = {
            {"force_command", {
                {"x", fx},
                {"y", fy},
                {"z", fz}
            }}
        };
        std::string msg_str = msg.dump() + "\n";
#ifdef _WIN32
        send(sock, msg_str.c_str(), msg_str.size(), 0);
#else
        ::send(sock, msg_str.c_str(), msg_str.size(), 0);
#endif
    }

private:
    bool connected;
    bool receiving_data;
    bool has_orientation;
    double x, y, z;
    double qx, qy, qz, qw;
    socket_t sock;
    std::string buffer;

    void parseMessage(const std::string& msg) {
        try {
            auto data = nlohmann::json::parse(msg);

            auto& inv3 = data["inverse3"];
            if (!inv3.empty()) {
                auto& state = inv3[0]["state"];
                if (state.contains("cursor_position")) {
                    auto& pos = state["cursor_position"];
                    x = pos["x"].get<double>();
                    y = pos["y"].get<double>();
                    z = pos["z"].get<double>();
                    receiving_data = true;
                }
            }

            auto& grip = data["wireless_verse_grip"];
            if (!grip.empty()) {
                auto& orientation = grip[0]["state"]["orientation"];
                qx = orientation["x"].get<double>();
                qy = orientation["y"].get<double>();
                qz = orientation["z"].get<double>();
                qw = orientation["w"].get<double>();
                has_orientation = true;
            }
        } catch (const std::exception& e) {
            std::cerr << "JSON parse error: " << e.what() << std::endl;
        }
    }
};

#endif