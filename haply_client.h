#ifndef HAPLY_CLIENT_H
#define HAPLY_CLIENT_H

#include "haply.h"
#include <libwebsockets.h>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>
#include <cstring>

class HaplyClient : public Haply {
    public:
        HaplyClient() : connected(false), x(0), y(0), z(0), 
                        context(nullptr), wsi(nullptr) {}

        bool isConnected() {
            return connected;
        }

        void connect() override {
            struct lws_context_creation_info info;
            memset(&info, 0, sizeof(info));
            info.port = CONTEXT_PORT_NO_LISTEN;
            info.protocols = protocols;
            info.user = this;

            context = lws_create_context(&info);
            if (!context) {
                std::cerr << "Failed to create WebSocket context" << std::endl;
                return;
            }

            struct lws_client_connect_info ccinfo;
            memset(&ccinfo, 0, sizeof(ccinfo));
            ccinfo.context = context;
            ccinfo.address = "localhost";
            ccinfo.port = 10001;
            ccinfo.path = "/";
            ccinfo.host = "localhost";
            ccinfo.origin = "localhost";
            ccinfo.protocol = protocols[0].name;
            ccinfo.userdata = this;

            wsi = lws_client_connect_via_info(&ccinfo);
            if (!wsi) {
                std::cerr << "Failed to connect to Haply service" << std::endl;
                return;
            }

            // Wait for connection to establish
            for (int i = 0; i < 50 && !connected; i++) {
                lws_service(context, 10);
            }

            // Start background thread
            recv_thread = std::thread(&HaplyClient::serviceLoop, this);
            std::cout << "Connected to Haply service" << std::endl;
        }

        void disconnect() override {
            connected = false;
            if (recv_thread.joinable())
                recv_thread.join();
            if (context)
                lws_context_destroy(context);
            std::cout << "Disconnected from Haply service" << std::endl;
        }

        void getPosition(double& out_x, double& out_y, double& out_z) override {
            std::lock_guard<std::mutex> lock(data_mutex);
            out_x = x;
            out_y = y;
            out_z = z;
        }

        void getOrientation(double& out_x, double& out_y, double& out_z, double& out_w) {
            std::lock_guard<std::mutex> lock(data_mutex);
            out_x = qx;
            out_y = qy;
            out_z = qz;
            out_w = qw;
            std::cout << "Orientation: " << qx << " " << qy << " " << qz << " " << qw << std::endl;
        }

        void sendForce(double fx, double fy, double fz) override {
            if (!wsi || !connected) return;
            
            // Build force JSON
            nlohmann::json force_msg = {
                {"inverse3", {{
                    {"device_id", device_id},
                    {"force", {{"x", fx}, {"y", fy}, {"z", fz}}}
                }}}
            };

            std::string msg = force_msg.dump();
            // libwebsockets requires LWS_PRE bytes before the payload
            std::vector<unsigned char> buf(LWS_PRE + msg.size());
            memcpy(buf.data() + LWS_PRE, msg.c_str(), msg.size());
            lws_write(wsi, buf.data() + LWS_PRE, msg.size(), LWS_WRITE_TEXT);
        }

        // Called by libwebsockets callback
        void onMessage(const std::string& msg) {
            try {
                auto data = nlohmann::json::parse(msg);
                auto& inv3 = data["inverse3"];
                if (!inv3.empty()) {
                    receiving_data = true;
                    auto& state = inv3[0]["state"];
                    auto& cursor = state["cursor"]["position"];
                    std::lock_guard<std::mutex> lock(data_mutex);
                    x = cursor["x"].get<double>();
                    y = cursor["y"].get<double>();
                    z = cursor["z"].get<double>();
                    device_id = inv3[0]["device_id"].get<std::string>();
                }
                auto& grip = data["wireless_verse_grip"];
                if (!grip.empty()) {
                    auto& orientation = grip[0]["state"]["orientation"];
                    std::lock_guard<std::mutex> lock(data_mutex);
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

    private:
        std::atomic<bool> connected;
        std::atomic<bool> receiving_data{false};
        std::thread recv_thread;
        std::mutex data_mutex;
        double x, y, z;
        double qx, qy, qz, qw;
        bool has_orientation{false};
        std::string device_id;
        struct lws_context* context;
        struct lws* wsi;

        void serviceLoop() {
            while (connected) {
                lws_service(context, 10);
            }
        }

        // libwebsockets callback
        static int callback(struct lws* wsi, enum lws_callback_reasons reason,
                   void* user, void* in, size_t len) {
            HaplyClient* client = static_cast<HaplyClient*>(lws_context_user(lws_get_context(wsi)));

            switch (reason) {
                case LWS_CALLBACK_CLIENT_ESTABLISHED:
                    if (client) client->connected = true;
                    std::cout << "WebSocket connection established" << std::endl;
                    break;
                case LWS_CALLBACK_CLIENT_RECEIVE:
                    if (client && in && len > 0) {
                        client->onMessage(std::string((char*)in, len));
                    }
                    break;
                case LWS_CALLBACK_CLIENT_CONNECTION_ERROR:
                    std::cerr << "WebSocket connection error" << std::endl;
                    if (client) client->connected = false;
                    break;
                default:
                    break;
            }
            return 0;
        }

        static const struct lws_protocols protocols[];
};

const struct lws_protocols HaplyClient::protocols[] = {
    {"haply-protocol", HaplyClient::callback, 0, 65536, 0, nullptr, 0},
    {nullptr, nullptr, 0, 0, 0, nullptr, 0}
};

#endif