// Phase 1: minimal vsomeip diagnostic client — requests DID 0xF190 (VIN)
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>
#include <vsomeip/vsomeip.hpp>

#if defined(__linux__) || defined(__QNX__)
#include <pthread.h>
#endif

static const vsomeip::service_t  SERVICE_ID  = 0x1234;
static const vsomeip::instance_t INSTANCE_ID = 0x0001;
static const vsomeip::method_t   METHOD_ID   = 0x0001; // ReadDataByIdentifier
static const uint16_t           DID_VIN     = 0xF190;

class diag_client {
public:
    diag_client()
        : rtm_(vsomeip::runtime::get()),
          app_(rtm_->create_application("diag_client")) {}

    bool init() {
        if (!app_->init()) {
            fprintf(stderr, "diag_client: init failed\n");
            return false;
        }
        app_->register_state_handler(
            [this](vsomeip::state_type_e st) { on_state(st); });
        app_->register_availability_handler(SERVICE_ID, INSTANCE_ID,
            [this](vsomeip::service_t s, vsomeip::instance_t i, bool avail) {
                on_avail(s, i, avail);
            });
        app_->register_message_handler(vsomeip::ANY_SERVICE, INSTANCE_ID, vsomeip::ANY_METHOD,
            [this](const std::shared_ptr<vsomeip::message>& msg) { on_message(msg); });
        return true;
    }

    void start() { app_->start(); }

    void stop() {
        app_->clear_all_handler();
        app_->release_service(SERVICE_ID, INSTANCE_ID);
        app_->stop();
    }

private:
    void on_state(vsomeip::state_type_e st) {
        if (st == vsomeip::state_type_e::ST_REGISTERED) {
            app_->request_service(SERVICE_ID, INSTANCE_ID);
            printf("diag_client: requested service 0x%04x\n", SERVICE_ID);
        }
    }

    void on_avail(vsomeip::service_t s, vsomeip::instance_t i, bool avail) {
        if (s == SERVICE_ID && i == INSTANCE_ID && avail) {
            printf("diag_client: service available, sending DID 0x%04x\n", DID_VIN);
            auto rq = rtm_->create_request();
            rq->set_service(SERVICE_ID);
            rq->set_instance(INSTANCE_ID);
            rq->set_method(METHOD_ID);
            auto pl = rtm_->create_payload();
            pl->set_data({static_cast<vsomeip::byte_t>(DID_VIN >> 8),
                          static_cast<vsomeip::byte_t>(DID_VIN & 0xFF)});
            rq->set_payload(pl);
            app_->send(rq);
        }
    }

    void on_message(const std::shared_ptr<vsomeip::message>& resp) {
        if (resp->get_service() != SERVICE_ID ||
            resp->get_instance() != INSTANCE_ID ||
            resp->get_message_type() != vsomeip::message_type_e::MT_RESPONSE ||
            resp->get_return_code() != vsomeip::return_code_e::E_OK) return;

        auto pl = resp->get_payload();
        const auto* data = pl->get_data();
        auto len = pl->get_length();

        if (len >= 2) {
            uint16_t did = (static_cast<uint16_t>(data[0]) << 8) | data[1];
            std::string hex_str;
            for (size_t i = 0; i < len; i++) {
                char buf[4];
                snprintf(buf, sizeof(buf), "%02x", data[i]);
                hex_str += buf;
                if (i < len - 1) hex_str += " ";
            }
            std::string ascii(data + 2, data + len);
            printf("Received DID 0x%04x: %s = %s\n", did, hex_str.c_str(), ascii.c_str());
        }
        stop();
    }

    std::shared_ptr<vsomeip::runtime> rtm_;
    std::shared_ptr<vsomeip::application> app_;
};

int main(int argc, char** argv) {
    (void)argc; (void)argv;

#if defined(__linux__) || defined(__QNX__)
    sigset_t sigs;
    sigemptyset(&sigs);
    sigaddset(&sigs, SIGINT);
    sigaddset(&sigs, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &sigs, nullptr);
#endif

    diag_client cli;
    if (!cli.init()) return 1;

#if defined(__linux__) || defined(__QNX__)
    std::thread signal_watcher([&cli]() {
        sigset_t wait_set;
        sigemptyset(&wait_set);
        sigaddset(&wait_set, SIGINT);
        sigaddset(&wait_set, SIGTERM);
        int sig = 0;
        while (sigwait(&wait_set, &sig) == 0) {
            if (sig == SIGINT || sig == SIGTERM) {
                cli.stop();
                return;
            }
        }
    });
#endif

    cli.start();

#if defined(__linux__) || defined(__QNX__)
    if (signal_watcher.joinable()) {
        pthread_cancel(signal_watcher.native_handle());
        signal_watcher.join();
    }
#endif
    return 0;
}
