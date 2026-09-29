// Phase 2/2b: vsomeip diagnostic service. The SOME/IP payload is the raw UDS
// request byte stream. Mode A (default): the in-process C UDS engine answers.
// Mode B (DIAG_TRANSPORT=can): the request is routed over ISO-TP/SocketCAN to
// the ECU on CAN (0x7E0) and the ECU's response is relayed verbatim.
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>
#include <vsomeip/vsomeip.hpp>

#include "diag_bridge.h"
#include "transport/socketcan_transport.h"

#if defined(__linux__) || defined(__QNX__)
#include <pthread.h>
#endif

// Diagnostic IDs
static const vsomeip::service_t  SERVICE_ID  = 0x1234;
static const vsomeip::instance_t INSTANCE_ID = 0x0001;
static const vsomeip::method_t   METHOD_ID   = 0x0001; // UDS passthrough

// Mode B: ECU round-trip budget. P2=50ms and N_Cr/N_Bs=1000ms fit with margin.
static const uint32_t CAN_REQ_TIMEOUT_MS = 2000;

class diag_service {
public:
    diag_service()
        : rtm_(vsomeip::runtime::get()),
          app_(rtm_->create_application("diag_service")),
          stop_(false) {
        stop_thread_ = std::thread{&diag_service::shutdown, this};
    }

    ~diag_service() {
        if (std::this_thread::get_id() != stop_thread_.get_id()) {
            if (stop_thread_.joinable()) stop_thread_.join();
        } else {
            stop_thread_.detach();
        }
    }

    bool init() {
        if (!app_->init()) {
            fprintf(stderr, "diag_service: init failed\n");
            return false;
        }
        // Transport selection (once): DIAG_TRANSPORT=can routes UDS over
        // ISO-TP/SocketCAN (Mode B); anything else uses the in-process UDS
        // engine (Mode A, default). DIAG_CAN_IF selects the interface.
        const char* transport = getenv("DIAG_TRANSPORT");
        use_can_ = (transport != nullptr && strcmp(transport, "can") == 0);
        if (use_can_) {
            const char* iface = getenv("DIAG_CAN_IF");
            if (iface == nullptr) iface = "can0";
            if (can_transport_init(iface) != 0) {
                fprintf(stderr, "diag_service: CAN transport init failed on %s\n", iface);
                return false;  // explicit opt-in must work; no silent fallback
            }
            printf("diag_service: Mode B (CAN transport, iface %s)\n", iface);
        } else {
            // Bring up the in-process UDS engine (session manager + dispatcher).
            diag_init();
        }
        app_->register_message_handler(SERVICE_ID, INSTANCE_ID, METHOD_ID,
            [this](const std::shared_ptr<vsomeip::message>& req) { on_message(req); });
        app_->register_state_handler(
            [this](vsomeip::state_type_e st) { on_state(st); });
        return true;
    }

    void start() { app_->start(); }

    void terminate() {
        std::scoped_lock lk(mutex_);
        stop_ = true;
        cond_.notify_one();
    }

private:
    void on_state(vsomeip::state_type_e st) {
        if (st == vsomeip::state_type_e::ST_REGISTERED) {
            app_->offer_service(SERVICE_ID, INSTANCE_ID);
            printf("diag_service: offering 0x%04x\n", SERVICE_ID);
        }
    }

    // UDS passthrough: the SOME/IP payload IS the raw UDS request.
    void on_message(const std::shared_ptr<vsomeip::message>& req) {
        auto pl = req->get_payload();
        const auto* data = pl->get_data();
        auto len = pl->get_length();

        uint8_t resp_buf[DIAG_RESP_MAX];
        size_t resp_len = 0;
        if (use_can_) {
            // Mode B: route the request over ISO-TP/SocketCAN to the ECU.
            if (can_transport_request(data, len, resp_buf, DIAG_RESP_MAX,
                                      &resp_len, CAN_REQ_TIMEOUT_MS) < 0) {
                resp_len = 0;  // timeout/error -> shared 0x7F fallback below
            }
        } else {
            // Mode A: run the in-process C UDS dispatcher.
            diag_dispatch(data, len, resp_buf, &resp_len);
        }

        std::vector<vsomeip::byte_t> resp_data;
        if (resp_len > 0) {
            resp_data.assign(resp_buf, resp_buf + resp_len);
        } else if (len > 0) {
            // Dispatcher produced no response (e.g. TesterPresent suppress, or
            // functional suppression) -> negative response service-not-supported.
            resp_data = {0x7F, data[0], 0x11};
        } else {
            resp_data = {0x7F, 0x00, 0x11};
        }

        auto resp = rtm_->create_response(req);
        auto resp_pl = rtm_->create_payload();
        resp_pl->set_data(resp_data);
        resp->set_payload(resp_pl);
        app_->send(resp);
        printf("diag_service: UDS response sent (%zu bytes)\n", resp_len);
    }

    // Graceful shutdown (called from stop_thread_)
    void shutdown() {
        std::unique_lock lk(mutex_);
        cond_.wait(lk, [this] { return stop_; });
        app_->stop_offer_service(SERVICE_ID, INSTANCE_ID);
        app_->clear_all_handler();
        app_->stop();
    }

    std::shared_ptr<vsomeip::runtime>  rtm_;
    std::shared_ptr<vsomeip::application> app_;
    bool use_can_ = false;
    bool stop_;
    std::mutex mutex_;
    std::condition_variable cond_;
    std::thread stop_thread_;
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

    diag_service srv;
    if (!srv.init()) return 1;

#if defined(__linux__) || defined(__QNX__)
    std::thread signal_watcher([&srv]() {
        sigset_t wait_set;
        sigemptyset(&wait_set);
        sigaddset(&wait_set, SIGINT);
        sigaddset(&wait_set, SIGTERM);
        int sig = 0;
        while (sigwait(&wait_set, &sig) == 0) {
            if (sig == SIGINT || sig == SIGTERM) {
                srv.terminate();
                return;
            }
        }
    });
#endif

    srv.start();

#if defined(__linux__) || defined(__QNX__)
    if (signal_watcher.joinable()) {
        pthread_cancel(signal_watcher.native_handle());
        signal_watcher.join();
    }
#endif
    return 0;
}
