#include "bsp_can.h"

#include <array>
#include <cstring>
#include <deque>
#include <mutex>

#include "pub_user.h"

namespace {

struct RxFrame {
    uint16_t can_id;
    std::array<uint8_t, 8> data;
};

struct SdkState {
    damiao_handle* handle = nullptr;
    device_handle* dev = nullptr;
    uint8_t channel = 1;
    std::mutex rx_mutex;
    std::deque<RxFrame> rx_queue;
};

SdkState g_state;

void rec_callback(usb_rx_frame_t* frame) {
    if (!frame) {
        return;
    }
    if (frame->head.channel != g_state.channel) {
        return;
    }
    RxFrame copy{};
    copy.can_id = static_cast<uint16_t>(frame->head.can_id & 0x7FFU);
    const size_t copy_len = frame->head.dlc < copy.data.size() ? frame->head.dlc : copy.data.size();
    std::memcpy(copy.data.data(), frame->payload, copy_len);

    std::lock_guard<std::mutex> lock(g_state.rx_mutex);
    g_state.rx_queue.push_back(copy);
    if (g_state.rx_queue.size() > 1024) {
        g_state.rx_queue.pop_front();
    }
}

void err_callback(usb_rx_frame_t* frame) {
    (void)frame;
}

}  // namespace

extern "C" {

hcan_t hcan1 = {1};

bool canx_open(hcan_t* hcan, uint8_t channel, int can_baud, int canfd_baud) {
    if (!hcan || g_state.dev) {
        return false;
    }

    g_state.handle = damiao_handle_create(DEV_USB2CANFD_DUAL);
    if (!g_state.handle) {
        return false;
    }

    if (damiao_handle_find_devices(g_state.handle) <= 0) {
        damiao_handle_destroy(g_state.handle);
        g_state.handle = nullptr;
        return false;
    }

    device_handle* dev_list[16] = {0};
    int device_count = 0;
    damiao_handle_get_devices(g_state.handle, dev_list, &device_count);
    if (device_count <= 0 || !dev_list[0]) {
        damiao_handle_destroy(g_state.handle);
        g_state.handle = nullptr;
        return false;
    }

    g_state.dev = dev_list[0];
    if (!device_open(g_state.dev)) {
        g_state.dev = nullptr;
        damiao_handle_destroy(g_state.handle);
        g_state.handle = nullptr;
        return false;
    }

    g_state.channel = channel;
    hcan->channel = channel;

    if (!device_channel_set_baud_with_sp(g_state.dev, channel, true, can_baud, canfd_baud, 0.75f, 0.75f) ||
        !device_open_channel(g_state.dev, channel)) {
        device_close(g_state.dev);
        g_state.dev = nullptr;
        damiao_handle_destroy(g_state.handle);
        g_state.handle = nullptr;
        return false;
    }

    device_hook_to_rec(g_state.dev, rec_callback);
    device_hook_to_err(g_state.dev, err_callback);
    return true;
}

void canx_close(hcan_t* hcan) {
    if (!hcan) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(g_state.rx_mutex);
        g_state.rx_queue.clear();
    }

    if (g_state.dev) {
        device_close_channel(g_state.dev, hcan->channel);
        device_close(g_state.dev);
        g_state.dev = nullptr;
    }

    if (g_state.handle) {
        damiao_handle_destroy(g_state.handle);
        g_state.handle = nullptr;
    }
}

void canx_send_data(hcan_t* hcan, uint16_t can_id, const uint8_t* data, uint8_t len) {
    if (!hcan || !data || !g_state.dev) {
        return;
    }

    device_channel_send_fast(
        g_state.dev,
        hcan->channel,
        static_cast<uint32_t>(can_id),
        1,
        false,
        true,
        true,
        len,
        const_cast<uint8_t*>(data));
}

void canx_receive(hcan_t* hcan, uint16_t* can_id, uint8_t* data) {
    (void)hcan;
    if (!can_id || !data) {
        return;
    }

    std::memset(data, 0, 8);
    *can_id = 0;

    std::lock_guard<std::mutex> lock(g_state.rx_mutex);
    if (g_state.rx_queue.empty()) {
        return;
    }

    const RxFrame frame = g_state.rx_queue.front();
    g_state.rx_queue.pop_front();
    *can_id = frame.can_id;
    std::memcpy(data, frame.data.data(), frame.data.size());
}

size_t canx_pending(const hcan_t* hcan) {
    (void)hcan;
    std::lock_guard<std::mutex> lock(g_state.rx_mutex);
    return g_state.rx_queue.size();
}

}  // extern "C"
