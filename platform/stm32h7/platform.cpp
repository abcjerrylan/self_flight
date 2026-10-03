#include "platform.hpp"
#include "main.h"
#include "usb_log.h"
#include "ux_api.h"
#include "ux_device_class_cdc_acm.h"
#include <cstring>

extern "C" {
extern TIM_HandleTypeDef htim2;
}

namespace {
UX_SLAVE_CLASS_CDC_ACM* volatile usb = nullptr;
}

void usb_log_attach(void* instance) {
    auto* device = static_cast<UX_SLAVE_CLASS_CDC_ACM*>(instance);
    if (device) {
        ux_device_class_cdc_acm_ioctl(device, UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_WRITE_TIMEOUT,
                                      reinterpret_cast<VOID*>(100));
    }
    usb = device;
}

namespace platform {
void start_timer() {
    if (HAL_TIM_Base_Start(&htim2) != HAL_OK) Error_Handler();
}

std::uint32_t cpu_hz() { return HAL_RCC_GetSysClockFreq(); }

std::uint64_t time_us() {
    static std::uint32_t previous = 0;
    static std::uint64_t high = 0;
    const auto count = __HAL_TIM_GET_COUNTER(&htim2);
    if (count < previous) high += (std::uint64_t{1} << 32);
    previous = count;
    return high + count;
}

void write(const char* text) {
    auto* device = usb;
    if (!device || !device->ux_slave_class_cdc_acm_data_dtr_state) return;
    ULONG sent = 0;
    auto* data = const_cast<UCHAR*>(reinterpret_cast<const UCHAR*>(text));
    if (ux_device_class_cdc_acm_write(device, data, std::strlen(text), &sent) != UX_SUCCESS) {
        const auto interrupts = tx_interrupt_control(TX_INT_DISABLE);
        if (usb == device) { // A USB reset may already have removed the endpoints.
            ux_device_class_cdc_acm_ioctl(device, UX_SLAVE_CLASS_CDC_ACM_IOCTL_ABORT_PIPE,
                                          reinterpret_cast<VOID*>(UX_SLAVE_CLASS_CDC_ACM_ENDPOINT_XMIT));
            usb = nullptr; // Resume on USB reconnection after a failed transfer.
        }
        tx_interrupt_control(interrupts);
    }
}
}
