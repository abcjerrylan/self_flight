#include "platform.hpp"
#include "main.h"
#include "usb_log.h"
#include "ux_api.h"
#include "ux_device_class_cdc_acm.h"
#include "imu.hpp"
#include "rc.hpp"
#include <cstring>

extern "C" {
extern TIM_HandleTypeDef htim2;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart6;
}

namespace {
UX_SLAVE_CLASS_CDC_ACM* volatile usb = nullptr;
std::uint8_t receiver_byte;
bool spi_transfer(void*, self_flight::bmi088::Sensor sensor, const std::uint8_t* tx,
                  std::uint8_t* rx, std::size_t count) {
    const auto pin = sensor == self_flight::bmi088::Sensor::Accel ? BMI088_A_CS_Pin : BMI088_G_CS_Pin;
    HAL_GPIO_WritePin(GPIOD, pin, GPIO_PIN_RESET);
    const auto status = HAL_SPI_TransmitReceive(&hspi2, const_cast<std::uint8_t*>(tx), rx,
                                                static_cast<std::uint16_t>(count), 2);
    HAL_GPIO_WritePin(GPIOD, pin, GPIO_PIN_SET);
    return status == HAL_OK;
}
void delay_ms(void*, unsigned ms) { tx_thread_sleep(ms + 1); }
}

extern "C" void HAL_GPIO_EXTI_Callback(std::uint16_t pin) {
    if (pin == BMI088_A_DRDY_Pin) self_flight::imu::drdy(self_flight::bmi088::Sensor::Accel);
    if (pin == BMI088_G_DRDY_Pin) self_flight::imu::drdy(self_flight::bmi088::Sensor::Gyro);
}
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* uart) {
    if (uart!=&huart6 || uart->ErrorCode!=HAL_UART_ERROR_NONE) return;
    self_flight::rc::received(receiver_byte);
    if (!platform::start_receiver()) self_flight::rc::received(0,0x80000000U);
}
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* uart) {
    if (uart!=&huart6) return;
    const auto error=uart->ErrorCode;
    HAL_UART_AbortReceive(uart);
    self_flight::rc::received(0,error);
    if (!platform::start_receiver()) self_flight::rc::received(0,0x80000000U);
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
CriticalSection::CriticalSection() : interrupts(__get_PRIMASK()) { __disable_irq(); }
CriticalSection::~CriticalSection() { __set_PRIMASK(interrupts); }
self_flight::bmi088::Bus imu_bus() { return {nullptr, spi_transfer, delay_ms}; }
void start_timer() {
    if (HAL_TIM_Base_Start(&htim2) != HAL_OK) Error_Handler();
}
bool start_receiver() { return HAL_UART_Receive_IT(&huart6,&receiver_byte,1)==HAL_OK; }

std::uint32_t cpu_hz() { return HAL_RCC_GetSysClockFreq(); }

std::uint64_t time_us() {
    const CriticalSection lock;
    static std::uint32_t previous = 0;
    static std::uint64_t high = 0;
    const auto count = __HAL_TIM_GET_COUNTER(&htim2);
    if (count < previous) high += (std::uint64_t{1} << 32);
    previous = count;
    return high + count;
}

bool write(const char* text) {
    auto* device = usb;
    if (!device || !device->ux_slave_class_cdc_acm_data_dtr_state) return false;
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
        return false;
    }
    return sent == std::strlen(text);
}
}
