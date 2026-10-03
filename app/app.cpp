#include "app.h"
#include "platform.hpp"
#include "tx_api.h"
#include <cstdio>

namespace {
TX_THREAD startup_thread;
alignas(8) unsigned char stack[4096];

void run(ULONG) {
    platform::start_timer();
    auto previous = platform::time_us();
    for (unsigned long sequence = 0;; ++sequence) {
        tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND);
        const auto now = platform::time_us();
        char line[128];
        std::snprintf(line, sizeof(line), "self_flight USB seq=%lu cpu=%lu t=%lu.%06lus tick=%lu dt_us=%lu\r\n",
                      sequence, static_cast<unsigned long>(platform::cpu_hz()),
                      static_cast<unsigned long>(now / 1000000),
                      static_cast<unsigned long>(now % 1000000), tx_time_get(),
                      static_cast<unsigned long>(now - previous));
        platform::write(line);
        previous = now;
    }
}
}

unsigned int app_start(void) {
    return tx_thread_create(&startup_thread, const_cast<char*>("startup"), run, 0,
                            stack, sizeof(stack), 10, 10, TX_NO_TIME_SLICE, TX_AUTO_START);
}
