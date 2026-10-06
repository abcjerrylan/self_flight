#include "rc.hpp"
#include "flight.hpp"
#include "platform.hpp"
#include "tx_api.h"
#include "receiver-profile.hpp"

namespace self_flight::rc {
namespace {
struct Event { std::uint64_t time; std::uint32_t byte, error; };
static_assert(sizeof(ULONG)==4 && sizeof(Event)==16);
TX_QUEUE queue;
TX_THREAD thread;
alignas(8) Event storage[128];
alignas(8) unsigned char stack[2048];
Statistics stats;
bool overflow=false;
void run(ULONG) {
    for (;;) {
        Event event;
        if (tx_queue_receive(&queue,&event,TX_WAIT_FOREVER)!=TX_SUCCESS) continue;
        bool dropped;
        {
            const platform::CriticalSection lock;
            dropped=overflow;
            if (dropped) { tx_queue_flush(&queue); overflow=false; }
        }
        if (dropped || event.error) { flight::reset_sbus(); continue; }
        flight::receive_sbus_byte(static_cast<std::uint8_t>(event.byte),event.time);
    }
}
}
unsigned start() {
    flight::configure_sbus(board::receiver_channels);
    auto result=tx_queue_create(&queue,const_cast<char*>("sbus_bytes"),4,storage,sizeof(storage));
    if (result!=TX_SUCCESS) return result;
    result=tx_thread_create(&thread,const_cast<char*>("sbus"),run,0,stack,sizeof(stack),
                            9,9,TX_NO_TIME_SLICE,TX_AUTO_START);
    if (result!=TX_SUCCESS) return result;
    return platform::start_receiver() ? TX_SUCCESS : TX_NOT_AVAILABLE;
}
void received(std::uint8_t byte, std::uint32_t error) {
    Event event{platform::time_us(),byte,error};
    const platform::CriticalSection lock;
    if (error) ++stats.errors; else ++stats.bytes;
    if (tx_queue_send(&queue,&event,TX_NO_WAIT)!=TX_SUCCESS) {
        ++stats.drops; overflow=true;
    }
    if (error) ++stats.restarts;
}
Statistics statistics() {
    const platform::CriticalSection lock;
    return stats;
}
}
