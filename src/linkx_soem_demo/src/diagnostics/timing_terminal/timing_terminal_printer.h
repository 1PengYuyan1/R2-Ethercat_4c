#ifndef TIMING_TERMINAL_PRINTER_H
#define TIMING_TERMINAL_PRINTER_H

#include <chrono>
#include <cstdint>

namespace timing_terminal
{
void WarnIfPeriodDrift(uint32_t tick,
                       std::chrono::steady_clock::time_point target);
}

#endif // TIMING_TERMINAL_PRINTER_H
