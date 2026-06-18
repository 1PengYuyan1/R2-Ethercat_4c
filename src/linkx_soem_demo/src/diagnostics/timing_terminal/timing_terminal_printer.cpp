#include "timing_terminal_printer.h"

#include <cstdio>

namespace
{
constexpr uint32_t kPeriodDriftWarnSkipFirst = 500;
constexpr int64_t kPeriodDriftWarnUs = 1500;
}

void timing_terminal::WarnIfPeriodDrift(uint32_t tick,
                                        std::chrono::steady_clock::time_point target)
{
    if (tick < kPeriodDriftWarnSkipFirst)
        return;

    const auto now = std::chrono::steady_clock::now();
    const auto drift_us =
        std::chrono::duration_cast<std::chrono::microseconds>(now - target).count();
    if (drift_us > kPeriodDriftWarnUs)
    {
        std::fprintf(stderr, "[TASK][WARN] tick=%u drift=%lldus\n",
                     static_cast<unsigned>(tick),
                     static_cast<long long>(drift_us));
    }
}
