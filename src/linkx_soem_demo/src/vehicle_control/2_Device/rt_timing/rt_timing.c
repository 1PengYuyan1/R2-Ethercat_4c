#include "rt_timing.h"
#include <inttypes.h>

void sync_absolute_time(struct timespec *ts, int cycle_ns) 
{
    ts->tv_nsec += cycle_ns;
    while (ts->tv_nsec >= 1000000000LL) {
        ts->tv_sec++;
        ts->tv_nsec -= 1000000000LL;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ts, NULL); 
}

void analyze_loop_frequency(struct timespec start_ts, int warn_threshold_ns) 
{
    static int loop_count = 0;
    static int64_t max_dt_ns = 0;
    struct timespec end_ts;
    
    clock_gettime(CLOCK_MONOTONIC, &end_ts);
    
    int64_t dt_ns = (int64_t)(end_ts.tv_sec - start_ts.tv_sec) * 1000000000LL + 
                    (end_ts.tv_nsec - start_ts.tv_nsec);
    
    if (dt_ns > max_dt_ns) max_dt_ns = dt_ns; 
    
    (void)warn_threshold_ns;
    
    if (++loop_count >= 1000) {
        max_dt_ns = 0; 
        loop_count = 0;
    }
}
