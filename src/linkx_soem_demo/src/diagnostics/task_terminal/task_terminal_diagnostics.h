#ifndef TASK_TERMINAL_DIAGNOSTICS_H
#define TASK_TERMINAL_DIAGNOSTICS_H

#include "linkx.h"

#include <cstdint>

class Class_Robot;

namespace task_terminal
{
void ParseEnvFlags();
void DispatchPeriodic(uint32_t tick, Class_Robot &robot, linkx_t *linkx);
void FlushStreams();
}

#endif // TASK_TERMINAL_DIAGNOSTICS_H
