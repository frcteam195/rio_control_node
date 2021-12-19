#pragma once

#include <stdint.h>

enum class OVERRIDE_MODE {
    NORMAL_OPERATION = 0,
    TUNING_PIDS = 1,
    TEST_SYSTEM = 2
};

struct OverrideModeStruct
{
    OVERRIDE_MODE overrideMode;
    uint32_t heartbeatTimeout;
};

static constexpr OverrideModeStruct NORMAL_OPERATION_MODE = {OVERRIDE_MODE::NORMAL_OPERATION, 0};

//Frequency of update in Hz
#define OVERRIDE_HEARTBEAT_RATE 10
static constexpr uint32_t DEFAULT_OVERRIDE_TIMEOUT = (0.5 * OVERRIDE_HEARTBEAT_RATE); //0.5 Second default timeout