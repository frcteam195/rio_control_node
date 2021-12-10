#pragma once

#include <stdint.h>

enum class OVERRIDE_MODE {
    NORMAL_OPERATION = 0,
    TUNING_PIDS = 1,
    TEST_SYSTEM = 2
};

typedef struct OverrideModeStruct
{
    OVERRIDE_MODE overrideMode;
    uint32_t heartbeatTimeout;
};

static constexpr OverrideModeStruct NORMAL_OPERATION_MODE = {OVERRIDE_MODE::NORMAL_OPERATION, 0};
static constexpr uint32_t DEFAULT_OVERRIDE_TIMEOUT = 2; //2 Second default timeout