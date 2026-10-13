#pragma once

#include "legic_prime.h"
#include <lib/nfc/nfc.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct LegicPrimePoller LegicPrimePoller;

typedef enum {
    LegicPrimePollerEventTypeRequestMode,
    LegicPrimePollerEventTypeSuccess,
    LegicPrimePollerEventTypeFailure,
} LegicPrimePollerEventType;

typedef enum {
    LegicPrimePollerModeRead,

    LegicPrimePollerModeNum,
} LegicPrimePollerMode;

typedef struct {
    LegicPrimePollerMode mode;
} LegicPrimePollerEventDataModeRequest;

typedef union {
    LegicPrimePollerEventDataModeRequest mode_request;
    LegicPrimeError error;
} LegicPrimePollerEventData;

typedef struct {
    LegicPrimePollerEventType type;
    LegicPrimePollerEventData* data;
} LegicPrimePollerEvent;

/** Run the full setup phase (IV, type, ACK). On success populates the type. */
LegicPrimeError legic_prime_poller_setup(LegicPrimePoller* instance, uint8_t iv);

/** Read one byte at the given address. */
LegicPrimeError
    legic_prime_poller_read_byte(LegicPrimePoller* instance, uint16_t addr, uint8_t* value);

/** Read the entire memory image into instance->data. */
LegicPrimeError legic_prime_poller_read(LegicPrimePoller* instance);

#ifdef __cplusplus
}
#endif
