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

/** Detect the tag: send the IV and decode the card type into instance->data. */
LegicPrimeError legic_prime_poller_setup(LegicPrimePoller* instance, uint8_t iv);

/**
 * Run one read session from a fresh field: handshake, then read every byte
 * not read yet.  Returns LegicPrimeErrorNone once all bytes are read; any
 * other error means the session ended early (reset the field and call again).
 */
LegicPrimeError legic_prime_poller_read(LegicPrimePoller* instance);

#ifdef __cplusplus
}
#endif
