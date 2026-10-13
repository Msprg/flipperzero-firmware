#pragma once

#include "legic_prime_poller.h"

#include <nfc/nfc_poller.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LEGIC_PRIME_POLLER_MAX_BUFFER_SIZE (8U)

// FWT (frame wait time) is the window the chip listens for tag response
// after end of reader TX. LEGIC's TAG_FRAME_WAIT is 330 µs (the moment the
// response begins). Bump to ~2.5 ms so the entire 6-bit tag response (≈600µs
// at TAG_BIT_PERIOD=100µs) lands inside the window.
#define LEGIC_PRIME_FWT_FC   (33900U) // ≈ 2500 µs
#define LEGIC_PRIME_GUARD_US (5000U)
// Frame timing is scheduled explicitly (legic_prime_poller_schedule), so the
// generic inter-frame blocking timers are kept short.
#define LEGIC_PRIME_FDT_POLL_FC  (500U)
#define LEGIC_PRIME_POLL_POLL_US (300U)

typedef enum {
    LegicPrimePollerStateSetup,
    LegicPrimePollerStateRequestMode,
    LegicPrimePollerStateRead,
    LegicPrimePollerStateResume,
    LegicPrimePollerStateSuccess,
    LegicPrimePollerStateFailure,

    LegicPrimePollerStateNum,
} LegicPrimePollerState;

struct LegicPrimePoller {
    Nfc* nfc;
    LegicPrimePollerState state;
    LegicPrimeData* data;
    BitBuffer* tx_buffer;
    BitBuffer* rx_buffer;

    uint16_t read_addr; // running read pointer
    uint8_t resumes; // field resets since the read last made progress
    uint16_t resume_addr; // read_addr at the last resume

    // PRNG bookkeeping: cmd_size and addr_size cached from the detected type.
    uint8_t cmd_size_bits;
    uint8_t addr_size_bits;
    // Length of the tag reply to the last frame we sent (0 = none), used to
    // schedule the next frame against the tag's idle keystream steps.
    uint8_t last_reply_bits;

    NfcGenericEvent general_event;
    LegicPrimePollerEvent legic_event;
    LegicPrimePollerEventData legic_event_data;
    NfcGenericCallback callback;
    void* context;
};

const LegicPrimeData* legic_prime_poller_get_data(LegicPrimePoller* instance);

#ifdef __cplusplus
}
#endif
