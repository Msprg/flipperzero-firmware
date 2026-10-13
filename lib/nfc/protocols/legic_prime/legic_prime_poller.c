#include "legic_prime_poller.h"
#include "legic_prime_poller_i.h"

#include <nfc/protocols/nfc_poller_base.h>

#define TAG "LegicPrimePoller"

typedef NfcCommand (*LegicPrimePollerStateHandler)(LegicPrimePoller* instance);

const LegicPrimeData* legic_prime_poller_get_data(LegicPrimePoller* instance) {
    furi_assert(instance);
    furi_assert(instance->data);
    return instance->data;
}

static LegicPrimePoller* legic_prime_poller_alloc(Nfc* nfc) {
    furi_assert(nfc);

    LegicPrimePoller* instance = malloc(sizeof(LegicPrimePoller));
    instance->nfc = nfc;
    instance->state = LegicPrimePollerStateSetup;
    instance->tx_buffer = bit_buffer_alloc(LEGIC_PRIME_POLLER_MAX_BUFFER_SIZE);
    instance->rx_buffer = bit_buffer_alloc(LEGIC_PRIME_POLLER_MAX_BUFFER_SIZE);

    nfc_config(instance->nfc, NfcModePoller, NfcTechLegic);
    nfc_set_guard_time_us(instance->nfc, LEGIC_PRIME_GUARD_US);
    nfc_set_fdt_poll_fc(instance->nfc, LEGIC_PRIME_FDT_POLL_FC);
    nfc_set_fdt_poll_poll_us(instance->nfc, LEGIC_PRIME_POLL_POLL_US);

    instance->data = legic_prime_alloc();

    instance->legic_event.data = &instance->legic_event_data;
    instance->general_event.protocol = NfcProtocolLegicPrime;
    instance->general_event.event_data = &instance->legic_event;
    instance->general_event.instance = instance;

    return instance;
}

static void legic_prime_poller_free(LegicPrimePoller* instance) {
    furi_assert(instance);
    bit_buffer_free(instance->tx_buffer);
    bit_buffer_free(instance->rx_buffer);
    legic_prime_free(instance->data);
    free(instance);
}

static void legic_prime_poller_set_callback(
    LegicPrimePoller* instance,
    NfcGenericCallback callback,
    void* context) {
    furi_assert(instance);
    furi_assert(callback);
    instance->callback = callback;
    instance->context = context;
}

static NfcCommand legic_prime_poller_setup_handler(LegicPrimePoller* instance) {
    // IV value matches Proxmark3's default (cmdhflegic.c). The LEGIC PRNG
    // requires IV to be odd; 0x55 is the empirically-known working value.
    LegicPrimeError err = legic_prime_poller_setup(instance, 0x55);
    if(err != LegicPrimeErrorNone) {
        instance->state = LegicPrimePollerStateFailure;
        instance->legic_event_data.error = err;
    } else {
        instance->state = LegicPrimePollerStateRequestMode;
    }
    return NfcCommandContinue;
}

static NfcCommand legic_prime_poller_request_mode_handler(LegicPrimePoller* instance) {
    instance->legic_event.type = LegicPrimePollerEventTypeRequestMode;
    NfcCommand cmd = instance->callback(instance->general_event, instance->context);
    // Only mode currently is Read.
    instance->state = LegicPrimePollerStateRead;
    instance->read_addr = 0;
    instance->resumes = 0;
    instance->resume_addr = 0;
    return cmd;
}

// A tag that misses a frame drops its session and waits for a new IV; a
// field reset plus a fresh handshake lets the read continue where it stopped.
#define LEGIC_PRIME_MAX_RESUMES (8U)

static NfcCommand legic_prime_poller_read_handler(LegicPrimePoller* instance) {
    NfcCommand command = NfcCommandContinue;
    LegicPrimeError err = legic_prime_poller_read(instance);
    // Only resumes that make no progress count against the limit.
    if(instance->read_addr != instance->resume_addr) {
        instance->resume_addr = instance->read_addr;
        instance->resumes = 0;
    }
    if(err == LegicPrimeErrorNone) {
        instance->state = LegicPrimePollerStateSuccess;
    } else if(err != LegicPrimeErrorWrongCrc && instance->resumes < LEGIC_PRIME_MAX_RESUMES) {
        instance->resumes++;
        instance->state = LegicPrimePollerStateResume;
        command = NfcCommandReset;
    } else {
        instance->state = LegicPrimePollerStateFailure;
        instance->legic_event_data.error = err;
    }
    return command;
}

static NfcCommand legic_prime_poller_resume_handler(LegicPrimePoller* instance) {
    NfcCommand command = NfcCommandContinue;
    const LegicPrimeType type = instance->data->type;
    const uint16_t data_size = instance->data->data_size;
    const uint8_t cmd_size_bits = instance->cmd_size_bits;
    const uint8_t addr_size_bits = instance->addr_size_bits;
    LegicPrimeError err = legic_prime_poller_setup(instance, instance->data->iv);
    if(err == LegicPrimeErrorNone && instance->data->type == type) {
        instance->state = LegicPrimePollerStateRead;
    } else if(instance->resumes < LEGIC_PRIME_MAX_RESUMES) {
        // Card type must not change mid-read; a different one is a misdecode.
        instance->data->type = type;
        instance->data->data_size = data_size;
        instance->cmd_size_bits = cmd_size_bits;
        instance->addr_size_bits = addr_size_bits;
        instance->resumes++;
        command = NfcCommandReset;
    } else {
        instance->state = LegicPrimePollerStateFailure;
        instance->legic_event_data.error = err;
    }
    return command;
}

static NfcCommand legic_prime_poller_success_handler(LegicPrimePoller* instance) {
    instance->legic_event.type = LegicPrimePollerEventTypeSuccess;
    NfcCommand cmd = instance->callback(instance->general_event, instance->context);
    furi_delay_ms(100);
    instance->state = LegicPrimePollerStateSetup;
    return cmd;
}

static NfcCommand legic_prime_poller_failure_handler(LegicPrimePoller* instance) {
    instance->legic_event.type = LegicPrimePollerEventTypeFailure;
    NfcCommand cmd = instance->callback(instance->general_event, instance->context);
    furi_delay_ms(100);
    instance->state = LegicPrimePollerStateSetup;
    return cmd;
}

static LegicPrimePollerStateHandler
    legic_prime_poller_state_handlers[LegicPrimePollerStateNum] = {
        [LegicPrimePollerStateSetup] = legic_prime_poller_setup_handler,
        [LegicPrimePollerStateRequestMode] = legic_prime_poller_request_mode_handler,
        [LegicPrimePollerStateRead] = legic_prime_poller_read_handler,
        [LegicPrimePollerStateResume] = legic_prime_poller_resume_handler,
        [LegicPrimePollerStateSuccess] = legic_prime_poller_success_handler,
        [LegicPrimePollerStateFailure] = legic_prime_poller_failure_handler,
};

static NfcCommand legic_prime_poller_run(NfcGenericEvent event, void* context) {
    furi_assert(context);
    furi_assert(event.protocol == NfcProtocolInvalid);
    furi_assert(event.event_data);

    LegicPrimePoller* instance = context;
    NfcEvent* nfc_event = event.event_data;
    NfcCommand command = NfcCommandContinue;

    furi_assert(instance->state < LegicPrimePollerStateNum);
    if(nfc_event->type == NfcEventTypePollerReady) {
        command = legic_prime_poller_state_handlers[instance->state](instance);
    }
    return command;
}

static bool legic_prime_poller_detect(NfcGenericEvent event, void* context) {
    furi_assert(context);
    furi_assert(event.event_data);
    furi_assert(event.instance);
    furi_assert(event.protocol == NfcProtocolInvalid);

    LegicPrimePoller* instance = context;
    NfcEvent* nfc_event = event.event_data;
    furi_assert(instance->state == LegicPrimePollerStateSetup);

    bool detected = false;
    if(nfc_event->type == NfcEventTypePollerReady) {
        LegicPrimeError err = legic_prime_poller_setup(instance, 0x55);
        detected = (err == LegicPrimeErrorNone);
    }
    return detected;
}

const NfcPollerBase nfc_poller_legic_prime = {
    .alloc = (NfcPollerAlloc)legic_prime_poller_alloc,
    .free = (NfcPollerFree)legic_prime_poller_free,
    .set_callback = (NfcPollerSetCallback)legic_prime_poller_set_callback,
    .run = (NfcPollerRun)legic_prime_poller_run,
    .detect = (NfcPollerDetect)legic_prime_poller_detect,
    .get_data = (NfcPollerGetData)legic_prime_poller_get_data,
};
