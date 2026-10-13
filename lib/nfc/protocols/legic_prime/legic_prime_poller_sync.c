#include "legic_prime_poller_sync.h"
#include "legic_prime_poller_i.h"

#include <nfc/nfc_poller.h>
#include <furi.h>

#define LEGIC_PRIME_POLLER_FLAG_COMMAND_COMPLETE (1UL << 0)

typedef struct {
    FuriThreadId thread_id;
    LegicPrimeError error;
    LegicPrimeData* data;
} LegicPrimePollerSyncContext;

static NfcCommand legic_prime_poller_sync_callback(NfcGenericEvent event, void* context) {
    furi_assert(context);
    furi_assert(event.event_data);
    furi_assert(event.instance);
    furi_assert(event.protocol == NfcProtocolLegicPrime);

    LegicPrimePollerSyncContext* ctx = context;
    LegicPrimePollerEvent* legic_event = event.event_data;

    NfcCommand command = NfcCommandContinue;
    if(legic_event->type == LegicPrimePollerEventTypeRequestMode) {
        legic_event->data->mode_request.mode = LegicPrimePollerModeRead;
    } else if(legic_event->type == LegicPrimePollerEventTypeSuccess) {
        memcpy(ctx->data, legic_prime_poller_get_data(event.instance), sizeof(LegicPrimeData));
        ctx->error = LegicPrimeErrorNone;
        furi_thread_flags_set(ctx->thread_id, LEGIC_PRIME_POLLER_FLAG_COMMAND_COMPLETE);
        command = NfcCommandStop;
    } else if(legic_event->type == LegicPrimePollerEventTypeFailure) {
        ctx->error = legic_event->data->error;
        furi_thread_flags_set(ctx->thread_id, LEGIC_PRIME_POLLER_FLAG_COMMAND_COMPLETE);
        command = NfcCommandStop;
    }
    return command;
}

LegicPrimeError legic_prime_poller_sync_read(Nfc* nfc, LegicPrimeData* data) {
    furi_check(nfc);
    furi_check(data);

    LegicPrimePollerSyncContext ctx = {
        .thread_id = furi_thread_get_current_id(),
        .error = LegicPrimeErrorNotPresent,
        .data = data,
    };

    NfcPoller* poller = nfc_poller_alloc(nfc, NfcProtocolLegicPrime);
    nfc_poller_start(poller, legic_prime_poller_sync_callback, &ctx);
    furi_thread_flags_wait(
        LEGIC_PRIME_POLLER_FLAG_COMMAND_COMPLETE, FuriFlagWaitAny, FuriWaitForever);
    furi_thread_flags_clear(LEGIC_PRIME_POLLER_FLAG_COMMAND_COMPLETE);

    nfc_poller_stop(poller);
    nfc_poller_free(poller);

    return ctx.error;
}
