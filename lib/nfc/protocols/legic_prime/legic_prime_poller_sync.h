#pragma once

#include "legic_prime.h"
#include <nfc/nfc.h>

#ifdef __cplusplus
extern "C" {
#endif

LegicPrimeError legic_prime_poller_sync_read(Nfc* nfc, LegicPrimeData* data);

#ifdef __cplusplus
}
#endif
