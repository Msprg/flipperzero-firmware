/**
 * @file legic_prime.h
 * @brief LEGIC Prime protocol data structures.
 *
 * LEGIC Prime is a 13.56 MHz proprietary protocol from LEGIC Identsystems AG.
 * Tags come in three memory sizes (MIM22 / MIM256 / MIM1024). The first 22
 * bytes of every tag are a fixed header containing the manufacturer block:
 * MCD (1B) + MSN (3B) + MCC (1B CRC-8) + DCF (2B) + CDF (15B).
 *
 * Reader-only support: emulation (listener) is not implemented in this build.
 */
#pragma once

#include <nfc/protocols/nfc_device_base_i.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LEGIC_PRIME_UID_SIZE         (4U) // MCD + MSN
#define LEGIC_PRIME_MEM_SIZE_MIM22   (22U)
#define LEGIC_PRIME_MEM_SIZE_MIM256  (256U)
#define LEGIC_PRIME_MEM_SIZE_MIM1024 (1024U)
#define LEGIC_PRIME_MEM_MAX          LEGIC_PRIME_MEM_SIZE_MIM1024

typedef enum {
    LegicPrimeErrorNone,
    LegicPrimeErrorNotPresent,
    LegicPrimeErrorWrongCrc,
    LegicPrimeErrorTimeout,
    LegicPrimeErrorCommunication,
    LegicPrimeErrorUnknownType,
} LegicPrimeError;

typedef enum {
    LegicPrimeTypeMim22,
    LegicPrimeTypeMim256,
    LegicPrimeTypeMim1024,

    LegicPrimeTypeNum,
} LegicPrimeType;

typedef struct {
    LegicPrimeType type;
    uint8_t iv; // IV used on the last successful read (kept for debugging)
    uint8_t data[LEGIC_PRIME_MEM_MAX];
    uint16_t data_size; // populated bytes count (22 / 256 / 1024)
} LegicPrimeData;

extern const NfcDeviceBase nfc_device_legic_prime;

LegicPrimeData* legic_prime_alloc(void);

void legic_prime_free(LegicPrimeData* data);

void legic_prime_reset(LegicPrimeData* data);

void legic_prime_copy(LegicPrimeData* data, const LegicPrimeData* other);

bool legic_prime_verify(LegicPrimeData* data, const FuriString* device_type);

bool legic_prime_load(LegicPrimeData* data, FlipperFormat* ff, uint32_t version);

bool legic_prime_save(const LegicPrimeData* data, FlipperFormat* ff);

bool legic_prime_is_equal(const LegicPrimeData* data, const LegicPrimeData* other);

const char* legic_prime_get_device_name(const LegicPrimeData* data, NfcDeviceNameType name_type);

const uint8_t* legic_prime_get_uid(const LegicPrimeData* data, size_t* uid_len);

bool legic_prime_set_uid(LegicPrimeData* data, const uint8_t* uid, size_t uid_len);

LegicPrimeData* legic_prime_get_base_data(const LegicPrimeData* data);

uint16_t legic_prime_get_mem_size(LegicPrimeType type);

/** Returns true if the MCC byte equals CRC8 over MCD+MSN. */
bool legic_prime_verify_mcc(const LegicPrimeData* data);

#ifdef __cplusplus
}
#endif
