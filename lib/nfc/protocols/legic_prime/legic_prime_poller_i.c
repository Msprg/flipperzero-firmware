#include "legic_prime_poller_i.h"
#include "legic_prime_prng.h"
#include "legic_prime_crc.h"

#include <furi.h>
#include <furi_hal_cortex.h>
#include <furi_hal_nfc.h>

#define TAG "LegicPrimePoller"

static LegicPrimeError legic_prime_poller_map_nfc_error(NfcError err) {
    switch(err) {
    case NfcErrorNone:
        return LegicPrimeErrorNone;
    case NfcErrorTimeout:
        return LegicPrimeErrorTimeout;
    case NfcErrorIncompleteFrame:
        return LegicPrimeErrorCommunication;
    default:
        return LegicPrimeErrorNotPresent;
    }
}

/*
 * Frame schedule.
 *
 * A LEGIC tag advances its keystream every 99.1 µs while idle (Proxmark
 * legicrfsim.c rx_frame).  If our next frame's first pause comes at t:
 *  - after a tag reply ending at E, the tag has advanced ceil((t - E) / 99.1 µs);
 *  - after our frame without a reply, ending at A, ceil((t - A) / 99.1 µs) - 1.
 * Proxmark's reader waits 220 µs and forwards 3 and 2 steps respectively.  We
 * aim 250 µs + k * 99.1 µs past the reference (±50 µs of margin), with k as
 * small as the software latency allows, and forward k more steps.
 */
#define LEGIC_PRIME_TAG_FRAME_WAIT_CHIPS (70U) // 330 µs, in 64/fc ticks
#define LEGIC_PRIME_TAG_BIT_CHIPS        (21U) // 99.1 µs
#define LEGIC_PRIME_NEXT_FRAME_US        (250U)
#define LEGIC_PRIME_SCHEDULE_MARGIN_US   (1500U)

// Read attempts per byte; retries alternate the receiver gain.
#define LEGIC_PRIME_READ_ATTEMPTS (4)

static uint32_t legic_prime_chips_to_cycles(uint32_t chips) {
    const uint64_t cpu_hz = (uint64_t)furi_hal_cortex_instructions_per_microsecond() * 1000000U;
    return (uint32_t)(chips * 64U * cpu_hz / 13560000U);
}

static void legic_prime_poller_schedule(LegicPrimePoller* instance) {
    const uint32_t cpu_mhz = furi_hal_cortex_instructions_per_microsecond();
    const uint32_t bit = legic_prime_chips_to_cycles(LEGIC_PRIME_TAG_BIT_CHIPS);

    uint32_t ref = furi_hal_nfc_legic_poller_get_frame_end();
    int steps = 2;
    if(instance->last_reply_bits) {
        ref += legic_prime_chips_to_cycles(
            LEGIC_PRIME_TAG_FRAME_WAIT_CHIPS +
            instance->last_reply_bits * LEGIC_PRIME_TAG_BIT_CHIPS);
        steps = 3;
    }

    uint32_t t = ref + LEGIC_PRIME_NEXT_FRAME_US * cpu_mhz;
    const uint32_t earliest = DWT->CYCCNT + LEGIC_PRIME_SCHEDULE_MARGIN_US * cpu_mhz;
    while((int32_t)(t - earliest) < 0) {
        t += bit;
        steps++;
    }

    legic_prime_prng_forward(steps);
    furi_hal_nfc_legic_poller_set_tx_start(t);
}

/*
 * Send `bits` of `value` (LSB-first on the wire, matching Proxmark's
 * tx_frame loop) XORed with the running keystream, advancing the PRNG by
 * `bits` steps, and receive a reply of `rx_bits` bits (0 = none expected).
 */
static LegicPrimeError legic_prime_poller_tx_frame(
    LegicPrimePoller* instance,
    uint32_t value,
    uint8_t bits,
    uint8_t rx_bits) {
    furi_assert(instance);
    furi_assert(bits <= 32);

    bit_buffer_reset(instance->tx_buffer);

    for(uint8_t i = 0; i < bits; i++) {
        bool data_bit = (value >> i) & 1u;
        bool key_bit = legic_prime_prng_get_bit();
        bit_buffer_append_bit(instance->tx_buffer, data_bit ^ key_bit);
        legic_prime_prng_forward(1);
    }

    furi_hal_nfc_legic_poller_set_rx_bits(rx_bits);
    instance->last_reply_bits = rx_bits;

    NfcError nerr =
        nfc_poller_trx(instance->nfc, instance->tx_buffer, instance->rx_buffer, LEGIC_PRIME_FWT_FC);
    return legic_prime_poller_map_nfc_error(nerr);
}

/* Decode `bits` of received data into `*out`, XOR-decoding against the
 * keystream and advancing the PRNG by `bits` steps. */
static void legic_prime_poller_decode_rx(
    const BitBuffer* rx_buffer,
    uint8_t bits,
    uint32_t* out) {
    uint32_t v = 0;
    for(uint8_t i = 0; i < bits; i++) {
        bool rx_bit = false;
        // BitBuffer stores bits LSB-first in each byte.
        size_t byte_idx = i / 8;
        size_t bit_idx = i % 8;
        if(byte_idx < bit_buffer_get_size_bytes(rx_buffer) ||
           (byte_idx == bit_buffer_get_size_bytes(rx_buffer) &&
            bit_buffer_has_partial_byte(rx_buffer))) {
            uint8_t byte = bit_buffer_get_byte(rx_buffer, byte_idx);
            rx_bit = (byte >> bit_idx) & 1u;
        }
        bool key_bit = legic_prime_prng_get_bit();
        v |= ((uint32_t)(rx_bit ^ key_bit)) << i;
        legic_prime_prng_forward(1);
    }
    *out = v;
}

// Mirrors Proxmark armsrc/legicrf.c setup_phase.
LegicPrimeError legic_prime_poller_setup(LegicPrimePoller* instance, uint8_t iv) {
    furi_assert(instance);

    // Explicit warmup: hold carrier briefly to fully power the tag before
    // sending the IV. Proxmark armsrc/legicrf.c uses ~5 ms.
    furi_delay_ms(5);


    // Phase 1: send IV (7 bits) with all-zero keystream, as soon as possible.
    legic_prime_prng_init(0);
    furi_hal_nfc_legic_poller_set_tx_start(0);
    LegicPrimeError ret = legic_prime_poller_tx_frame(instance, iv, 7, 6);
    if(ret != LegicPrimeErrorNone) return ret;

    // Phase 2: the tag seeds its PRNG with the IV and steps it twice before
    // the 6-bit card type reply.
    legic_prime_prng_init(iv);
    legic_prime_prng_forward(2);
    uint32_t type_word = 0;
    legic_prime_poller_decode_rx(instance->rx_buffer, 6, &type_word);
    uint8_t type_byte = (uint8_t)(type_word & 0x3F);

    uint8_t ack = 0;
    switch(type_byte) {
    case 0x0D: // MIM22
        instance->data->type = LegicPrimeTypeMim22;
        instance->data->data_size = LEGIC_PRIME_MEM_SIZE_MIM22;
        instance->cmd_size_bits = 6;
        instance->addr_size_bits = 5;
        ack = 0x19;
        break;
    case 0x1D: // MIM256
        instance->data->type = LegicPrimeTypeMim256;
        instance->data->data_size = LEGIC_PRIME_MEM_SIZE_MIM256;
        instance->cmd_size_bits = 9;
        instance->addr_size_bits = 8;
        ack = 0x39;
        break;
    case 0x3D: // MIM1024
        instance->data->type = LegicPrimeTypeMim1024;
        instance->data->data_size = LEGIC_PRIME_MEM_SIZE_MIM1024;
        instance->cmd_size_bits = 11;
        instance->addr_size_bits = 10;
        ack = 0x39;
        break;
    default:
        FURI_LOG_E(TAG, "Unknown LEGIC type byte: 0x%02X", type_byte);
        return LegicPrimeErrorUnknownType;
    }

    instance->data->iv = iv;

    // Phase 3: ACK in the scheduled slot.  No reply is expected; the FWT
    // timeout that follows is benign.
    legic_prime_poller_schedule(instance);
    ret = legic_prime_poller_tx_frame(instance, ack, 6, 0);
    if(ret != LegicPrimeErrorNone && ret != LegicPrimeErrorTimeout) return ret;

    return LegicPrimeErrorNone;
}

// Mirrors Proxmark armsrc/legicrf.c read_byte.
LegicPrimeError
    legic_prime_poller_read_byte(LegicPrimePoller* instance, uint16_t addr, uint8_t* value) {
    furi_assert(instance);
    furi_assert(value);

    legic_prime_poller_schedule(instance);

    // Command: (addr << 1) | READ(=1)
    uint32_t cmd = (((uint32_t)addr) << 1) | 1u;
    LegicPrimeError ret = legic_prime_poller_tx_frame(instance, cmd, instance->cmd_size_bits, 12);
    if(ret != LegicPrimeErrorNone) return ret;

    // Response is 12 bits = 8-bit data | 4-bit CRC, XOR-encrypted, sent after
    // the tag steps its PRNG twice.
    legic_prime_prng_forward(2);
    uint32_t resp = 0;
    legic_prime_poller_decode_rx(instance->rx_buffer, 12, &resp);

    uint8_t data_byte = (uint8_t)(resp & 0xFF);
    uint8_t rx_crc = (uint8_t)((resp >> 8) & 0x0F);

    uint8_t calc_crc = legic_prime_crc4(cmd, instance->cmd_size_bits, data_byte);
    if(rx_crc != calc_crc) {
        FURI_LOG_D(
            TAG,
            "CRC mismatch at addr %u: got 0x%X expected 0x%X (data 0x%02X)",
            addr,
            rx_crc,
            calc_crc,
            data_byte);
        return LegicPrimeErrorWrongCrc;
    }

    *value = data_byte;
    return LegicPrimeErrorNone;
}

LegicPrimeError legic_prime_poller_read(LegicPrimePoller* instance) {
    furi_assert(instance);
    furi_assert(instance->data->data_size <= LEGIC_PRIME_MEM_MAX);

    // A CRC error means our decode was wrong; the keystream schedule is
    // unaffected, so just read the byte again (alternating receiver gain).
    for(uint16_t addr = instance->read_addr; addr < instance->data->data_size; addr++) {
        LegicPrimeError ret = LegicPrimeErrorWrongCrc;
        for(int attempt = 0; attempt < LEGIC_PRIME_READ_ATTEMPTS; attempt++) {
            furi_hal_nfc_legic_poller_set_rx_reduced_gain(attempt % 2);
            ret = legic_prime_poller_read_byte(instance, addr, &instance->data->data[addr]);
            if(ret != LegicPrimeErrorWrongCrc) break;
        }
        furi_hal_nfc_legic_poller_set_rx_reduced_gain(false);
        if(ret != LegicPrimeErrorNone) {
            FURI_LOG_E(TAG, "Read failed at addr %u: %d", addr, ret);
            return ret;
        }
        instance->read_addr = addr + 1;
    }

    return LegicPrimeErrorNone;
}
