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
// Failed reads allowed per byte and session (CRC errors or disagreeing
// reads).  Each resumed session starts afresh, so a byte that ran out of
// attempts is not lost for good; the resume limit bounds the total.
#define LEGIC_PRIME_READ_ATTEMPTS (6U)

static uint8_t legic_prime_type_code(LegicPrimeType type) {
    return type == LegicPrimeTypeMim22 ? 0x0D : type == LegicPrimeTypeMim1024 ? 0x3D : 0x1D;
}

static uint8_t legic_prime_ack_code(LegicPrimeType type) {
    // Card type | READCMD, as Proxmark armsrc/legicrf.c setup_phase.
    return type == LegicPrimeTypeMim22 ? 0x19 : 0x39;
}

static void legic_prime_poller_set_type(LegicPrimePoller* instance, LegicPrimeType type) {
    instance->data->type = type;
    switch(type) {
    case LegicPrimeTypeMim22:
        instance->data->data_size = LEGIC_PRIME_MEM_SIZE_MIM22;
        instance->cmd_size_bits = 6;
        instance->addr_size_bits = 5;
        break;
    case LegicPrimeTypeMim1024:
        instance->data->data_size = LEGIC_PRIME_MEM_SIZE_MIM1024;
        instance->cmd_size_bits = 11;
        instance->addr_size_bits = 10;
        break;
    default:
        instance->data->data_size = LEGIC_PRIME_MEM_SIZE_MIM256;
        instance->cmd_size_bits = 9;
        instance->addr_size_bits = 8;
        break;
    }
}

static bool legic_prime_type_from_code(uint8_t code, LegicPrimeType* type) {
    for(LegicPrimeType t = 0; t < LegicPrimeTypeNum; t++) {
        if(legic_prime_type_code(t) == code) {
            *type = t;
            return true;
        }
    }
    return false;
}

/*
 * Build one reader frame on the running keystream: forward `pre_steps`
 * (the tag's idle steps since the previous frame or reply: 3 after a reply,
 * 2 after the ACK, for the ~220 µs gaps the HAL keeps), XOR the frame bits,
 * then, if a reply is expected, skip the tag's 2 steps and keep the reply's
 * keystream so the reply can be decoded after later frames were built.
 */
static void legic_prime_poller_build(
    LegicPrimeExchange* ex,
    uint32_t value,
    uint8_t bits,
    uint8_t pre_steps,
    uint8_t rx_bits) {
    legic_prime_prng_forward(pre_steps);
    ex->tx = 0;
    for(uint8_t i = 0; i < bits; i++) {
        ex->tx |= (uint32_t)(((value >> i) & 1u) ^ legic_prime_prng_get_bit()) << i;
        legic_prime_prng_forward(1);
    }
    ex->tx_bits = bits;
    ex->rx_bits = rx_bits;
    ex->ks = 0;
    if(rx_bits) {
        legic_prime_prng_forward(2);
        for(uint8_t i = 0; i < rx_bits; i++) {
            ex->ks |= (uint32_t)legic_prime_prng_get_bit() << i;
            legic_prime_prng_forward(1);
        }
    }
}

static void legic_prime_poller_frame_bytes(const LegicPrimeExchange* ex, uint8_t bytes[4]) {
    for(size_t i = 0; i < 4; i++) {
        bytes[i] = (ex->tx >> (8 * i)) & 0xFF;
    }
}

static void legic_prime_poller_queue(
    const LegicPrimeExchange* first,
    const LegicPrimeExchange* second) {
    uint8_t a[4];
    uint8_t b[4];
    legic_prime_poller_frame_bytes(first, a);
    if(second) legic_prime_poller_frame_bytes(second, b);
    furi_hal_nfc_legic_poller_set_next(
        a,
        first->tx_bits,
        second ? b : NULL,
        second ? second->tx_bits : 0,
        second ? second->rx_bits : first->rx_bits);
}

// Send ex (or claim it if the HAL already sent it) and decode its reply.
static LegicPrimeError
    legic_prime_poller_exchange(LegicPrimePoller* instance, const LegicPrimeExchange* ex, uint32_t* reply) {
    bit_buffer_reset(instance->tx_buffer);
    for(uint8_t i = 0; i < ex->tx_bits; i++) {
        bit_buffer_append_bit(instance->tx_buffer, (ex->tx >> i) & 1u);
    }
    furi_hal_nfc_legic_poller_set_rx_bits(ex->rx_bits);

    NfcError nerr =
        nfc_poller_trx(instance->nfc, instance->tx_buffer, instance->rx_buffer, LEGIC_PRIME_FWT_FC);
    if(nerr != NfcErrorNone) return legic_prime_poller_map_nfc_error(nerr);

    uint32_t v = 0;
    const size_t rx_bits = bit_buffer_get_size(instance->rx_buffer);
    for(uint8_t i = 0; i < ex->rx_bits; i++) {
        bool bit = i < rx_bits && ((bit_buffer_get_byte(instance->rx_buffer, i / 8) >> (i % 8)) & 1u);
        v |= (uint32_t)bit << i;
    }
    *reply = v ^ ex->ks;
    return LegicPrimeErrorNone;
}

// The IV frame (plain) and the keystream of the tag's 6-bit type reply.
static void legic_prime_poller_build_iv(LegicPrimeExchange* ex, uint8_t iv) {
    ex->tx = iv;
    ex->tx_bits = 7;
    ex->rx_bits = 6;
    ex->ks = 0;
    legic_prime_prng_init(iv);
    legic_prime_prng_forward(2);
    for(uint8_t i = 0; i < 6; i++) {
        ex->ks |= (uint32_t)legic_prime_prng_get_bit() << i;
        legic_prime_prng_forward(1);
    }
}

// Detection: send the IV and decode the card type.  The tag is then waiting
// for an ACK, so reset the field before reading.
LegicPrimeError legic_prime_poller_setup(LegicPrimePoller* instance, uint8_t iv) {
    furi_assert(instance);

    // Hold the carrier briefly to power the tag (Proxmark uses ~5 ms).
    furi_delay_ms(5);

    LegicPrimeExchange iv_ex;
    legic_prime_poller_build_iv(&iv_ex, iv);
    uint32_t reply = 0;
    LegicPrimeError ret = legic_prime_poller_exchange(instance, &iv_ex, &reply);
    if(ret != LegicPrimeErrorNone) return ret;

    LegicPrimeType type;
    if(!legic_prime_type_from_code(reply & 0x3F, &type)) {
        FURI_LOG_E(TAG, "Unknown LEGIC type byte: 0x%02lX", reply & 0x3F);
        return LegicPrimeErrorUnknownType;
    }
    legic_prime_poller_set_type(instance, type);
    instance->data->iv = iv;
    return LegicPrimeErrorNone;
}

static bool legic_prime_poller_is_done(const LegicPrimePoller* instance, uint16_t addr) {
    return instance->done[addr / 8] & (1u << (addr % 8));
}

// Next address to read after `after`, cycling, `after` itself last.
static bool legic_prime_poller_next_addr(LegicPrimePoller* instance, uint16_t after, uint16_t* addr) {
    const uint16_t size = instance->data->data_size;
    for(uint16_t i = 1; i <= size; i++) {
        const uint16_t a = (after + i) % size;
        if(!legic_prime_poller_is_done(instance, a) &&
           instance->attempts[a] < LEGIC_PRIME_READ_ATTEMPTS) {
            *addr = a;
            return true;
        }
    }
    return false;
}

/*
 * One read session, mirroring Proxmark armsrc/legicrf.c but pipelined: the
 * ACK and the first READ go out chained behind the type reply, and every
 * further READ is queued before the previous reply is decoded, so each frame
 * follows the tag's reply by ~220 µs as real tags require.  Returns
 * LegicPrimeErrorNone once every byte was read.
 */
LegicPrimeError legic_prime_poller_read(LegicPrimePoller* instance) {
    furi_assert(instance);
    furi_assert(instance->data->data_size <= LEGIC_PRIME_MEM_MAX);

    memset(instance->attempts, 0, sizeof(instance->attempts));
    uint16_t addr;
    if(!legic_prime_poller_next_addr(instance, instance->data->data_size - 1, &addr)) {
        return instance->done_count == instance->data->data_size ? LegicPrimeErrorNone :
                                                                    LegicPrimeErrorWrongCrc;
    }

    furi_delay_ms(5);

    const LegicPrimeType type = instance->data->type;
    LegicPrimeExchange iv_ex, ack, cur, next;
    legic_prime_prng_init(0);
    legic_prime_poller_build_iv(&iv_ex, instance->data->iv);
    legic_prime_poller_build(&ack, legic_prime_ack_code(type), 6, 3, 0);
    legic_prime_poller_build(&cur, (addr << 1) | 1u, instance->cmd_size_bits, 2, 12);
    cur.addr = addr;
    legic_prime_poller_queue(&ack, &cur);

    uint32_t reply = 0;
    LegicPrimeError ret = legic_prime_poller_exchange(instance, &iv_ex, &reply);
    if(ret != LegicPrimeErrorNone) return ret;
    if((reply & 0x3F) != legic_prime_type_code(type)) {
        // ACK and READ were built for the wrong type; retry with the new one.
        LegicPrimeType seen;
        if(legic_prime_type_from_code(reply & 0x3F, &seen)) {
            FURI_LOG_D(TAG, "Card type 0x%02lX, expected 0x%02X", reply & 0x3F, legic_prime_type_code(type));
        }
        return LegicPrimeErrorUnknownType;
    }

    for(uint32_t budget = instance->data->data_size * LEGIC_PRIME_READ_ATTEMPTS + 16; budget;
        budget--) {
        if(!furi_hal_nfc_legic_poller_next_sent()) return LegicPrimeErrorTimeout;

        uint16_t next_addr;
        const bool have_next = legic_prime_poller_next_addr(instance, cur.addr, &next_addr);
        if(have_next) {
            legic_prime_poller_build(
                &next, (next_addr << 1) | 1u, instance->cmd_size_bits, 3, 12);
            next.addr = next_addr;
            legic_prime_poller_queue(&next, NULL);
        }

        ret = legic_prime_poller_exchange(instance, &cur, &reply);
        if(ret != LegicPrimeErrorNone) return ret;

        // A 4-bit CRC passes 1 in 16 misdecoded replies, so a byte is only
        // accepted once two reads (with different keystreams) agree.
        const uint8_t data = reply & 0xFF;
        const uint8_t crc = (reply >> 8) & 0x0F;
        const uint8_t bit = 1u << (cur.addr % 8);
        if(legic_prime_poller_is_done(instance, cur.addr)) {
            // Redundant read of an already confirmed byte.
        } else if(crc == legic_prime_crc4((cur.addr << 1) | 1u, instance->cmd_size_bits, data)) {
            if((instance->seen[cur.addr / 8] & bit) && instance->data->data[cur.addr] == data) {
                instance->done[cur.addr / 8] |= bit;
                instance->done_count++;
            } else {
                if(instance->seen[cur.addr / 8] & bit) instance->attempts[cur.addr]++;
                instance->data->data[cur.addr] = data;
                instance->seen[cur.addr / 8] |= bit;
            }
        } else {
            instance->attempts[cur.addr]++;
            FURI_LOG_D(TAG, "CRC mismatch at addr %u", cur.addr);
        }

        if(instance->done_count == instance->data->data_size) return LegicPrimeErrorNone;
        if(!have_next) break;
        cur = next;
    }

    return LegicPrimeErrorWrongCrc;
}
