/*
 * @file furi_hal_nfc_legic.c
 * @brief LEGIC Prime technology layer for the ST25R3916.
 *
 * LEGIC Prime: 13.56 MHz carrier.
 *  - Reader→tag: pause-position 100% OOK.  Every bit is a ~19 µs pause
 *    followed by ~42 µs ('0') or ~80 µs ('1') of carrier, and the frame is
 *    closed by one extra pause: the tag measures pause-to-pause intervals,
 *    so without the closing pause it never sees the last bit.  Emitted
 *    through the FIFO in om_subcarrier_stream at stx_212 (4.72 µs/chip-bit).
 *  - Tag→reader: plain ASK on a 212 kHz load-modulation subcarrier, 99.1 µs
 *    per bit, no SOF/EOF/Manchester, starting 330 µs after the closing pause
 *    (Proxmark armsrc/legicrfsim.c).
 *  - The tag advances its keystream every 99.1 µs while idle, so frames are
 *    scheduled at exact times chosen by the poller (see set_tx_start).
 *
 * RX uses the chip's subcarrier-stream receiver (scf_sc212, scp_4): each
 * FIFO bit reports subcarrier presence over ~9.44 µs.  A reception ends
 * ~20 µs into the first '0' bit and the receiver does not re-arm by itself,
 * so one reply arrives as one reception per run of '1' bits.  The receiver
 * is re-armed after each one, and the runs are placed into 99.1 µs bit
 * windows by their start time.  AGC and dynamic squelch are off: with them
 * on, later runs in a reply fade out.
 *
 * References:
 *  - Proxmark3 armsrc/legicrf.c (reader), armsrc/legicrfsim.c (tag side)
 *  - Tracking issue: github.com/flipperdevices/flipperzero-firmware/issues/1688
 */
#include "furi_hal_nfc_i.h"
#include "furi_hal_nfc_tech_i.h"

#include <furi.h>
#include <furi_hal_cortex.h>
#include <storage/storage.h>

#define TAG "FuriHalLegic"

#define LEGIC_TX_DBG_LOG_PATH "/ext/legic_tx_dbg.log"
#define LEGIC_RX_DBG_LOG_PATH "/ext/legic_rx_dbg.log"

#define FURI_HAL_NFC_LEGIC_POLLER_MAX_BUFFER_SIZE (16U)
#define FURI_HAL_NFC_LEGIC_FIFO_SIZE              (512U)

#define FURI_HAL_NFC_LEGIC_POLLER_FWT_COMP_FC (-1300)

#define BITS_IN_BYTE (8U)

// All LEGIC timing is in units of one 212 kHz subcarrier period (64/fc,
// 4.72 µs), the same tick Proxmark uses.
#define LEGIC_TAG_FRAME_WAIT_CHIPS (70U) // 330 µs from closing pause to reply
#define LEGIC_TAG_BIT_CHIPS        (21U) // 99.1 µs per reply bit
#define LEGIC_FIFO_BIT_CHIPS       (2U) // scp_4 report period, measured
// Detection lag between subcarrier onset and the reported reception start:
// ~25 µs for the first reception after TX and for a receiver re-armed long
// before the run, ~40 µs when re-armed within ~100 µs of the onset.
#define LEGIC_RX_LAG_CHIPS        (5U)
#define LEGIC_RX_REARM_LAG_CHIPS  (8U)
#define LEGIC_RX_REARM_SHORT_CHIPS (21U)
// TRANSMIT command to first chip-bit on air, measured via TxE (TxE comes
// ~(45 + chips * 4.72 + 5) µs after the command).  The frame end is derived
// from the command time; TxE is only logged as a cross-check because the
// event thread can see it late.
#define LEGIC_TX_CMD_LATENCY_US (45U)
// Longest busy-wait for a scheduled frame.
#define LEGIC_TX_MAX_WAIT_US (20000U)

#define LEGIC_RX_GAP_MS       (2U)
#define LEGIC_RX_MAX_SEGMENTS (16U)
#define LEGIC_RX_WINDOW_CHIPS (LEGIC_TAG_FRAME_WAIT_CHIPS + 23U * LEGIC_TAG_BIT_CHIPS + 40U)

// First-stage AM gain reduction (RX_CONF3 rg1_am = 4) used on read retries;
// see furi_hal_nfc_legic_common_init for the RX tuning history.
#define LEGIC_RX_CONF3_RETRY (0x80U)

typedef struct {
    uint8_t frame_buf[FURI_HAL_NFC_LEGIC_POLLER_MAX_BUFFER_SIZE * 4];
    size_t frame_buf_bits;
    bool rx_reduced_gain;
    uint8_t fifo_buf[FURI_HAL_NFC_LEGIC_FIFO_SIZE];
    size_t fifo_buf_bits;
    // Frame schedule (DWT cycles).
    uint32_t tx_start_at;
    uint32_t t_tx_cmd;
    uint32_t t_frame_end;
    uint32_t t_txe;
    int32_t tx_wait_us;
    size_t tx_pad_chips;
    size_t rx_bits_expected;
    // RX segments of the current exchange: one per reception.
    bool rx_started;
    bool seg_open;
    uint32_t t_last_rx_event;
    size_t seg_count;
    uint32_t seg_t_rxs[LEGIC_RX_MAX_SEGMENTS];
    uint32_t seg_t_rxe[LEGIC_RX_MAX_SEGMENTS];
    uint16_t seg_bits[LEGIC_RX_MAX_SEGMENTS];
    uint16_t seg_off[LEGIC_RX_MAX_SEGMENTS];
    // Debug instrumentation: per-TX/RX records buffered in RAM and written to
    // SD only between exchanges (an SD write blocks the thread for up to
    // several ms, which wrecks event timestamps if it happens mid-exchange).
    char dbg_tx_buf[8192];
    size_t dbg_tx_len;
    char dbg_rx_buf[12288];
    size_t dbg_rx_len;
    Storage* dbg_storage;
    File* dbg_tx_file;
    File* dbg_rx_file;
    uint32_t dbg_tx_seq;
    uint32_t dbg_rx_seq;
} FuriHalNfcLegicPoller;

static FuriHalNfcLegicPoller* furi_hal_nfc_legic_poller = NULL;

static uint32_t legic_chips_to_cycles(uint32_t chips) {
    const uint64_t cpu_hz = (uint64_t)furi_hal_cortex_instructions_per_microsecond() * 1000000U;
    return (uint32_t)(chips * 64U * cpu_hz / 13560000U);
}

static int32_t legic_cycles_to_us(int32_t cycles) {
    return cycles / (int32_t)furi_hal_cortex_instructions_per_microsecond();
}

static void legic_dbg_flush(FuriHalNfcLegicPoller* instance) {
    if(instance->dbg_tx_len && instance->dbg_tx_file && storage_file_is_open(instance->dbg_tx_file)) {
        storage_file_write(instance->dbg_tx_file, instance->dbg_tx_buf, instance->dbg_tx_len);
    }
    if(instance->dbg_rx_len && instance->dbg_rx_file && storage_file_is_open(instance->dbg_rx_file)) {
        storage_file_write(instance->dbg_rx_file, instance->dbg_rx_buf, instance->dbg_rx_len);
    }
    instance->dbg_tx_len = 0;
    instance->dbg_rx_len = 0;
}

static FuriHalNfcError furi_hal_nfc_legic_common_init(const FuriHalSpiBusHandle* handle) {
    // AM channel, 12 kHz / 80 kHz zeros, 300 kHz LPF, static full gain.
    // Tuned 2026-10-13 against the PM3 `hf legic sim` card-type reply
    // (a 2-bit and a 3-bit run of subcarrier): with the ISO15693 600 kHz LPF
    // the receiver dropped the subcarrier ~150-200 µs into runs of 3+ '1'
    // bits (<=25% of replies correct); 300 kHz kept them intact (94-100%
    // across gain settings).  AGC (any mode) and dynamic squelch made runs
    // fade; the PM channel and CORR_CONF1=0x00 cut runs after 8 FIFO bits.
    st25r3916_write_reg(
        handle,
        ST25R3916_REG_RX_CONF1,
        ST25R3916_REG_RX_CONF1_z12k | ST25R3916_REG_RX_CONF1_h80 |
            ST25R3916_REG_RX_CONF1_lp_300khz);
    st25r3916_write_reg(handle, ST25R3916_REG_RX_CONF2, 0x00);
    st25r3916_write_reg(handle, ST25R3916_REG_RX_CONF3, 0x00);
    st25r3916_write_reg(handle, ST25R3916_REG_RX_CONF4, 0x00);

    st25r3916_write_reg(
        handle,
        ST25R3916_REG_CORR_CONF1,
        ST25R3916_REG_CORR_CONF1_corr_s0 | ST25R3916_REG_CORR_CONF1_corr_s1 |
            ST25R3916_REG_CORR_CONF1_corr_s4);

    st25r3916_write_reg(handle, ST25R3916_REG_CORR_CONF2, ST25R3916_REG_CORR_CONF2_corr_s8);

    return FuriHalNfcErrorNone;
}

static FuriHalNfcError furi_hal_nfc_legic_poller_init(const FuriHalSpiBusHandle* handle) {
    furi_check(furi_hal_nfc_legic_poller == NULL);

    furi_hal_nfc_legic_poller = malloc(sizeof(FuriHalNfcLegicPoller));
    memset(furi_hal_nfc_legic_poller, 0, sizeof(FuriHalNfcLegicPoller));

    furi_hal_nfc_legic_poller->dbg_storage = furi_record_open(RECORD_STORAGE);
    furi_hal_nfc_legic_poller->dbg_tx_file =
        storage_file_alloc(furi_hal_nfc_legic_poller->dbg_storage);
    storage_file_open(
        furi_hal_nfc_legic_poller->dbg_tx_file, LEGIC_TX_DBG_LOG_PATH, FSAM_WRITE, FSOM_OPEN_APPEND);
    furi_hal_nfc_legic_poller->dbg_rx_file =
        storage_file_alloc(furi_hal_nfc_legic_poller->dbg_storage);
    storage_file_open(
        furi_hal_nfc_legic_poller->dbg_rx_file, LEGIC_RX_DBG_LOG_PATH, FSAM_WRITE, FSOM_OPEN_APPEND);

    // Subcarrier-stream mode + OOK: FIFO chip-bits are OOK-modulated directly
    // on TX, and RX demodulates a 212 kHz subcarrier into the FIFO.
    st25r3916_change_reg_bits(
        handle,
        ST25R3916_REG_MODE,
        ST25R3916_REG_MODE_om_mask | ST25R3916_REG_MODE_tr_am,
        ST25R3916_REG_MODE_om_subcarrier_stream | ST25R3916_REG_MODE_tr_am_ook);

    //  - scf_sc212: 212 kHz subcarrier (LEGIC tag).
    //  - scp_4pulses: one FIFO bit per ~9.44 µs (scp_1/scp_2 give false
    //    RxStart ~68 µs after TxEnd and no data).
    //  - stx_212: 4.72 µs/chip-bit (PM3 sim timing showed stx_424 = 2.36 µs).
    st25r3916_write_reg(
        handle,
        ST25R3916_REG_STREAM_MODE,
        ST25R3916_REG_STREAM_MODE_scf_sc212 | ST25R3916_REG_STREAM_MODE_stx_212 |
            ST25R3916_REG_STREAM_MODE_scp_4pulses);

    st25r3916_clear_reg_bits(
        handle,
        ST25R3916_REG_AUX_MOD,
        ST25R3916_REG_AUX_MOD_dis_reg_am | ST25R3916_REG_AUX_MOD_res_am);

    return furi_hal_nfc_legic_common_init(handle);
}

static FuriHalNfcError furi_hal_nfc_legic_poller_deinit(const FuriHalSpiBusHandle* handle) {
    UNUSED(handle);
    furi_check(furi_hal_nfc_legic_poller);

    legic_dbg_flush(furi_hal_nfc_legic_poller);
    if(furi_hal_nfc_legic_poller->dbg_tx_file) {
        storage_file_close(furi_hal_nfc_legic_poller->dbg_tx_file);
        storage_file_free(furi_hal_nfc_legic_poller->dbg_tx_file);
    }
    if(furi_hal_nfc_legic_poller->dbg_rx_file) {
        storage_file_close(furi_hal_nfc_legic_poller->dbg_rx_file);
        storage_file_free(furi_hal_nfc_legic_poller->dbg_rx_file);
    }
    if(furi_hal_nfc_legic_poller->dbg_storage) {
        furi_record_close(RECORD_STORAGE);
    }

    free(furi_hal_nfc_legic_poller);
    furi_hal_nfc_legic_poller = NULL;
    return FuriHalNfcErrorNone;
}

uint32_t furi_hal_nfc_legic_poller_get_frame_end(void) {
    furi_check(furi_hal_nfc_legic_poller);
    return furi_hal_nfc_legic_poller->t_frame_end;
}

void furi_hal_nfc_legic_poller_set_tx_start(uint32_t dwt_cycles) {
    furi_check(furi_hal_nfc_legic_poller);
    furi_hal_nfc_legic_poller->tx_start_at = dwt_cycles;
}

void furi_hal_nfc_legic_poller_set_rx_bits(size_t bits) {
    furi_check(furi_hal_nfc_legic_poller);
    furi_hal_nfc_legic_poller->rx_bits_expected = bits;
}

void furi_hal_nfc_legic_poller_set_rx_reduced_gain(bool reduced) {
    furi_check(furi_hal_nfc_legic_poller);
    furi_hal_nfc_legic_poller->rx_reduced_gain = reduced;
}

/*
 * LEGIC Prime pause-position encoder.
 *
 * In om_subcarrier_stream + OOK the chip emits one OOK symbol per FIFO
 * chip-bit (LSB-first): chip-bit 1 = pause (carrier off), 0 = carrier on.
 * At stx_212 one chip-bit is ~4.72 µs, so:
 *
 *   pause       = 4 chip-bits  = 18.9 µs
 *   '0' pulse   = 9 chip-bits  = 42.4 µs
 *   '1' pulse   = 17 chip-bits = 80.2 µs
 *
 * matching Proxmark legicrfsim.c RWD_TIME_* (in 4.72 µs ticks).
 */
#define LEGIC_PAUSE_CHIP_BITS  4U
#define LEGIC_BIT0_PULSE_CHIPS 9U
#define LEGIC_BIT1_PULSE_CHIPS 17U

typedef struct {
    uint8_t buf[FURI_HAL_NFC_LEGIC_POLLER_MAX_BUFFER_SIZE * 4];
    size_t bit_pos;
} LegicChipBitWriter;

static void legic_chip_bit_writer_emit(LegicChipBitWriter* w, bool bit, size_t count) {
    for(size_t i = 0; i < count; i++) {
        if(w->bit_pos >= sizeof(w->buf) * 8) return; // full
        if(bit) {
            w->buf[w->bit_pos / 8] |= (1u << (w->bit_pos % 8));
        }
        w->bit_pos++;
    }
}

static void legic_encode_bit(LegicChipBitWriter* w, bool legic_bit) {
    legic_chip_bit_writer_emit(w, true, LEGIC_PAUSE_CHIP_BITS);
    legic_chip_bit_writer_emit(
        w, false, legic_bit ? LEGIC_BIT1_PULSE_CHIPS : LEGIC_BIT0_PULSE_CHIPS);
}

static FuriHalNfcError furi_hal_nfc_legic_poller_tx(
    const FuriHalSpiBusHandle* handle,
    const uint8_t* tx_data,
    size_t tx_bits) {
    FuriHalNfcLegicPoller* instance = furi_hal_nfc_legic_poller;

    LegicChipBitWriter w = {0};

    // tx_data is LSB-first per the legic_prime_poller_tx_frame contract.
    for(size_t i = 0; i < tx_bits; i++) {
        bool b = (tx_data[i / 8] >> (i % 8)) & 1u;
        legic_encode_bit(&w, b);
    }
    // Closing pause: the tag decodes a bit when it sees the *next* pause.
    legic_chip_bit_writer_emit(&w, true, LEGIC_PAUSE_CHIP_BITS);

    // Round up to whole bytes for the chip FIFO; padding chip-bits are 0
    // (carrier on).
    instance->frame_buf_bits = ((w.bit_pos + 7) / 8) * 8;
    memcpy(instance->frame_buf, w.buf, instance->frame_buf_bits / 8);

    instance->rx_started = false;
    instance->seg_open = false;
    instance->seg_count = 0;
    instance->fifo_buf_bits = 0;
    instance->t_txe = 0;
    st25r3916_write_reg(
        handle, ST25R3916_REG_RX_CONF3, instance->rx_reduced_gain ? LEGIC_RX_CONF3_RETRY : 0x00);

    // Same preparation as furi_hal_nfc_poller_tx_common, with the TRANSMIT
    // command held back until the scheduled start.
    st25r3916_direct_cmd(handle, ST25R3916_CMD_CLEAR_FIFO);
    st25r3916_clear_reg_bits(
        handle, ST25R3916_REG_TIMER_EMV_CONTROL, ST25R3916_REG_TIMER_EMV_CONTROL_nrt_emv);
    st25r3916_change_reg_bits(
        handle,
        ST25R3916_REG_ISO14443A_NFC,
        (ST25R3916_REG_ISO14443A_NFC_no_tx_par | ST25R3916_REG_ISO14443A_NFC_no_rx_par),
        (ST25R3916_REG_ISO14443A_NFC_no_tx_par_off | ST25R3916_REG_ISO14443A_NFC_no_rx_par_off));
    uint32_t interrupts =
        (ST25R3916_IRQ_MASK_FWL | ST25R3916_IRQ_MASK_TXE | ST25R3916_IRQ_MASK_RXS |
         ST25R3916_IRQ_MASK_RXE | ST25R3916_IRQ_MASK_PAR | ST25R3916_IRQ_MASK_CRC |
         ST25R3916_IRQ_MASK_ERR1 | ST25R3916_IRQ_MASK_ERR2 | ST25R3916_IRQ_MASK_NRE);
    st25r3916_get_irq(handle);
    st25r3916_mask_irq(handle, ~interrupts);
    st25r3916_write_fifo(handle, instance->frame_buf, instance->frame_buf_bits);

    const uint32_t cmd_latency = LEGIC_TX_CMD_LATENCY_US *
                                 furi_hal_cortex_instructions_per_microsecond();
    instance->tx_wait_us = 0;
    if(instance->tx_start_at) {
        const uint32_t cmd_at = instance->tx_start_at - cmd_latency;
        int32_t wait = (int32_t)(cmd_at - DWT->CYCCNT);
        instance->tx_wait_us = legic_cycles_to_us(wait);
        if(wait > 0 && instance->tx_wait_us < (int32_t)LEGIC_TX_MAX_WAIT_US) {
            while((int32_t)(cmd_at - DWT->CYCCNT) > 0) {
            }
        }
        instance->tx_start_at = 0;
    }
    instance->t_tx_cmd = DWT->CYCCNT;
    st25r3916_direct_cmd(handle, ST25R3916_CMD_TRANSMIT_WITHOUT_CRC);
    instance->t_frame_end = instance->t_tx_cmd + cmd_latency + legic_chips_to_cycles(w.bit_pos);
    instance->tx_pad_chips = instance->frame_buf_bits - w.bit_pos;

    if(sizeof(instance->dbg_tx_buf) - instance->dbg_tx_len > 160) {
        char* line = instance->dbg_tx_buf + instance->dbg_tx_len;
        const size_t size = sizeof(instance->dbg_tx_buf) - instance->dbg_tx_len;
        int n = snprintf(
            line,
            size,
            "tx#%lu legic_bits=%u chip_bits=%u wait_us=%ld rx_bits=%u t_ms=%lu in:",
            (unsigned long)(instance->dbg_tx_seq++),
            (unsigned)tx_bits,
            (unsigned)w.bit_pos,
            (long)instance->tx_wait_us,
            (unsigned)instance->rx_bits_expected,
            (unsigned long)furi_get_tick());
        size_t in_bytes = (tx_bits + 7) / 8;
        for(size_t i = 0; i < in_bytes && n + 4 < (int)size; i++) {
            n += snprintf(line + n, size - n, " %02X", tx_data[i]);
        }
        n += snprintf(line + n, size - n, "\n");
        if(n > 0 && (size_t)n < size) instance->dbg_tx_len += n;
    }

    return FuriHalNfcErrorNone;
}

static uint32_t furi_hal_nfc_legic_rx_window_end(FuriHalNfcLegicPoller* instance) {
    uint32_t chips = instance->rx_bits_expected ?
                         LEGIC_TAG_FRAME_WAIT_CHIPS +
                             instance->rx_bits_expected * LEGIC_TAG_BIT_CHIPS + 10U :
                         LEGIC_RX_WINDOW_CHIPS;
    return instance->t_frame_end + legic_chips_to_cycles(chips);
}

// Append the FIFO content of one finished reception to fifo_buf and re-arm
// the receiver for the next run of '1' bits.
static void furi_hal_nfc_legic_collect_segment(FuriHalNfcLegicPoller* instance, uint32_t now) {
    const FuriHalSpiBusHandle* handle = &furi_hal_spi_bus_handle_nfc;
    uint8_t seg[64];
    size_t seg_bits = 0;
    st25r3916_read_fifo(handle, seg, sizeof(seg), &seg_bits);
    instance->seg_open = false;

    if((int32_t)(now - furi_hal_nfc_legic_rx_window_end(instance)) < 0) {
        st25r3916_direct_cmd(handle, ST25R3916_CMD_UNMASK_RECEIVE_DATA);
    }

    // Byte-align each segment in fifo_buf so segments stay readable in the log.
    size_t pos = (instance->fifo_buf_bits + 7) / BITS_IN_BYTE;
    size_t seg_bytes = (seg_bits + 7) / BITS_IN_BYTE;
    if(seg_bytes > sizeof(seg)) seg_bytes = sizeof(seg);
    if(instance->seg_count >= LEGIC_RX_MAX_SEGMENTS) return;
    if(pos + seg_bytes > sizeof(instance->fifo_buf)) return;

    memcpy(&instance->fifo_buf[pos], seg, seg_bytes);
    instance->fifo_buf_bits = pos * BITS_IN_BYTE + seg_bits;
    instance->seg_t_rxe[instance->seg_count] = now;
    instance->seg_bits[instance->seg_count] = seg_bits;
    instance->seg_off[instance->seg_count] = pos;
    instance->seg_count++;
}

static FuriHalNfcEvent furi_hal_nfc_legic_poller_wait_event(uint32_t timeout_ms) {
    FuriHalNfcLegicPoller* instance = furi_hal_nfc_legic_poller;

    if(!instance->rx_started) {
        FuriHalNfcEvent event = furi_hal_nfc_wait_event_common(timeout_ms);
        uint32_t now = DWT->CYCCNT;
        if(event & FuriHalNfcEventTxEnd) instance->t_txe = now;
        if(event & FuriHalNfcEventRxStart) {
            instance->rx_started = true;
            instance->seg_open = true;
            instance->seg_t_rxs[0] = now;
            instance->t_last_rx_event = now;
            if(event & FuriHalNfcEventRxEnd) furi_hal_nfc_legic_collect_segment(instance, now);
            // Report only RxStart now; RxEnd is reported once collection ends.
            event &= ~FuriHalNfcEventRxEnd;
        }
        return event;
    }

    // Collecting: swallow further RxStart/RxEnd until the reply window has
    // passed or the line stays quiet.
    while(true) {
        FuriHalNfcEvent event = furi_hal_nfc_wait_event_common(1);
        uint32_t now = DWT->CYCCNT;
        if(event & (FuriHalNfcEventRxStart | FuriHalNfcEventRxEnd)) instance->t_last_rx_event = now;
        if(event & FuriHalNfcEventRxStart) {
            instance->seg_open = true;
            if(instance->seg_count < LEGIC_RX_MAX_SEGMENTS) {
                instance->seg_t_rxs[instance->seg_count] = now;
            }
        }
        if(event & FuriHalNfcEventRxEnd) furi_hal_nfc_legic_collect_segment(instance, now);

        bool window_over = (int32_t)(now - furi_hal_nfc_legic_rx_window_end(instance)) > 0;
        bool gap_over = legic_cycles_to_us(now - instance->t_last_rx_event) >
                        (int32_t)(LEGIC_RX_GAP_MS * 1000U);
        if((window_over && !instance->seg_open) || gap_over ||
           (event & FuriHalNfcEventAbortRequest)) {
            if(instance->seg_open) {
                // A reception started but never ended: stop it and keep its data.
                st25r3916_direct_cmd(&furi_hal_spi_bus_handle_nfc, ST25R3916_CMD_STOP);
                furi_hal_nfc_legic_collect_segment(instance, now);
            }
            instance->rx_started = false;
            return FuriHalNfcEventRxEnd | (event & FuriHalNfcEventAbortRequest);
        }
    }
}

/*
 * Place the received runs into LEGIC bit windows.  Bit k occupies
 * [t0 + k*99.1 µs, t0 + (k+1)*99.1 µs) with t0 330 µs after our closing
 * pause; it is '1' when FIFO '1' samples, shifted back by the detection lag,
 * cover at least half of the window.  (A 40% threshold without the lag
 * misread MIM1024's card type 110110 as 111110 in 25% of replies.)
 */
static size_t furi_hal_nfc_legic_decode(
    FuriHalNfcLegicPoller* instance,
    uint8_t* out,
    size_t out_size,
    size_t max_bits) {
    memset(out, 0, out_size);
    const uint32_t t0 =
        instance->t_frame_end + legic_chips_to_cycles(LEGIC_TAG_FRAME_WAIT_CHIPS);
    const int32_t tb = legic_chips_to_cycles(LEGIC_TAG_BIT_CHIPS);
    const int32_t fb = legic_chips_to_cycles(LEGIC_FIFO_BIT_CHIPS);
    const int32_t lag_first = legic_chips_to_cycles(LEGIC_RX_LAG_CHIPS);
    const int32_t lag_rearm = legic_chips_to_cycles(LEGIC_RX_REARM_LAG_CHIPS);
    const int32_t rearm_short = legic_chips_to_cycles(LEGIC_RX_REARM_SHORT_CHIPS);

    size_t nbits = max_bits;
    if(nbits > out_size * BITS_IN_BYTE) nbits = out_size * BITS_IN_BYTE;

    size_t last_one = 0;
    for(size_t k = 0; k < nbits; k++) {
        const int32_t w0 = k * tb;
        const int32_t w1 = w0 + tb;
        int32_t ones = 0;
        for(size_t i = 0; i < instance->seg_count; i++) {
            const bool just_rearmed =
                i && (int32_t)(instance->seg_t_rxs[i] - instance->seg_t_rxe[i - 1]) < rearm_short;
            int32_t s = (int32_t)(instance->seg_t_rxs[i] - t0) -
                        (just_rearmed ? lag_rearm : lag_first);
            const uint8_t* seg = &instance->fifo_buf[instance->seg_off[i]];
            for(size_t j = 0; j < instance->seg_bits[i]; j++) {
                if(!((seg[j / 8] >> (j % 8)) & 1u)) continue;
                int32_t a = s + (int32_t)j * fb;
                int32_t b = a + fb;
                if(a < w0) a = w0;
                if(b > w1) b = w1;
                if(b > a) ones += b - a;
            }
        }
        if(ones * 2 >= tb) {
            out[k / BITS_IN_BYTE] |= 1u << (k % BITS_IN_BYTE);
            last_one = k + 1;
        }
    }
    return instance->rx_bits_expected ? nbits : last_one;
}

static FuriHalNfcError furi_hal_nfc_legic_poller_rx(
    const FuriHalSpiBusHandle* handle,
    uint8_t* rx_data,
    size_t rx_data_size,
    size_t* rx_bits) {
    UNUSED(handle);
    FuriHalNfcLegicPoller* instance = furi_hal_nfc_legic_poller;

    size_t max_bits = instance->rx_bits_expected ? instance->rx_bits_expected : 23U;
    *rx_bits = furi_hal_nfc_legic_decode(instance, rx_data, rx_data_size, max_bits);

    if(sizeof(instance->dbg_rx_buf) - instance->dbg_rx_len < 600 ||
       sizeof(instance->dbg_tx_buf) - instance->dbg_tx_len < 400) {
        legic_dbg_flush(instance);
    }
    {
        const uint32_t t0 =
            instance->t_frame_end + legic_chips_to_cycles(LEGIC_TAG_FRAME_WAIT_CHIPS);
        char line[600];
        int n = snprintf(
            line,
            sizeof(line),
            "rx#%lu reduced_gain=%u want=%u got=%u txe_late_us=%ld segs=%u t_ms=%lu",
            (unsigned long)(instance->dbg_rx_seq++),
            instance->rx_reduced_gain,
            (unsigned)instance->rx_bits_expected,
            (unsigned)*rx_bits,
            (long)legic_cycles_to_us(
                instance->t_txe - instance->t_frame_end -
                legic_chips_to_cycles(instance->tx_pad_chips)),
            (unsigned)instance->seg_count,
            (unsigned long)furi_get_tick());
        // Per segment: start/end µs relative to the expected reply start (t0).
        for(size_t i = 0; i < instance->seg_count && n + 40 < (int)sizeof(line); i++) {
            n += snprintf(
                line + n,
                sizeof(line) - n,
                " [%ld..%ld %ub]",
                (long)legic_cycles_to_us(instance->seg_t_rxs[i] - t0),
                (long)legic_cycles_to_us(instance->seg_t_rxe[i] - t0),
                (unsigned)instance->seg_bits[i]);
        }
        n += snprintf(line + n, sizeof(line) - n, " raw:");
        size_t bytes = (instance->fifo_buf_bits + 7) / BITS_IN_BYTE;
        for(size_t i = 0; i < bytes && n + 4 < (int)sizeof(line); i++) {
            n += snprintf(line + n, sizeof(line) - n, " %02X", instance->fifo_buf[i]);
        }
        n += snprintf(line + n, sizeof(line) - n, " dec:");
        for(size_t k = 0; k < *rx_bits && n + 3 < (int)sizeof(line); k++) {
            n += snprintf(
                line + n, sizeof(line) - n, "%u", (rx_data[k / 8] >> (k % 8)) & 1u);
        }
        n += snprintf(line + n, sizeof(line) - n, "\n");
        if(n > 0 && (size_t)n < sizeof(line) &&
           instance->dbg_rx_len + n <= sizeof(instance->dbg_rx_buf)) {
            memcpy(instance->dbg_rx_buf + instance->dbg_rx_len, line, n);
            instance->dbg_rx_len += n;
        }
    }

    return FuriHalNfcErrorNone;
}


const FuriHalNfcTechBase furi_hal_nfc_legic = {
    .poller =
        {
            .compensation =
                {
                    .fdt = FURI_HAL_NFC_POLLER_FDT_COMP_FC,
                    .fwt = FURI_HAL_NFC_LEGIC_POLLER_FWT_COMP_FC,
                },
            .init = furi_hal_nfc_legic_poller_init,
            .deinit = furi_hal_nfc_legic_poller_deinit,
            .wait_event = furi_hal_nfc_legic_poller_wait_event,
            .tx = furi_hal_nfc_legic_poller_tx,
            .rx = furi_hal_nfc_legic_poller_rx,
        },
    // Listener not implemented.
    .listener =
        {
            .compensation = {0},
            .init = NULL,
            .deinit = NULL,
            .wait_event = NULL,
            .tx = NULL,
            .rx = NULL,
            .sleep = NULL,
            .idle = NULL,
        },
};
