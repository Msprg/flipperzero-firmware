#include "legic_prime_render.h"

void nfc_render_legic_prime_info(
    const LegicPrimeData* data,
    NfcProtocolFormatType format_type,
    FuriString* str) {
    furi_string_cat_printf(str, "UID:");
    for(size_t i = 0; i < LEGIC_PRIME_UID_SIZE; i++) {
        furi_string_cat_printf(str, " %02X", data->data[i]);
    }
    furi_string_cat_printf(str, "\nMCC: %02X (%s)", data->data[4],
        legic_prime_verify_mcc(data) ? "OK" : "FAIL");

    uint16_t dcf = (uint16_t)data->data[5] | ((uint16_t)data->data[6] << 8);
    const char* dcf_class;
    if(dcf == 0xFFFF)
        dcf_class = "New/Blank";
    else if(dcf > 60000)
        dcf_class = "Master Token";
    else
        dcf_class = "IM";
    furi_string_cat_printf(str, "\nDCF: %04X (%s)", dcf, dcf_class);

    if(format_type == NfcProtocolFormatTypeFull) {
        furi_string_cat_printf(str, "\nMemory: %u bytes", (unsigned)data->data_size);
    }
}

void nfc_render_legic_prime_dump(const LegicPrimeData* data, FuriString* str) {
    // The font escape applies to one line only.
    for(size_t i = 0; i < data->data_size; i++) {
        if((i % 4) == 0) furi_string_cat_printf(str, "%s\e*%03X:", i ? "\n" : "", (unsigned)i);
        furi_string_cat_printf(str, " %02X", data->data[i]);
    }
}
