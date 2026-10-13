#include "legic_prime.h"
#include "legic_prime_crc.h"

#include <flipper_format/flipper_format.h>
#include <furi.h>

#include <nfc/nfc_common.h>

#define LEGIC_PRIME_PROTOCOL_NAME "LEGIC Prime"
#define LEGIC_PRIME_TYPE_KEY      "Legic Prime Type"
#define LEGIC_PRIME_DATA_KEY      "Data"

typedef struct {
    uint16_t mem_size;
    const char* type_name;
    const char* full_name;
} LegicPrimeFeatures;

static const LegicPrimeFeatures legic_prime_features[LegicPrimeTypeNum] = {
    [LegicPrimeTypeMim22] =
        {
            .mem_size = LEGIC_PRIME_MEM_SIZE_MIM22,
            .type_name = "MIM22",
            .full_name = "LEGIC Prime MIM22",
        },
    [LegicPrimeTypeMim256] =
        {
            .mem_size = LEGIC_PRIME_MEM_SIZE_MIM256,
            .type_name = "MIM256",
            .full_name = "LEGIC Prime MIM256",
        },
    [LegicPrimeTypeMim1024] =
        {
            .mem_size = LEGIC_PRIME_MEM_SIZE_MIM1024,
            .type_name = "MIM1024",
            .full_name = "LEGIC Prime MIM1024",
        },
};

const NfcDeviceBase nfc_device_legic_prime = {
    .protocol_name = LEGIC_PRIME_PROTOCOL_NAME,
    .alloc = (NfcDeviceAlloc)legic_prime_alloc,
    .free = (NfcDeviceFree)legic_prime_free,
    .reset = (NfcDeviceReset)legic_prime_reset,
    .copy = (NfcDeviceCopy)legic_prime_copy,
    .verify = (NfcDeviceVerify)legic_prime_verify,
    .load = (NfcDeviceLoad)legic_prime_load,
    .save = (NfcDeviceSave)legic_prime_save,
    .is_equal = (NfcDeviceEqual)legic_prime_is_equal,
    .get_name = (NfcDeviceGetName)legic_prime_get_device_name,
    .get_uid = (NfcDeviceGetUid)legic_prime_get_uid,
    .set_uid = (NfcDeviceSetUid)legic_prime_set_uid,
    .get_base_data = (NfcDeviceGetBaseData)legic_prime_get_base_data,
};

LegicPrimeData* legic_prime_alloc(void) {
    LegicPrimeData* data = malloc(sizeof(LegicPrimeData));
    legic_prime_reset(data);
    return data;
}

void legic_prime_free(LegicPrimeData* data) {
    furi_check(data);
    free(data);
}

void legic_prime_reset(LegicPrimeData* data) {
    furi_check(data);
    memset(data, 0, sizeof(LegicPrimeData));
}

void legic_prime_copy(LegicPrimeData* data, const LegicPrimeData* other) {
    furi_check(data);
    furi_check(other);
    *data = *other;
}

bool legic_prime_verify(LegicPrimeData* data, const FuriString* device_type) {
    UNUSED(data);
    furi_check(device_type);
    return furi_string_equal_str(device_type, LEGIC_PRIME_PROTOCOL_NAME);
}

bool legic_prime_load(LegicPrimeData* data, FlipperFormat* ff, uint32_t version) {
    furi_check(data);
    furi_check(ff);

    bool parsed = false;
    FuriString* tmp = furi_string_alloc();

    do {
        if(version < NFC_UNIFIED_FORMAT_VERSION) break;
        if(!flipper_format_read_string(ff, LEGIC_PRIME_TYPE_KEY, tmp)) break;

        bool type_parsed = false;
        for(size_t i = 0; i < LegicPrimeTypeNum; i++) {
            if(furi_string_equal_str(tmp, legic_prime_features[i].type_name)) {
                data->type = (LegicPrimeType)i;
                data->data_size = legic_prime_features[i].mem_size;
                type_parsed = true;
                break;
            }
        }
        if(!type_parsed) break;

        if(!flipper_format_read_hex(ff, LEGIC_PRIME_DATA_KEY, data->data, data->data_size)) break;

        parsed = true;
    } while(false);

    furi_string_free(tmp);
    return parsed;
}

bool legic_prime_save(const LegicPrimeData* data, FlipperFormat* ff) {
    furi_check(data);
    furi_check(ff);
    furi_check(data->type < LegicPrimeTypeNum);

    bool saved = false;

    do {
        if(!flipper_format_write_comment_cstr(ff, LEGIC_PRIME_PROTOCOL_NAME " specific data"))
            break;
        if(!flipper_format_write_string_cstr(
               ff, LEGIC_PRIME_TYPE_KEY, legic_prime_features[data->type].type_name))
            break;
        if(!flipper_format_write_hex(ff, LEGIC_PRIME_DATA_KEY, data->data, data->data_size)) break;

        saved = true;
    } while(false);

    return saved;
}

bool legic_prime_is_equal(const LegicPrimeData* data, const LegicPrimeData* other) {
    furi_check(data);
    furi_check(other);
    return memcmp(data, other, sizeof(LegicPrimeData)) == 0; //-V1103
}

const char* legic_prime_get_device_name(const LegicPrimeData* data, NfcDeviceNameType name_type) {
    furi_check(data);
    furi_check(data->type < LegicPrimeTypeNum);
    return name_type == NfcDeviceNameTypeFull ? legic_prime_features[data->type].full_name :
                                                legic_prime_features[data->type].type_name;
}

const uint8_t* legic_prime_get_uid(const LegicPrimeData* data, size_t* uid_len) {
    furi_check(data);
    if(uid_len) *uid_len = LEGIC_PRIME_UID_SIZE;
    return data->data;
}

bool legic_prime_set_uid(LegicPrimeData* data, const uint8_t* uid, size_t uid_len) {
    furi_check(data);
    furi_check(uid);
    if(uid_len != LEGIC_PRIME_UID_SIZE) return false;
    memcpy(data->data, uid, LEGIC_PRIME_UID_SIZE);
    // Recompute MCC so the saved tag stays internally consistent.
    data->data[4] = legic_prime_crc8(data->data, LEGIC_PRIME_UID_SIZE);
    return true;
}

LegicPrimeData* legic_prime_get_base_data(const LegicPrimeData* data) {
    UNUSED(data);
    furi_crash("No base data");
}

uint16_t legic_prime_get_mem_size(LegicPrimeType type) {
    furi_check(type < LegicPrimeTypeNum);
    return legic_prime_features[type].mem_size;
}

bool legic_prime_verify_mcc(const LegicPrimeData* data) {
    furi_check(data);
    return legic_prime_crc8(data->data, LEGIC_PRIME_UID_SIZE) == data->data[4];
}
