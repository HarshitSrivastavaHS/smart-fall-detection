#include "nfc.h"
#include <string.h>
#include <stdio.h>

#define NFC_INSTANCE 0
#define NFC_NDEF_START 0x0004  // IMPORTANT: NDEF starts at 0x0008

void NFC_Init(void)
{
    BSP_NFCTAG_Init(NFC_INSTANCE);

    // Write Capability Container (Type 5 Tag)
    uint8_t cc_file[4] = {0xE1, 0x40, 0x40, 0x01};
    BSP_NFCTAG_WriteData(NFC_INSTANCE, cc_file, 0x0000, 4);

    BSP_NFCTAG_ResetRFDisable_Dyn(NFC_INSTANCE);
}

void NFC_WriteURL(const char *url)
{
    uint8_t buffer[256];
    uint8_t empty[256];

    memset(buffer, 0, sizeof(buffer));

    uint16_t url_len = strlen(url);
    if (url_len > 200) url_len = 200;

    uint8_t payload_length = url_len + 1;
    uint8_t record_length  = 4 + payload_length;
    uint8_t tlv_length     = record_length;

    buffer[0] = 0x03;
    buffer[1] = tlv_length;
    buffer[2] = 0xD1;
    buffer[3] = 0x01;
    buffer[4] = payload_length;
    buffer[5] = 0x55;
    buffer[6] = 0x01;

    memcpy(&buffer[7], url, url_len);

    buffer[7 + url_len] = 0xFE;

    BSP_NFCTAG_WriteData(
        NFC_INSTANCE,
        buffer,
        NFC_NDEF_START,
        2 + tlv_length + 1
    );
}
