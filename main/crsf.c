#include "crsf.h"
#include <string.h>
#include <stdio.h>
// #include <unistd.h>

#include "crc.h"


#define CRSF_FRAME_SIZE_MAX 64
#define EXIT_FAILURE        1

#define HZ_TO_MICROS(hz) (1000000 / (hz))

//#define DEBUG_PRINT

static crsfFrame_t crsfChannelDataFrame;

// static int loop_hz = 1000;

crsfFrame_t crsf_in_frame;
crsfFrame_t crsf_out_frame;
int crsf_in_flag = 0;
int crsf_out_flag = 0;
int crsf_thread_exit_code = CRSF_OK;


uint8_t crsf_calc_crc(const crsfFrame_t *frame)
{
    // CRC includes type and payload
    uint8_t crc = crc8_dvb_s2(0, frame->frame.type);
    for (int ii = 0; ii < frame->frame.frameLength - CRSF_FRAME_LENGTH_TYPE_CRC; ++ii) {
        crc = crc8_dvb_s2(crc, frame->frame.payload[ii]);
    }
    return crc;
}


/**
 * Pack channels from regular (uint32) C-array to CRSF frame in place and calc and set CRC
 * @param crsfFrame to CRSF frame of crsfFrame_t type
 * @param unpackedChannels from channels array of rcChannelsUnpacked_t type (uint32 * 16)
 */
void pack_RC_channels(crsfFrame_t* crsfFrame, const rcChannelsUnpacked_t unpackedChannels) {

    // cast pointer so changes on new pointer also change original data
    crsfPayloadRcChannelsPacked_t* packedChannels = (crsfPayloadRcChannelsPacked_t*)&crsfFrame->frame.payload;

    // Assign the channel values to the packed structure
    packedChannels->chan0  = unpackedChannels[ 0];
    packedChannels->chan1  = unpackedChannels[ 1];
    packedChannels->chan2  = unpackedChannels[ 2];
    packedChannels->chan3  = unpackedChannels[ 3];
    packedChannels->chan4  = unpackedChannels[ 4];
    packedChannels->chan5  = unpackedChannels[ 5];
    packedChannels->chan6  = unpackedChannels[ 6];
    packedChannels->chan7  = unpackedChannels[ 7];
    packedChannels->chan8  = unpackedChannels[ 8];
    packedChannels->chan9  = unpackedChannels[ 9];
    packedChannels->chan10 = unpackedChannels[10];
    packedChannels->chan11 = unpackedChannels[11];
    packedChannels->chan12 = unpackedChannels[12];
    packedChannels->chan13 = unpackedChannels[13];
    packedChannels->chan14 = unpackedChannels[14];
    packedChannels->chan15 = unpackedChannels[15];

    crsfFrame->frame.crc = crsf_calc_crc(crsfFrame);

}

int packed_buffer(const crsfFrame_t* crsfFrame, crsfBufPacked_t buffer) {
    static int len = 0;
    buffer[0] = crsfFrame->frame.deviceAddress;
    buffer[1] = crsfFrame->frame.frameLength;
    buffer[2] = crsfFrame->frame.type;
    memcpy(buffer + 3, crsfFrame->frame.payload, crsfFrame->frame.frameLength - CRSF_FRAME_LENGTH_TYPE_CRC);
    buffer[crsfFrame->frame.frameLength + 1] = crsf_calc_crc(crsfFrame);
    len = CRSF_FRAME_LENGTH_NON_PAYLOAD + crsfFrame->frame.frameLength - CRSF_FRAME_LENGTH_TYPE_CRC;
    return len;
}

void build_frame(rcChannelsUnpacked_t channels, uint8_t* out_buff, int len) {
#ifdef DEBUG_PRINT
    printf("[crsf.c] Send Channels: ");
    for (int i = 0; i < CRSF_MAX_CHANNEL; ++i) {
        printf("%u ", channels[i]);
    }
    printf("\n");
#endif
    crsfChannelDataFrame.frame.deviceAddress = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    crsfChannelDataFrame.frame.type = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
    crsfChannelDataFrame.frame.frameLength = CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC;
    pack_RC_channels(&crsfChannelDataFrame, channels);
    // memcpy(&crsf_out_frame, &crsfChannelDataFrame, sizeof(crsfFrame_t));
    if (len >= CRSF_FRAME_SIZE_RC) {
        memcpy(out_buff, crsfChannelDataFrame.bytes, CRSF_FRAME_SIZE_RC - 1);
        out_buff[25] = crsfChannelDataFrame.frame.crc;
    }
}
