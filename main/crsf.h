#ifndef CRSF_H
#define CRSF_H

#include "crsf_protocol.h"
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>


/**
 * CRSF protocol description: https://github.com/crsf-wg/crsf/wiki/Message-Format
 */

#ifndef CRSF_SYNC_BYTE
#define CRSF_SYNC_BYTE      0xC8 // defined in crsf_protocol.h
#endif
// Define channel input limite
#define CRSF_CHANNEL_MIN    172
#define CRSF_CHANNEL_MID    992
#define CRSF_CHANNEL_MAX    1811
#define CRSF_MAX_CHANNEL    16
#define CRSF_FRAME_SIZE_MAX 64
#define CRSF_FRAME_SIZE_RC  26
#define BUFFER_SIZE         256
#define MAX_EVENTS          10

typedef struct {
    uint8_t deviceAddress;
    uint8_t frameLength;
    uint8_t type;
    uint8_t payload[CRSF_PAYLOAD_SIZE_MAX];
    uint8_t crc;
} crsfFrameDef_t;

typedef union {
    uint8_t bytes[CRSF_FRAME_SIZE_MAX];
    crsfFrameDef_t frame;
} crsfFrame_t;

struct crsfPayloadRcChannelsPacked_s {
    // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
    unsigned int chan0 : 11;
    unsigned int chan1 : 11;
    unsigned int chan2 : 11;
    unsigned int chan3 : 11;
    unsigned int chan4 : 11;
    unsigned int chan5 : 11;
    unsigned int chan6 : 11;
    unsigned int chan7 : 11;
    unsigned int chan8 : 11;
    unsigned int chan9 : 11;
    unsigned int chan10 : 11;
    unsigned int chan11 : 11;
    unsigned int chan12 : 11;
    unsigned int chan13 : 11;
    unsigned int chan14 : 11;
    unsigned int chan15 : 11;
} __attribute__ ((__packed__));

typedef uint32_t rcChannelsUnpacked_t[CRSF_MAX_CHANNEL];

typedef struct crsfPayloadRcChannelsPacked_s crsfPayloadRcChannelsPacked_t;

typedef uint8_t crsfBufPacked_t[64];

enum {
    OFFSET_SYNC,
    OFFSET_LEN,
    OFFSET_TYPE,
    OFFSET_PAYLOAD
};

enum {
    CRSF_OK = 0,                    ///< Operation successful
    CRSF_ERROR_UKNOWN = -1,         ///< Unknown error
    CRSF_ERROR_IOCTL_TCGETS2 = -2,  ///< Error in ioctl TCGETS2
    CRSF_ERROR_IOCTL_TCSETS2 = -3,  ///< Error in ioctl TCSETS2
    CRSF_ERROR_OPEN_SERIAL = -4,    ///< Error opening serial port
    CRSF_ERROR_EPOLL_CREATE1 = -5,  ///< Error creating epoll instance
    CRSF_ERROR_EPOLL_CTL = -6,      ///< Error in epoll control
    CRSF_ERROR_EPOLL_WAIT = -7,     ///< Error in epoll wait
    CRSF_ERROR_SERIAL_READ = -8,    ///< Error reading from serial port
    CRSF_ERROR_FCNTL_GETFL = -9,    ///< Error in fcntl get flags
    CRSF_ERROR_FCNTL_SETFL = -10    ///< Error in fcntl set flags
};


// extern crsfFrame_t crsf_in_frame;
extern crsfFrame_t crsf_out_frame;

void pack_RC_channels(crsfFrame_t* crsfFrame, const rcChannelsUnpacked_t unpackedChannels);
int packed_buffer(const crsfFrame_t* crsfFrame, crsfBufPacked_t buffer);
void build_frame(rcChannelsUnpacked_t channels, uint8_t* out_buff, int len);


#endif // CRSF_H
