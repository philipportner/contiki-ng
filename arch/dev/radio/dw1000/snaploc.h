#ifndef _SNAPLOC_PACKET
#define _SNAPLOC_PACKET
#include <stdint.h>

// IEEE 802.15.4 frame
#pragma pack(1)
typedef struct {
    uint8_t frame_ctrl[2];
    uint8_t sequence_number;
    uint8_t short_d_ad[2];
    uint8_t short_s_ad[2];
    uint8_t payload[120];
} frame_802154_t;
#pragma pack()

#define FRAME_HEADER_LEN 7

typedef enum { SYNC=1, LOCAL=2, DATA=3, TWR=4, TDOA_MEAS=5, TEST=6, SELF_LOCAL=7, SYNC_RESP=8, INIT=9, RESP=10} enum_packets_t;
typedef enum { SYNC_T_NONE = 0, SYNC_T_SKIP=1, SYNC_T_LOCAL=2, SYNC_T_RET=3 } enum_sync_t;                       // Synchronization interval type.
                                                                               // Skip means that anchor and mobile nodes should not send within this interval
                                                                               // Local means that mobile nodes are allowed to send localization messages
                                                                               // Ret means that anchor nodes are allowed to return received data to the
                                                                               // reference node
//settings struct
#pragma pack(1)
typedef struct {
    uint8_t packet_type;
    uint32_t pkt_id;
    uint8_t txt[5];
    enum_sync_t interval_type;
    uint8_t crc[2];
} packet_sync_t;
#pragma pack()

//settings struct
#pragma pack(1)
typedef struct {
    uint8_t packet_type;
    uint8_t crc[2];
} packet_init_t;
#pragma pack()


#pragma pack(1)
typedef struct {
    uint8_t packet_type;
    uint32_t pkt_id;
    uint8_t crc[2];
} packet_local_t;
#pragma pack()

#pragma pack(1)
typedef struct {
    uint8_t packet_type;
    uint32_t pkt_id;
    uint32_t twr_pkt_id;
    uint8_t tproc[5];
    uint8_t tproc_c[5];
    uint8_t tproc_c_simple[5];
    uint8_t txt[5];
    uint8_t rxt[6];
    uint8_t crc[2];
} packet_twr_t;
#pragma pack()

#pragma pack(1)
typedef struct {
    uint8_t packet_type;
    uint8_t crc[2];
} packet_test_t;
#pragma pack()

#pragma pack(1)
typedef struct {
    uint8_t packet_type;
    uint8_t anchor_id;
    uint32_t local_packet_id;
    //uint8 rxt[6];
    uint64_t rxt;
    uint8_t crc[2];
} packet_tdoa_meas_t;
#pragma pack()

#pragma pack(1)
typedef struct {
    uint8_t packet_type;
    uint32_t sync_id;
    uint8_t crc[2];
} packet_self_local_t;
#pragma pack()

#pragma pack(1)
typedef struct {
    uint8_t packet_type;
    uint32_t sync_id;
    uint8_t anchor_id;
    uint8_t crc[2];
} packet_sync_resp_t;
#pragma pack()

#pragma pack(1)
typedef struct {
	uint8_t packet_type;
//    uint8_t data[20];
    uint8_t crc[2];
} packet_resp_conc_t;
#pragma pack()

#endif /* _SNAPLOC_PACKET */
