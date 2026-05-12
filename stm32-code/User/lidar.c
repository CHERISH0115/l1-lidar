#include "lidar.h"
#include <string.h>

/* =========================== Ring Buffer =========================== */
#define RING_SIZE 1024
#define RING_MASK (RING_SIZE - 1)

static volatile uint8_t  rx_ring[RING_SIZE];
static volatile uint16_t rx_head = 0;
static          volatile uint16_t rx_tail = 0;

void lidar_ring_push(uint8_t byte)
{
    uint16_t next = (rx_head + 1) & RING_MASK;
    if (next != rx_tail) {
        rx_ring[rx_head] = byte;
        rx_head = next;
    }
}

uint16_t lidar_ring_available(void)
{
    return (uint16_t)(rx_head - rx_tail) & RING_MASK;
}

uint8_t lidar_ring_pop(uint8_t *byte)
{
    if (rx_head == rx_tail) return 0;
    *byte = rx_ring[rx_tail];
    rx_tail = (rx_tail + 1) & RING_MASK;
    return 1;
}

/* =========================== Global Data =========================== */
volatile LidarImuData  g_imu   = {0};
volatile LidarDistData g_dist  = {0};
volatile LidarAuxData  g_aux   = {0};
volatile uint32_t      g_last_lidar_byte_ms = 0;
volatile uint8_t       g_lidar_connected    = 0;

/* ========================== CRC-16/MCRF4XX ======================== */
static uint16_t crc_accumulate(uint16_t crc, uint8_t byte)
{
    uint8_t tmp = byte ^ (uint8_t)(crc & 0xFF);
    tmp ^= (tmp << 4);
    return (crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ ((uint16_t)tmp >> 4);
}

static uint16_t mavlink_crc(const uint8_t *buf, uint16_t len, uint8_t crc_extra)
{
    uint16_t crc = 0xFFFF;
    uint16_t i;
    for (i = 0; i < len; i++) {
        crc = crc_accumulate(crc, buf[i]);
    }
    crc = crc_accumulate(crc, crc_extra);
    return crc;
}

static uint8_t crc_extra_for_msgid(uint32_t msgid)
{
    switch (msgid) {
        case 16: return 74;   /* RET_LIDAR_DISTANCE_DATA_PACKET */
        case 17: return 99;   /* RET_LIDAR_AUXILIARY_DATA_PACKET */
        case 19: return 110;  /* RET_IMU_ATTITUDE_DATA_PACKET */
        default: return 0;
    }
}

/* ====================== Little-endian helpers ====================== */
static float    le_float(const uint8_t *p)    { float v; memcpy(&v, p, 4); return v; }
static uint16_t le_u16(const uint8_t *p)      { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static uint32_t le_u24(const uint8_t *p)      { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16); }

/* ===================== MAVLink v2 Parser =========================== */
#define MAV_STX 0xFD

typedef enum {
    MAV_STATE_STX,
    MAV_STATE_HEADER,
    MAV_STATE_PAYLOAD,
    MAV_STATE_CRC,
} MavState;

static MavState mav_state   = MAV_STATE_STX;
static uint8_t  mav_buf[284];  /* max MAVLink v2 frame: 12 header + 255 payload + 2 CRC + 13 sig => 282 */
static uint16_t mav_idx     = 0;
static uint8_t  mav_payload_len = 0;
static uint8_t  mav_flags    = 0;

static void mavlink_dispatch(uint32_t msgid, const uint8_t *payload, uint8_t len);

void mavlink_feed_byte(uint8_t byte)
{
    switch (mav_state) {

    case MAV_STATE_STX:
        if (byte == MAV_STX) {
            mav_buf[0] = byte;
            mav_idx = 1;
            mav_state = MAV_STATE_HEADER;
        }
        break;

    case MAV_STATE_HEADER:
        mav_buf[mav_idx++] = byte;
        if (mav_idx == 10) {
            /* mav_buf[1] = payload_len */
            mav_payload_len = mav_buf[1];
            mav_flags = mav_buf[2];
            if (mav_payload_len == 0) {
                mav_state = MAV_STATE_CRC;
            } else {
                mav_state = MAV_STATE_PAYLOAD;
            }
        }
        break;

    case MAV_STATE_PAYLOAD:
        mav_buf[mav_idx++] = byte;
        if (mav_idx >= (uint16_t)(10 + mav_payload_len)) {
            mav_state = MAV_STATE_CRC;
        }
        break;

    case MAV_STATE_CRC:
        mav_buf[mav_idx++] = byte;
        if (mav_idx >= (uint16_t)(10 + mav_payload_len + 2)) {
            /* Full frame received. Verify CRC. */
            uint32_t msgid = le_u24(mav_buf + 7);
            uint8_t  extra = crc_extra_for_msgid(msgid);

            if (extra != 0) {
                uint16_t calc = mavlink_crc(mav_buf + 1, 9 + mav_payload_len, extra);
                uint16_t recv = le_u16(mav_buf + 10 + mav_payload_len);
                if (calc == recv) {
                    mavlink_dispatch(msgid, mav_buf + 10, mav_payload_len);
                }
            } else {
                /* Unknown msgid: parse without CRC check */
                mavlink_dispatch(msgid, mav_buf + 10, mav_payload_len);
            }
            mav_state = MAV_STATE_STX;
        }
        break;
    }
}

static void mavlink_dispatch(uint32_t msgid, const uint8_t *payload, uint8_t len)
{
    switch (msgid) {

    case 19: /* RET_IMU_ATTITUDE_DATA_PACKET */
        if (len >= 42) {
            g_imu.gyro_x = le_float(payload + 16);
            g_imu.gyro_y = le_float(payload + 20);
            g_imu.gyro_z = le_float(payload + 24);
            g_imu.acc_x  = le_float(payload + 28);
            g_imu.acc_y  = le_float(payload + 32);
            g_imu.acc_z  = le_float(payload + 36);
            g_imu.packet_id = le_u16(payload + 40);
            g_imu.updated = 1;
        }
        break;

    case 16: /* RET_LIDAR_DISTANCE_DATA_PACKET */
        {
            uint16_t dist_count = (len >= 6) ? ((len - 6) / 2) : 0;
            if (dist_count > LIDAR_DIST_COUNT) dist_count = LIDAR_DIST_COUNT;
            g_dist.packet_id    = le_u16(payload);
            g_dist.packet_cnt   = le_u16(payload + 2);
            g_dist.payload_size = le_u16(payload + 4);
            uint16_t i;
            for (i = 0; i < dist_count; i++) {
                g_dist.distances[i] = le_u16(payload + 6 + 2 * i);
            }
            g_dist.updated = 1;
        }
        break;

    case 17: /* RET_LIDAR_AUXILIARY_DATA_PACKET */
        if (len >= 89) {
            g_aux.com_horizontal_angle_start = le_float(payload + 20);
            g_aux.com_horizontal_angle_step  = le_float(payload + 24);
            g_aux.sys_vertical_angle_start   = le_float(payload + 28);
            g_aux.sys_vertical_angle_span    = le_float(payload + 32);
            g_aux.b_axis_dist                = le_float(payload + 72);
            g_aux.theta_angle                = le_float(payload + 76);
            g_aux.ksi_angle                  = le_float(payload + 80);
            g_aux.valid = 1;
        }
        break;

    default:
        break;
    }
}

/* ==================== Distance Compression ========================= */
void lidar_compress_distances(const uint16_t raw[120], const LidarAuxData *aux,
                              uint16_t sectors[8])
{
    uint8_t s, i;
    (void)aux;

    for (s = 0; s < 8; s++) {
        uint16_t min_val = 0xFFFF;
        for (i = 0; i < 15; i++) {
            uint16_t idx = s * 15 + i;
            if (idx < 120) {
                uint16_t d = raw[idx];
                if (d > 0 && d < min_val) min_val = d;
            }
        }
        sectors[s] = (min_val == 0xFFFF) ? 0 : min_val;
    }
}
