#ifndef __LIDAR_H
#define __LIDAR_H

#include "stm32f10x.h"
#include <stdint.h>

/* IMU data frame (MSGID 19: RET_IMU_ATTITUDE_DATA_PACKET) */
typedef struct {
    float    acc_x, acc_y, acc_z;
    float    gyro_x, gyro_y, gyro_z;
    uint16_t packet_id;
    uint8_t  updated;
} LidarImuData;

/* Distance data - 120 raw uint16 distances in mm */
#define LIDAR_DIST_COUNT 120
typedef struct {
    uint16_t distances[LIDAR_DIST_COUNT];
    uint16_t packet_id;
    uint16_t packet_cnt;
    uint16_t payload_size;
    uint8_t  updated;
} LidarDistData;

/* Auxiliary data (MSGID 17: RET_LIDAR_AUXILIARY_DATA_PACKET) */
typedef struct {
    float com_horizontal_angle_start;
    float com_horizontal_angle_step;
    float sys_vertical_angle_start;
    float sys_vertical_angle_span;
    float b_axis_dist;
    float theta_angle;
    float ksi_angle;
    uint8_t  valid;
} LidarAuxData;

/* Global data - updated by MAVLink parser ISR context */
extern volatile LidarImuData  g_imu;
extern volatile LidarDistData g_dist;
extern volatile LidarAuxData  g_aux;
extern volatile uint32_t      g_last_lidar_byte_ms;
extern volatile uint8_t       g_lidar_connected;

/* Ring buffer for USART1 RX (filled by ISR) */
void     lidar_ring_push(uint8_t byte);
uint8_t  lidar_ring_pop(uint8_t *byte);
uint16_t lidar_ring_available(void);

/* MAVLink parser - feed bytes one at a time from main loop */
void mavlink_feed_byte(uint8_t byte);

/* Distance compression: 120 raw -> 8 sectors */
void lidar_compress_distances(const uint16_t raw[120], const LidarAuxData *aux,
                              uint16_t sectors[8]);

#endif
