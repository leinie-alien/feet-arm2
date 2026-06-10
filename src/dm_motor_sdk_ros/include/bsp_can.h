#ifndef DM_MOTOR_SDK_ROS_BSP_CAN_H_
#define DM_MOTOR_SDK_ROS_BSP_CAN_H_

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "can.h"

#ifdef __cplusplus
extern "C" {
#endif

bool canx_open(hcan_t* hcan, uint8_t channel, int can_baud, int canfd_baud);
void canx_close(hcan_t* hcan);

void canx_send_data(hcan_t* hcan, uint16_t can_id, const uint8_t* data, uint8_t len);
void canx_receive(hcan_t* hcan, uint16_t* can_id, uint8_t* data);

size_t canx_pending(const hcan_t* hcan);

#ifdef __cplusplus
}
#endif

#endif
