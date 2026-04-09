#ifndef DM_MOTOR_SDK_ROS_CAN_H_
#define DM_MOTOR_SDK_ROS_CAN_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t channel;
} hcan_t;

extern hcan_t hcan1;

#ifdef __cplusplus
}
#endif

#endif
