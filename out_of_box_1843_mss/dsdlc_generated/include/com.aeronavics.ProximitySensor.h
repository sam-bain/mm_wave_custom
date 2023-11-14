#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>
#include <com.aeronavics.OBSTACLE.h>

#define COM_AERONAVICS_PROXIMITYSENSOR_MAX_SIZE 187
#define COM_AERONAVICS_PROXIMITYSENSOR_SIGNATURE (0x41528FF10CD13209ULL)
#define COM_AERONAVICS_PROXIMITYSENSOR_ID 20802

#define COM_AERONAVICS_PROXIMITYSENSOR_PROXIMITY_SENSOR_ID_UNDEFINED    0
#define COM_AERONAVICS_PROXIMITYSENSOR_PROXIMITY_SENSOR_ID_FRONT_RIGHT  1
#define COM_AERONAVICS_PROXIMITYSENSOR_PROXIMITY_SENSOR_ID_REAR_LEFT    2
#define COM_AERONAVICS_PROXIMITYSENSOR_PROXIMITY_SENSOR_ID_FRONT_LEFT   3
#define COM_AERONAVICS_PROXIMITYSENSOR_PROXIMITY_SENSOR_ID_REAR_RIGHT   4


#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class com_aeronavics_ProximitySensor_cxx_iface;
#endif

struct com_aeronavics_ProximitySensor {
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = com_aeronavics_ProximitySensor_cxx_iface;
#endif
    uint8_t sensor_id;
    struct { uint8_t len; struct com_aeronavics_OBSTACLE data[31]; }obstacles;
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t com_aeronavics_ProximitySensor_encode(struct com_aeronavics_ProximitySensor* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool com_aeronavics_ProximitySensor_decode(const CanardRxTransfer* transfer, struct com_aeronavics_ProximitySensor* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _com_aeronavics_ProximitySensor_encode(uint8_t* buffer, uint32_t* bit_ofs, struct com_aeronavics_ProximitySensor* msg, bool tao);
static inline void _com_aeronavics_ProximitySensor_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct com_aeronavics_ProximitySensor* msg, bool tao);
void _com_aeronavics_ProximitySensor_encode(uint8_t* buffer, uint32_t* bit_ofs, struct com_aeronavics_ProximitySensor* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 3, &msg->sensor_id);
    *bit_ofs += 3;
    if (!tao) {
        canardEncodeScalar(buffer, *bit_ofs, 5, &msg->obstacles.len);
        *bit_ofs += 5;
    }
    size_t i;
    for (i=0; i < msg->obstacles.len; i++) {
        _com_aeronavics_OBSTACLE_encode(buffer, bit_ofs, &msg->obstacles.data[i], false);
    }
}

void _com_aeronavics_ProximitySensor_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct com_aeronavics_ProximitySensor* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardDecodeScalar(transfer, *bit_ofs, 3, false, &msg->sensor_id);
    *bit_ofs += 3;

    if (!tao) {
        canardDecodeScalar(transfer, *bit_ofs, 5, false, &msg->obstacles.len);
        *bit_ofs += 5;
    }


    if (tao) {
        msg->obstacles.len = 0;
        while ((transfer->payload_len*8) > *bit_ofs) {
            _com_aeronavics_OBSTACLE_decode(transfer, bit_ofs, &msg->obstacles.data[msg->obstacles.len], false);
            msg->obstacles.len++;
        }
    } else {
        size_t i;
        for (i=0; i < msg->obstacles.len; i++) {
            _com_aeronavics_OBSTACLE_decode(transfer, bit_ofs, &msg->obstacles.data[i], false);
        }
    }

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct com_aeronavics_ProximitySensor sample_com_aeronavics_ProximitySensor_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
BROADCAST_MESSAGE_CXX_IFACE(com_aeronavics_ProximitySensor, COM_AERONAVICS_PROXIMITYSENSOR_ID, COM_AERONAVICS_PROXIMITYSENSOR_SIGNATURE, COM_AERONAVICS_PROXIMITYSENSOR_MAX_SIZE);
#endif
#endif
