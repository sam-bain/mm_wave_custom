#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define COM_AERONAVICS_OBSTACLE_MAX_SIZE 6
#define COM_AERONAVICS_OBSTACLE_SIGNATURE (0x95DF056B49533A25ULL)


struct com_aeronavics_OBSTACLE {
    float yaw;
    float pitch;
    float distance;
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t com_aeronavics_OBSTACLE_encode(struct com_aeronavics_OBSTACLE* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool com_aeronavics_OBSTACLE_decode(const CanardRxTransfer* transfer, struct com_aeronavics_OBSTACLE* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _com_aeronavics_OBSTACLE_encode(uint8_t* buffer, uint32_t* bit_ofs, struct com_aeronavics_OBSTACLE* msg, bool tao);
static inline void _com_aeronavics_OBSTACLE_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct com_aeronavics_OBSTACLE* msg, bool tao);
void _com_aeronavics_OBSTACLE_encode(uint8_t* buffer, uint32_t* bit_ofs, struct com_aeronavics_OBSTACLE* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->yaw);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->pitch);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
    {
        uint16_t float16_val = canardConvertNativeFloatToFloat16(msg->distance);
        canardEncodeScalar(buffer, *bit_ofs, 16, &float16_val);
    }
    *bit_ofs += 16;
}

void _com_aeronavics_OBSTACLE_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct com_aeronavics_OBSTACLE* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->yaw = canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->pitch = canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

    {
        uint16_t float16_val;
        canardDecodeScalar(transfer, *bit_ofs, 16, true, &float16_val);
        msg->distance = canardConvertFloat16ToNativeFloat(float16_val);
    }
    *bit_ofs += 16;

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct com_aeronavics_OBSTACLE sample_com_aeronavics_OBSTACLE_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
#endif
#endif
