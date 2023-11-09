#define CANARD_DSDLC_INTERNAL
#include <com.aeronavics.OBSTACLE.h>
#include <string.h>

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t com_aeronavics_OBSTACLE_encode(struct com_aeronavics_OBSTACLE* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, COM_AERONAVICS_OBSTACLE_MAX_SIZE);
    _com_aeronavics_OBSTACLE_encode(buffer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    tao
#else
    true
#endif
    );
    return ((bit_ofs+7)/8);
}

/*
  return true if the decode is invalid
 */
bool com_aeronavics_OBSTACLE_decode(const CanardRxTransfer* transfer, struct com_aeronavics_OBSTACLE* msg) {
    uint32_t bit_ofs = 0;
    _com_aeronavics_OBSTACLE_decode(transfer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    transfer->tao
#else
    true
#endif
    );

    const uint32_t byte_len = (bit_ofs+7U)/8U;
#if CANARD_ENABLE_TAO_OPTION
    // if this could be CANFD then the dlc could indicating more bytes than
    // we actually have
    if (!transfer->tao) {
        return byte_len > transfer->payload_len;
    }
#endif
    return byte_len != transfer->payload_len;
}

#ifdef CANARD_DSDLC_TEST_BUILD
struct com_aeronavics_OBSTACLE sample_com_aeronavics_OBSTACLE_msg(void) {
    struct com_aeronavics_OBSTACLE msg;

    msg.yaw = random_float16_val();
    msg.pitch = random_float16_val();
    msg.distance = random_float16_val();
    return msg;
}
#endif
