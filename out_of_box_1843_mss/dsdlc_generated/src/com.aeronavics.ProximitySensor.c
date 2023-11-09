#define CANARD_DSDLC_INTERNAL
#include <com.aeronavics.ProximitySensor.h>
#include <string.h>

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t com_aeronavics_ProximitySensor_encode(struct com_aeronavics_ProximitySensor* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, COM_AERONAVICS_PROXIMITYSENSOR_MAX_SIZE);
    _com_aeronavics_ProximitySensor_encode(buffer, &bit_ofs, msg, 
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
bool com_aeronavics_ProximitySensor_decode(const CanardRxTransfer* transfer, struct com_aeronavics_ProximitySensor* msg) {
    uint32_t bit_ofs = 0;
    _com_aeronavics_ProximitySensor_decode(transfer, &bit_ofs, msg, 
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
struct com_aeronavics_ProximitySensor sample_com_aeronavics_ProximitySensor_msg(void) {
    struct com_aeronavics_ProximitySensor msg;

    msg.sensor_id = (uint8_t)random_bitlen_unsigned_val(3);
    msg.obstacles.len = (uint8_t)random_range_unsigned_val(0, 31);
    for (size_t i=0; i < msg.obstacles.len; i++) {
        msg.obstacles.data[i] = sample_com_aeronavics_OBSTACLE_msg();
    }
    return msg;
}
#endif
