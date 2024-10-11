#include "channel_codec/phylayer.h"
#include "rpc_transmission/server/generated_general/RPC_types.h"
#include "serial.h"

RPC_RESULT phyPushDataBuffer(channel_codec_instance_t *instance, const char *buffer, size_t length) {
    (void)instance;
    for (size_t i = 0; i < length; i++) {
        xSerialPutChar(buffer[i]);
    }

    return RPC_SUCCESS;
}
