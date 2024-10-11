/*
 * rpc_service_protothreads.c
 *
 *  Created on: 15.11.2015
 *      Author: arne
 */

#include <assert.h>
#include "../../../include/rpc_transmission/server/generated_general/RPC_types.h"
#include "../../../include/rpc_transmission/server/generated_general/RPC_TRANSMISSION_network.h"
#include "serial.h"
#include <stdint.h>
#include "channel_codec/channel_codec.h"
#include "globals.h"

typedef struct {
    uint8_t locked;
    // int8_t recursion_depth;
} singleThreadRPCMutex_t;

static volatile singleThreadRPCMutex_t RPC_TRANSMISSION_mutex[RPC_MUTEX_COUNT];

const uint32_t RPC_MUTEX_TIMEOUT_MAX = UINT32_MAX;
const uint32_t RPC_MUTEX_TIMEOUT_ms = 500UL;
// 500
/* Initializes all rpc mutexes. */
void RPC_TRANSMISSION_mutex_init(void) {
    for (int i = 0; i < RPC_MUTEX_COUNT; i++) {
        RPC_TRANSMISSION_mutex[i].locked = 0;
        // RPC_TRANSMISSION_mutex[i].recursion_depth = 0;
    }
}

/* Unlocks the mutex. The mutex is locked when the function is called. */
void RPC_TRANSMISSION_mutex_unlock(RPC_mutex_id mutex_id) {
    RPC_TRANSMISSION_mutex[mutex_id].locked = 0;
}

static char RPC_TRANSMISSION_mutex_lock_timeout_raw(const unsigned long timeout, RPC_mutex_id mutex_id) {
    unsigned long timer = 0;
    bool timeout_happened = true;
    bool ignore = false;
    if (mutex_id ==
        RPC_mutex_caller) { // in RPC is used to prevent sending a request while another request is in progress. Not possible in Single thread system
        ignore = true;
    }
    if (mutex_id ==
        RPC_mutex_in_caller) { // in RPC is used to prevent that the answer is parsed before the request. Not possible in Single thread system
        ignore = true;
    }
    if (mutex_id == RPC_mutex_parsing_complete) {
        ignore = true;
    }

    if (ignore) {
        return 1;
    }
    // RPC_TRANSMISSION_mutex[mutex_id].recursion_depth++;
    while (timeout_happened) {
        timer++;
        if ((timer >= timeout) && (timeout != RPC_MUTEX_TIMEOUT_MAX)) {
            break;
        }

        if (RPC_TRANSMISSION_mutex[mutex_id].locked == 1) {
            // if (RPC_TRANSMISSION_mutex[mutex_id].recursion_depth == 1)
            { xSerialToRPC(); }
            delay_ms(1);

        } else {
            timeout_happened = false;
        }
    }
    // RPC_TRANSMISSION_mutex[mutex_id].recursion_depth--;

    if (timeout_happened == false) {
        RPC_TRANSMISSION_mutex[mutex_id].locked = 1;
        return 1;
    } else {
        return 0;
    }
}

/* Locks the mutex. If it is already locked it yields until it can lock the mutex. */
void RPC_TRANSMISSION_mutex_lock(RPC_mutex_id mutex_id) {
    RPC_TRANSMISSION_mutex_lock_timeout_raw(RPC_MUTEX_TIMEOUT_MAX, mutex_id);
}

/* Tries to lock a mutex. Returns 1 if the mutex was locked and 0 if a timeout
   occured. The timeout length should be the time you want to wait for an answer
   before giving up. If the time is infinite a lost answer will get the calling
   thread stuck indefinitely. */
char RPC_TRANSMISSION_mutex_lock_timeout(RPC_mutex_id mutex_id) {
    return RPC_TRANSMISSION_mutex_lock_timeout_raw(RPC_MUTEX_TIMEOUT_ms, mutex_id);
}

/*  This function is called when a new message starts. {size} is the number of
                    bytes the message will require. In the implementation you can allocate  a
                    buffer or write a preamble. The implementation can be empty if you do not
                    need to do that. */
void RPC_TRANSMISSION_message_start(size_t size) {
    channel_start_message_from_RPC(&cc_instances[channel_codec_comport_transmission], size);
}

/* Pushes a byte to be sent via network. You should put all the pushed bytes
                   into a buffer and send the buffer when RPC_TRANSMISSION_message_commit is called. If you run
                   out of buffer space you can send multiple partial messages as long as the
                   other side puts them back together. */
void RPC_TRANSMISSION_message_push_byte(unsigned char byte) {
    channel_push_byte_from_RPC(&cc_instances[channel_codec_comport_transmission], byte);
}

/* This function is called when a complete message has been pushed using
                   RPC_TRANSMISSION_message_push_byte. Now is a good time to send the buffer over the network,
                   even if the buffer is not full yet. You may also want to free the buffer that
                   you may have allocated in the RPC_TRANSMISSION_message_start function.
                   RPC_TRANSMISSION_message_commit should return RPC_TRANSMISSION_SUCCESS if the buffer has been successfully
                   sent and RPC_TRANSMISSION_FAILURE otherwise. */
RPC_RESULT RPC_TRANSMISSION_message_commit(void) {
    return channel_commit_from_RPC(&cc_instances[channel_codec_comport_transmission]);
}

RPC_SIZE_RESULT RPC_CHANNEL_CODEC_get_request_size(channel_codec_instance_t *instance, const void *buffer, size_t size_bytes) {
    if (instance->aux.port == channel_codec_comport_transmission) {
        return RPC_TRANSMISSION_get_request_size(buffer, size_bytes);
    } else {
        RPC_SIZE_RESULT result;
        result.size = 0;
        result.result = RPC_FAILURE;
        assert(0);
        return result;
    }
}

RPC_SIZE_RESULT RPC_CHANNEL_CODEC_get_answer_length(channel_codec_instance_t *instance, const void *buffer, size_t size_bytes) {
    if (instance->aux.port == channel_codec_comport_transmission) {
        return RPC_TRANSMISSION_get_answer_length(buffer, size_bytes);
    } else {
        RPC_SIZE_RESULT result;
        result.size = 0;
        result.result = RPC_FAILURE;
        assert(0);
        return result;
    }
}

void RPC_CHANNEL_CODEC_parse_request(channel_codec_instance_t *instance, const void *buffer, size_t size_bytes) {
    if (instance->aux.port == channel_codec_comport_transmission) {
        RPC_TRANSMISSION_parse_request(buffer, size_bytes);
    } else {
        assert(0);
    }
}

void RPC_CHANNEL_CODEC_parse_answer(channel_codec_instance_t *instance, const void *buffer, size_t size_bytes) {
    if (instance->aux.port == channel_codec_comport_transmission) {
        RPC_TRANSMISSION_parse_answer(buffer, size_bytes);
    } else {
        assert(0);
    }
}

void RPC_CHANNEL_CODEC_parser_init(channel_codec_instance_t *instance) {
    if (instance->aux.port == channel_codec_comport_transmission) {
        RPC_TRANSMISSION_Parser_init();
    } else {
        assert(0);
    }
}
