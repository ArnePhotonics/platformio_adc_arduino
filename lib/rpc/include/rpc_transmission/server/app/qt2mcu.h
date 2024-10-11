#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
#pragma RPC prefix RPC_TRANSMISSION_

typedef struct {
    uint16_t round_times_64us[8];
    uint8_t triggered[8];
} round_times_t;

typedef struct {
    uint32_t githash;
    uint32_t gitDate_unix;

    uint32_t serialnumber;
    uint16_t deviceID;
    uint8_t guid[12];
    uint8_t boardRevision;

    char name[12];
    char version[8];
} device_descriptor_v1_t;

typedef enum {
    d0_roundstop,
    d1_roundstop,
    d2_roundstop,
    d3_roundstop,
    d4_roundstop,
    d5_roundstop,
    d6_roundstop,
    d7_roundstop
} arduino_dig_pin_roundstop_t;
typedef enum { d2_trigger, d3_trigger } arduino_dig_pin_trigger_t;
typedef enum { edge_rising, edge_falling } arduino_dig_edge_t;

device_descriptor_v1_t get_device_descriptor(void);
void reset_all();
void reset_times_only();
void start_timer(arduino_dig_pin_trigger_t triggered_by, arduino_dig_edge_t edge);
void stop_timer();
void define_roundstop_pin(arduino_dig_pin_roundstop_t pin, arduino_dig_edge_t edge);
round_times_t get_round_times();

#ifdef __cplusplus
}
#endif /* __cplusplus */
