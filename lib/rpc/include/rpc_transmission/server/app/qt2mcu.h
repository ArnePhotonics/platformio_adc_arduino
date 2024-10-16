#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
#pragma RPC prefix RPC_TRANSMISSION_

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

device_descriptor_v1_t get_device_descriptor(void);

typedef struct {
  int16_t v[16];
  uint8_t count;
} rpc_adc_values_t;

rpc_adc_values_t get_adc_values();

void set_train_length(uint16_t length);
void set_train_frequency(uint16_t frequency_Hz);
void set_aquisition_frequncy(uint16_t frequency_Hz);

#ifdef __cplusplus
}
#endif /* __cplusplus */
