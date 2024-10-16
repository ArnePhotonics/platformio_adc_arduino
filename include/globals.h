#ifndef GLOBALS_H_
#define GLOBALS_H_
#include <stdint.h>

//#include "../lib/rpc/include/rpc_transmission/server/app/qt2mcu.h"
#include "channel_codec/channel_codec_types.h"

#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#define ADC_VALUE_BUFFER_LENGTH 16
typedef struct {
  int16_t v[ADC_VALUE_BUFFER_LENGTH];
  uint8_t count;
} adc_values_t;

void main_set_train_length(uint16_t length);
void main_set_train_frequency(uint16_t frequency_Hz);
void main_set_aquisition_frequncy(uint16_t frequency_Hz);

adc_values_t main_get_last_values();

void delay_ms(uint32_t sleep_ms);

#ifdef __cplusplus
}
#endif

extern channel_codec_instance_t cc_instances[channel_codec_comport_COUNT];

#define TRUE 1
#define FALSE 0

#endif /*GLOBALS_H_*/
