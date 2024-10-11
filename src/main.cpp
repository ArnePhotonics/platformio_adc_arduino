// clang-format off

#include <Arduino.h>



#include "main.h"



#include "channel_codec/channel_codec.h"
#include "errorlogger/generic_eeprom_errorlogger.h"
#include "globals.h"
#include "rpc_transmission/client/generated_app/RPC_TRANSMISSION_mcu2qt.h"
#include "vc.h"
// clang-format on

#define CHANNEL_CODEC_TX_BUFFER_SIZE 64
#define CHANNEL_CODEC_RX_BUFFER_SIZE 64

channel_codec_instance_t cc_instances[channel_codec_comport_COUNT];

static char cc_rxBuffers[channel_codec_comport_COUNT]
                        [CHANNEL_CODEC_RX_BUFFER_SIZE];
static char cc_txBuffers[channel_codec_comport_COUNT]
                        [CHANNEL_CODEC_TX_BUFFER_SIZE];

volatile uint16_t round_times[INPUT_PINS_COUNT_FOR_ROUND_TIME] = {0};
volatile uint8_t is_round_time_measured[INPUT_PINS_COUNT_FOR_ROUND_TIME] = {0};

volatile uint8_t old_input_pins = 0;
volatile uint8_t edge_mask_falling = 0;

volatile uint8_t active_pins_for_roundtime_map = 0;

volatile bool timer_has_been_overflowed = false;

ISR(PCINT2_vect) {
  // PD0: PCINT16
  // PD1: PCINT17
  // PD2: PCINT18
  // PD3: PCINT19
  // PD4: PCINT20
  // PD5: PCINT21
  // PD6: PCINT22
  // PD7: PCINT23
  uint8_t input_pins = PIND;

  for (uint8_t i = 0; i < INPUT_PINS_COUNT_FOR_ROUND_TIME; i++) {
    uint8_t mask = 1 << i;
    if ((active_pins_for_roundtime_map & mask) == 0) {
      continue;
    }
    if (is_round_time_measured[i] == false) {
      if (edge_mask_falling & mask) {
        // if rising edge for specific pin
        if (((old_input_pins & mask) == 0) && ((input_pins & mask) > 0)) {
          // rising edge detected
          if (timer_has_been_overflowed == false) {
            round_times[i] = TCNT1;
          } else {
            round_times[i] = 0xFFFF;
          }
          is_round_time_measured[i] = true;
          digitalWrite(LEDPIN, 0);  // set led
        }
      } else {
        if (((old_input_pins & mask) > 0) && ((input_pins & mask) == 0)) {
          // falling edge detected
          if (timer_has_been_overflowed == false) {
            round_times[i] = TCNT1;
          } else {
            round_times[i] = 0xFFFF;
          }
          is_round_time_measured[i] = true;
          digitalWrite(LEDPIN, 0);  // set led
        }
      }
    }
  }

  old_input_pins = input_pins;
}

bool xSerialCharAvailable() {
  if (Serial.available()) {
    return true;
  } else {
    return false;
  }
}

bool xSerialGetChar(char *data) {
  if (Serial.available()) {
    *data = Serial.read();
    return true;
  } else {
    return false;
  }
}

#ifdef __cplusplus
extern "C" {
#endif

void xSerialToRPC(void) {
  while (xSerialCharAvailable()) {
    // read the incoming byte:
    char inByte = 0;
    xSerialGetChar(&inByte);

    channel_push_byte_to_RPC(&cc_instances[channel_codec_comport_transmission],
                             inByte);
  }
}

void xSerialPutChar(uint8_t data) { Serial.write(data); }

void delay_ms(uint32_t sleep_ms) { delay(sleep_ms); }

#ifdef __cplusplus
}
#endif

void ChannelCodec_errorHandler(channel_codec_instance_t *instance,
                               channelCodecErrorNum_t ErrNum) {
  (void)ErrNum;
  (void)instance;
}

void setup() {
  DDRD = 0x00;          // set PORTD to input (D0-D7)
  PORTD = 0xFF;         // enable pullups
  MCUCR |= (1 << PUD);  // enable pullups

  pinMode(LEDPIN, OUTPUT);  // LED init (D13=B5)
  digitalWrite(LEDPIN, 0);  // write inversed state back

  Serial.begin(115200);
  RPC_TRANSMISSION_mutex_init();

  cc_instances[channel_codec_comport_transmission].aux.port =
      channel_codec_comport_transmission;

  channel_init_instance(&cc_instances[channel_codec_comport_transmission],
                        cc_rxBuffers[channel_codec_comport_transmission],
                        CHANNEL_CODEC_RX_BUFFER_SIZE,
                        cc_txBuffers[channel_codec_comport_transmission],
                        CHANNEL_CODEC_TX_BUFFER_SIZE);

#if 1
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }
#endif
}

void loop() {
  while (1) {
    xSerialToRPC();
  }
}
