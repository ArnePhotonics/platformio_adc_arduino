// clang-format off

#include <Arduino.h>

#include <Wire.h>
#include <MCP342x.h>

#include "main.h"
#include <LTC2633.h>


//#include "../lib/rpc/include/rpc_transmission/server/app/qt2mcu.h"

#include "channel_codec/channel_codec.h"
#include "errorlogger/generic_eeprom_errorlogger.h"
#include "globals.h"
#include "rpc_transmission/client/generated_app/RPC_TRANSMISSION_mcu2qt.h"
#include "vc.h"
// clang-format on

const uint8_t ADDRESS_MCP342xs = 0x68;
MCP342x adc = MCP342x(ADDRESS_MCP342xs);
const uint8_t CLK_OUT_PIN = 9;
const uint8_t LED_PIN = LED_BUILTIN;
LTC2633 myDAC;

typedef enum {
  PRESCALER_1 = 1,
  PRESCALER_8 = 2,
  PRESCALER_64 = 3,
  PRESCALER_256 = 4,
  PRESCALER_1024 = 5,
} prescaler_t;

typedef enum {
  state_waiting_for_start,          //
  state_pulse_train_to_be_started,  //
  state_pulse_train_started,        //
  state_pulse_train_stopped,        //
} acquisition_state_t;

static volatile acquisition_state_t acquisition_state = state_waiting_for_start;

static uint16_t prescaler_t2num(prescaler_t ps) {
  switch (ps) {
    case PRESCALER_1:
      return 1;
    case PRESCALER_8:
      return 8;
    case PRESCALER_64:
      return 64;
    case PRESCALER_256:
      return 256;
    case PRESCALER_1024:
      return 1024;
  }
  return 1;
}
const prescaler_t TIMER_PULSE_PRESCALER = PRESCALER_8;
const prescaler_t TIMER_ACQU_PRESCALER = PRESCALER_8;

#define CHANNEL_CODEC_TX_BUFFER_SIZE 64
#define CHANNEL_CODEC_RX_BUFFER_SIZE 64

channel_codec_instance_t cc_instances[channel_codec_comport_COUNT];

static char cc_rxBuffers[channel_codec_comport_COUNT]
                        [CHANNEL_CODEC_RX_BUFFER_SIZE];
static char cc_txBuffers[channel_codec_comport_COUNT]
                        [CHANNEL_CODEC_TX_BUFFER_SIZE];

uint16_t PULSE_TRAIN_LENGTH = 50;
uint16_t PULSE_TRAIN_FREQUENCY = 4000;

// static int16_t last_adc_value = 0;
// static bool is_last_adc_updated = 0;
adc_values_t adc_values;

volatile uint16_t pulse_train_progress = 0;
volatile uint8_t acquisistion_frequency_hz = 10;

static volatile uint16_t timer_acqu_overflow_counter = 0;
static volatile uint16_t timer_acqu_cmp_target = 0;
static volatile uint16_t timer_acqu_overflow_target = 0;

static void timer_pulse_stop() {
  TCCR1A &= (~((1 << COM1A1) | (1 << COM1A0)));  // release pin
  TCCR1B &= ~0x07;                               // stop_timer
}

static void timer_pulse_start() {
  TCNT1 = 0;                                // reset timer value
  TCCR1A |= (0 << COM1A1) | (1 << COM1A0);  // toggle on compare Channel A
  TCCR1B |= TIMER_PULSE_PRESCALER;
}

static void timer_pulse_set_frequency(uint16_t frequency) {
  uint32_t value = 16000000;
  value /= prescaler_t2num(TIMER_PULSE_PRESCALER);
  value /= frequency;
  value /= 2;
  value -= 1;
  OCR1A = value;
}

static void timer_pulse_init() {
  // Timer1 is used normally by servo library
  TCCR1A = (0 << WGM11) | (0 << WGM10);  // CTC mode
  TCCR1B = (0 << WGM13) | (1 << WGM12);
  TCCR1C = 0;

  // OC1B: Pin 10
  // OC1A: Pin 9

  // 0.999kHz @ 1000
  // 1.000kHz @ 999
  // 1.999kHz @ 500
  // 3.999kHz @ 250
  // 333kHz @ 2
  // 500kHz @ 1
  // 1000kHz @ 0

  TCNT1 = 0;  // reset timer value
  TIMSK1 = (1 << OCIE1A);
}

ISR(TIMER1_COMPA_vect) {
  pulse_train_progress++;
  if (pulse_train_progress >= PULSE_TRAIN_LENGTH) {
    timer_pulse_stop();
    digitalWrite(CLK_OUT_PIN, 0);
    TCNT1 = 0;  // reset timer value
    pulse_train_progress = 0;
    acquisition_state = state_pulse_train_stopped;
  }
}

static void timer_acqu_stop() {
  TCNT2 = 0;  // reset timer value
  TCCR2B = 0;
}

static void timer_acqu_start() {
  if (timer_acqu_overflow_target) {
    timer_acqu_overflow_counter = timer_acqu_overflow_target;
    TCCR2A = (0 << WGM21) | (0 << WGM20);  // Normal mode
    TCCR2B = (0 << WGM22);
    TIMSK2 = (1 << TOIE2) | (0 << OCIE2A);
  } else {
    TCCR2A = (1 << WGM21) | (0 << WGM20);  // CTC mode
    TCCR2B = (0 << WGM22);
    TIMSK2 = (0 << TOIE2) | (1 << OCIE2A);
    OCR2A = timer_acqu_cmp_target;
  }

  TCNT2 = 0;  // reset timer value
  TCCR2B |= TIMER_ACQU_PRESCALER;
}

static void timer_acqu_set_frequency(uint16_t frequency) {
  uint32_t value = 16000000;
  value /= prescaler_t2num(TIMER_ACQU_PRESCALER);
  value /= frequency;
  // value /= 2;
  value -= 1;
  timer_acqu_overflow_target = value / 256;
  timer_acqu_cmp_target = value % 256;
  // Serial.println(timer_acqu_overflow_target);
  // Serial.println(timer_acqu_cmp_target);
}

static void timer_acqu_init() {
  TCCR2A = (0 << WGM21) | (0 << WGM20);
  TCCR2B = (0 << WGM22);

  // OC1B: Pin 10
  // OC1A: Pin 9

  // 0.999kHz @ 1000
  // 1.000kHz @ 999
  // 1.999kHz @ 500
  // 3.999kHz @ 250
  // 333kHz @ 2
  // 500kHz @ 1
  // 1000kHz @ 0

  TCNT2 = 0;  // reset timer value
  TIMSK2 = (0 << TOIE2) | (0 << OCIE2A);
}

static void acqu_timer_trigger() {
  if (acquisition_state == state_waiting_for_start) {
    acquisition_state = state_pulse_train_to_be_started;
  }
  static uint8_t led_state = 0;
  if (led_state & 1) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
  led_state++;
}

ISR(TIMER2_OVF_vect) {
#if 1
  timer_acqu_overflow_counter--;  //
  if (timer_acqu_overflow_counter == 0) {
    if (timer_acqu_cmp_target) {
      OCR2A = timer_acqu_cmp_target;
      TIMSK2 = (0 << TOIE2) | (1 << OCIE2A);

      // TCCR2A &= ~((1 << WGM01) | (1 << WGM00));  //
      TCCR2A |= (1 << WGM21) | (0 << WGM20);  // CTC mode
    } else {
      acqu_timer_trigger();
      timer_acqu_overflow_counter = timer_acqu_overflow_target;
    }
  }
#else
  acqu_timer_trigger();
#endif
}

ISR(TIMER2_COMPA_vect) {
#if 1
  if (timer_acqu_overflow_target) {
    TIMSK2 = (1 << TOIE2) | (0 << OCIE2A);     // overflow mode
    TCCR2A &= ~((1 << WGM21) | (1 << WGM20));  // Normal mode

    timer_acqu_overflow_counter = timer_acqu_overflow_target;
  } else {
    TIMSK2 = (0 << TOIE2) | (1 << OCIE2A);  // ctc mode
    OCR2A = timer_acqu_cmp_target;
  }
  acqu_timer_trigger();
#endif
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

void xSerialPutChar(uint8_t data) {
  Serial.write(data);  //
}

void delay_ms(uint32_t sleep_ms) {
  delay(sleep_ms);  //
}

void main_set_train_length(uint16_t length) {
  PULSE_TRAIN_LENGTH = length * 2;  //
}

void main_set_train_frequency(uint16_t frequency_Hz) {
  PULSE_TRAIN_FREQUENCY = frequency_Hz;  //
}

void main_set_aquisition_frequncy(uint16_t frequency_Hz) {
  acquisistion_frequency_hz = frequency_Hz;
  timer_acqu_set_frequency(acquisistion_frequency_hz);
}

adc_values_t main_get_last_values() {
  adc_values_t result;
  memcpy(&result, &adc_values, sizeof(adc_values_t));
  adc_values.count = 0;
  return result;  //
}

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

  pinMode(CLK_OUT_PIN, OUTPUT);  // LED init (D13=B5)
  pinMode(LED_PIN, OUTPUT);      // LED init (D13=B5)
#if 0
  while (1) {
    digitalWrite(LED_PIN, HIGH);

    digitalWrite(LED_PIN, LOW);
  }
#endif
  sei();
  timer_acqu_init();
  timer_acqu_set_frequency(acquisistion_frequency_hz);
  timer_acqu_start();

  timer_pulse_init();
  timer_pulse_set_frequency(PULSE_TRAIN_FREQUENCY);
  timer_pulse_start();
  Serial.begin(115200);
#if 1
  RPC_TRANSMISSION_mutex_init();

  cc_instances[channel_codec_comport_transmission].aux.port =
      channel_codec_comport_transmission;

  channel_init_instance(&cc_instances[channel_codec_comport_transmission],
                        cc_rxBuffers[channel_codec_comport_transmission],
                        CHANNEL_CODEC_RX_BUFFER_SIZE,
                        cc_txBuffers[channel_codec_comport_transmission],
                        CHANNEL_CODEC_TX_BUFFER_SIZE);
#endif
#if 1
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }

  Wire.begin();
  MCP342x::generalCallReset();
  Serial.print("hello\n");

  delay(1);  // MC342x needs 300us to settle, wait 1ms

  // Check device present
  do {
    Wire.requestFrom(ADDRESS_MCP342xs, (uint8_t)1);
    if (!Wire.available()) {
      Serial.print("No device found at address ");
      Serial.println(ADDRESS_MCP342xs, HEX);

      delay(100);
    } else {
      break;
    }
  } while (1);

#endif
}

void loop() {
  while (1) {
    xSerialToRPC();

    switch (acquisition_state) {
      case state_pulse_train_to_be_started:
        acquisition_state = state_pulse_train_started;
        timer_pulse_start();
        break;
      case state_pulse_train_stopped:
        acquisition_state = state_waiting_for_start;
        long value = 0;
        MCP342x::Config status = 0;

        uint8_t err = adc.convertAndRead(MCP342x::channel2, MCP342x::oneShot,
                                         MCP342x::resolution12, MCP342x::gain1,
                                         1000000, value, status);
        adc_values.v[adc_values.count % ADC_VALUE_BUFFER_LENGTH] = value;
        if (adc_values.count < ADC_VALUE_BUFFER_LENGTH) {
          adc_values.count++;
        }

        // Serial.println(value);

        break;
    }
  }
}
