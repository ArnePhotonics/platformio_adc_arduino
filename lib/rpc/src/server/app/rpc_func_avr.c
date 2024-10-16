/*
 * rpc_func_arduino.c
 *
 *  Created on: 15.11.2015
 *      Author: arne
 */

#include <avr/io.h>

#include "../../../include/rpc_transmission/server/app/qt2mcu.h"
#include "Arduino.h"
#include "board.h"
#include "globals.h"
#include "vc.h"

device_descriptor_v1_t get_device_descriptor(void) {
  device_descriptor_v1_t descriptor = {
      .githash = GITHASH,
      .gitDate_unix = GITUNIX,
      .deviceID = 0,
      .guid = {0},
      .boardRevision = 0,
      .name = "ArduADC",
      .version = "-",
  };
  return descriptor;
}

rpc_adc_values_t get_adc_values() {
  rpc_adc_values_t result = {0};
  adc_values_t adc_values = main_get_last_values();
  result.count = adc_values.count;
  // assert(sizeof(adc_values_t) == sizeof(rpc_adc_values_t));
  memcpy(&result.v, &adc_values.v,
         ADC_VALUE_BUFFER_LENGTH * sizeof(adc_values.v[0]));

  return result;  //
}

void set_train_length(uint16_t length) {
  main_set_train_length(length);  //
}

void set_train_frequency(uint16_t frequency_Hz) {
  main_set_train_frequency(frequency_Hz);
}

void set_aquisition_frequncy(uint16_t frequency_Hz) {
  main_set_aquisition_frequncy(frequency_Hz);
}
