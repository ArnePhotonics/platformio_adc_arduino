/*
 * rpc_func_arduino.c
 *
 *  Created on: 15.11.2015
 *      Author: arne
 */

#include "../../../include/rpc_transmission/server/app/qt2mcu.h"
#include "Arduino.h"
#include "board.h"
#include "globals.h"
#include "vc.h"
#include <avr/io.h>

static volatile bool is_triggered_;
static uint8_t trigger_pin_map = 0;
static uint8_t round_time_pin_map = 0;

ISR(TIMER1_OVF_vect) {
    timer_has_been_overflowed = true;
#if 0
    static bool led_state=false;
    if (led_state){
         digitalWrite(LEDPIN, 0);
    }else{
         digitalWrite(LEDPIN, 1);
    }
    led_state  = !led_state;
#endif
}

void isr_trigger_edge() {
    if ((is_triggered_ == false)) {
        TCNT1 = 0;
        TCCR1B = (1 << CS10) | (1 << CS12); // 1024 prescaler --> 64us/tick

        old_input_pins = PIND;
        PCMSK2 = active_pins_for_roundtime_map; // activate pinchange interrupt
        PCIFR = (1 << PCIE2);                   // Clear Interrupt flag for Pinchange 2
        PCICR = (1 << PCIE2);                   // Pin change interrrupt for PortD

        is_triggered_ = true;
        digitalWrite(LEDPIN, 1); // set led
    }
    EIMSK = 0;
}

ISR(INT0_vect) {
    isr_trigger_edge();
}

ISR(INT1_vect) {
    isr_trigger_edge();
}

void timer_init() {
    // Timer1 is used normally by servo library
    TCCR1A = 0;
    TCCR1B = 0; // Timer1 is a 16bit timer
    TCCR1C = 0;
    TCNT1 = 0; // reset timer value
    TIMSK1 = (1 << TOIE1);
    timer_has_been_overflowed = false;
    TCCR1B = 0; // stop_timer
}

round_times_t get_round_times(void) {
    round_times_t result;
	uint32_t t;
    for (uint8_t i = 0; i < 8; i++) {
		t = round_times[i];
		t *= 4096;  // for some reason there is a factor 4.096 missing somewhere
		t /= 1000;
        result.round_times_64us[i] = t;
        result.triggered[i] = is_round_time_measured[i];
    }
    return result;
}

void reset_all() {
    stop_timer();
    for (uint8_t i = 0; i < 8; i++) {
        round_times[i] = 0;
        is_round_time_measured[i] = 0;
    }
    trigger_pin_map = 0;
    round_time_pin_map = 0;
    active_pins_for_roundtime_map = 0;
    edge_mask_falling = 0;
}

void stop_timer() {
    EIMSK = 0;
    timer_init();
    PCICR = 0;  // disable pinchange interrupts
    TCCR1B = 0; // stop_timer
    active_pins_for_roundtime_map = 0;
    old_input_pins = 0;
    is_triggered_ = false;
}

void start_timer(arduino_dig_pin_trigger_t triggered_by, arduino_dig_edge_t edge) {
    timer_init();

    timer_has_been_overflowed = false;
    digitalWrite(LEDPIN, 0); // set led
    uint8_t mode = 0;
    if (edge == edge_rising) {
        mode = (1 << ISC01) | (1 << ISC00); // rising edge
    } else {
        mode = (1 << ISC01) | (0 << ISC00); // falling edge
    }
    uint8_t pin_mask = 0;
    bool ok = false;
    if (triggered_by == d2_trigger) {
        ok = true;
        pin_mask = 1 << 2;
        EICRA = mode << 0;
        EIFR = 0xFF;
        EIMSK = (1 << INT0);
    } else if (triggered_by == d3_trigger) {
        ok = true;
        pin_mask = 1 << 3;
        EICRA = mode << 2;
        EIFR = 0xFF;
        EIMSK = (1 << INT1);
    }
    if (ok) {
        // PD2 = INT0
        // PD3 = INT1

        active_pins_for_roundtime_map = round_time_pin_map;
        active_pins_for_roundtime_map &= ~pin_mask;
        // initialize pin states to current value
        is_triggered_ = false;

        PCICR = 0; // disable Pin change interrrupt for PortD
        sei();
    }
}

void define_roundstop_pin(arduino_dig_pin_roundstop_t pin, arduino_dig_edge_t edge) {
    uint8_t pin_mask = 1 << pin;
    round_time_pin_map |= pin_mask;
    if (edge == edge_falling) {
        edge_mask_falling |= pin_mask;
    } else {
        edge_mask_falling &= ~pin_mask;
    }
}

void reset_times_only() {
    stop_timer();
    for (uint8_t i = 0; i < 8; i++) {
        round_times[i] = 0;
        is_round_time_measured[i] = 0;
    }
}

device_descriptor_v1_t get_device_descriptor(void) {
    device_descriptor_v1_t descriptor = {
        .githash = GITHASH,
        .gitDate_unix = GITUNIX,
        .deviceID = 0,
        .guid = {0},
        .boardRevision = 0,
        .name = "ArduWatch",
        .version = "-",
    };
    return descriptor;
}
