/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017 Scott Shawcroft for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "common-hal/microcontroller/Pin.h"
#include "shared-bindings/bitbangio/OneWire.h"
#include "shared-bindings/microcontroller/__init__.h"
#include "shared-bindings/digitalio/DigitalInOut.h"
#include "shared-module/bitbangio/types.h"

// Durations are taken from here: https://www.maximintegrated.com/en/app-notes/index.mvp/id/126

void shared_module_bitbangio_onewire_construct(bitbangio_onewire_obj_t* self,
        const mcu_pin_obj_t* pin) {
    self->pin.base.type = &digitalio_digitalinout_type;
    self->mode = IO_MODE_ONEWIRE;
    common_hal_digitalio_digitalinout_construct(&self->pin, pin);
}

bool shared_module_bitbangio_onewire_deinited(bitbangio_onewire_obj_t* self) {
    return common_hal_digitalio_digitalinout_deinited(&self->pin);
}

void shared_module_bitbangio_onewire_deinit(bitbangio_onewire_obj_t* self) {
    if (shared_module_bitbangio_onewire_deinited(self)) {
        return;
    }
    common_hal_digitalio_digitalinout_deinit(&self->pin);
}

void shared_module_bitbangio_onewire_mode(bitbangio_onewire_obj_t* self, uint8_t mode) {
    self->mode = mode;
}

// We use common_hal_mcu_delay_us(). It should not be dependent on interrupts
// to do accurate timekeeping, since we disable interrupts during the delays below.

bool shared_module_bitbangio_onewire_reset(bitbangio_onewire_obj_t* self) {
    bool value = false;
    common_hal_mcu_disable_interrupts();
    common_hal_digitalio_digitalinout_switch_to_output(&self->pin, false, DRIVE_MODE_OPEN_DRAIN);
    common_hal_mcu_delay_us(480);

    if (self->mode == IO_MODE_ONEWIRE) {
        common_hal_digitalio_digitalinout_switch_to_input(&self->pin, PULL_NONE);
        common_hal_mcu_delay_us(70);
        value = common_hal_digitalio_digitalinout_get_value(&self->pin);
        common_hal_mcu_delay_us(410);

    } else if (self->mode == IO_MODE_SINGLEWIRE) {

    } else if (self->mode == IO_MODE_SIO) {
        // From http://ww1.microchip.com/downloads/en/AppNotes/Atmel-8976-SEEPROM-AT21CS-Resest-Discovery_ApplicationNote.pdf

        // Event 3
        // SI/O transitions to the high state after the Master releases SI/O. After the time
        // specified by tRRT (Reset Recovery Time), the Master drives SI/O low to request
        // a Discovery Response Acknowledge from the slave device.
        //
        // tRRT range : 8 us to ___
        common_hal_digitalio_digitalinout_set_value(&self->pin, true);
        common_hal_mcu_delay_us(10);

        // Event 4
        // The Master continues holding SI/O low for during the entire tDRR (Discovery Response Request) interval.
        //
        // tDRR range : 1 us to 2 - tPUP
        // tPUP is the time required once the SI/O line is released to be pulled up from VIL to VIH.
        // This value is application specific and is a function of the loading capacitance on the SI/O line as well as the RPUP chosen
        common_hal_digitalio_digitalinout_set_value(&self->pin, false);
        common_hal_mcu_delay_us(2);

        // Measured delay (from start of low pulse to end of low pulse) with no device on bus
        // 4.5us when specified delay is 2us.  Likely caused by processing within the switch_to_input function.

        // Event 5
        // The slave device detects SI/O low within the tDRR interval and concurrently
        // holds SI/O low for tDACK. The slave device releases SI/O after tDACK.
        //
        // tDACK spec is 8 to 24 us, and includes the tDDR time above.
        // But AT21CS releases the line after a total pulse width of 13.5us (in PDF and measured during development).
        // Delay here must be short enough (with additional delays in the get_value function) to see that low-period.
        common_hal_digitalio_digitalinout_switch_to_input(&self->pin, PULL_NONE);
        common_hal_mcu_delay_us(2);
        value = common_hal_digitalio_digitalinout_get_value(&self->pin);
    }

    common_hal_mcu_enable_interrupts();
    return value;
}

STATIC bool sio_read_bit(bitbangio_onewire_obj_t* self) {

    // Emit a pulse of 1.3 us
    common_hal_digitalio_digitalinout_set_value(&self->pin, false);
    common_hal_digitalio_digitalinout_set_value(&self->pin, true);

    // Immediately sample bus to see if slave has extended the low pulse
    bool value = common_hal_digitalio_digitalinout_get_value(&self->pin);

    common_hal_mcu_delay_us(10);
    return value;
}

STATIC void sio_write_bit(bitbangio_onewire_obj_t* self, bool bit) {
    // --- Standard Speed (not the default) ---
    // tbit should be 40 to 100 us
    //                                                  delay => result     delay => result
    // logic 1 should be low  4us to  8us, then high.   3/32  => 6/48       3/37  => 6/56
    // logic 0 should be low 24us to 64us, then high.   12/14 => 19/22      20/20 => 30/30

    // --- High Speed (the default) ---
    // tbit should be __ to 25 us
    //
    // logic 1 should be low  1us to  2us, then high.   Code here produces 1.4/19.8 (tbit of 21.1us) us on M0
    // logic 0 should be low  6us to 16us, then high.   Code here produces 9.1/12.5 (tbit of 21.6us) on M0
    common_hal_digitalio_digitalinout_set_value(&self->pin, false);

    if (!bit) {
        common_hal_mcu_delay_us(5);
    }

    common_hal_digitalio_digitalinout_set_value(&self->pin, true);
    common_hal_mcu_delay_us(bit? 12 : 7);
}

STATIC void sio_start_stop(bitbangio_onewire_obj_t* self) {
    common_hal_digitalio_digitalinout_switch_to_output(&self->pin, true, DRIVE_MODE_OPEN_DRAIN);
    common_hal_mcu_delay_us(150);
}

bool shared_module_bitbangio_onewire_read_bit(bitbangio_onewire_obj_t* self) {
    bool value = false;
    common_hal_mcu_disable_interrupts();

    if (self->mode == IO_MODE_ONEWIRE) {
        common_hal_digitalio_digitalinout_switch_to_output(&self->pin, false, DRIVE_MODE_OPEN_DRAIN);
        common_hal_mcu_delay_us(6);
        common_hal_digitalio_digitalinout_switch_to_input(&self->pin, PULL_NONE);
        // TODO(tannewt): Test with more devices and maybe make the delays
        // configurable. This should be 9 by the datasheet but all bits read as 1
        // then.
        common_hal_mcu_delay_us(6);
        value = common_hal_digitalio_digitalinout_get_value(&self->pin);
        common_hal_mcu_delay_us(55);

    } else if (self->mode == IO_MODE_SINGLEWIRE) {

    } else if (self->mode == IO_MODE_SIO) {
        value = sio_read_bit(self);
    }

    common_hal_mcu_enable_interrupts();
    return value;
}

void shared_module_bitbangio_onewire_write_bit(bitbangio_onewire_obj_t* self, bool bit) {
    common_hal_mcu_disable_interrupts();

    if (self->mode == IO_MODE_ONEWIRE) {
        // tlow_1 : delay of  6us measured at 14us
        // tlow_0 : delay of 60us measured at 92us
        // tbit (tlow + thigh) : 340us
        common_hal_digitalio_digitalinout_switch_to_output(&self->pin, false, DRIVE_MODE_OPEN_DRAIN);
        common_hal_mcu_delay_us(bit? 6 : 60);
        common_hal_digitalio_digitalinout_switch_to_input(&self->pin, PULL_NONE);
        common_hal_mcu_delay_us(bit? 64 : 10);

    } else if (self->mode == IO_MODE_SINGLEWIRE) {

    } else if (self->mode == IO_MODE_SIO) {
        sio_write_bit(self, bit);
    }

    common_hal_mcu_enable_interrupts();
}

bool shared_module_bitbangio_onewire_read_byte(bitbangio_onewire_obj_t* self, uint8_t *val) {

    if (self->mode == IO_MODE_ONEWIRE) {

    } else if (self->mode == IO_MODE_SINGLEWIRE) {

    } else if (self->mode == IO_MODE_SIO) {
        // Atmel SIO is MSB first
        uint8_t data = 0;
        for (int i = 7; i >= 0; i--) {
            data = (data << 1) | sio_read_bit(self);
        }
        *val = data;
    }

    return true;
}

bool shared_module_bitbangio_onewire_write_byte(bitbangio_onewire_obj_t* self, uint8_t value) {
    bool status = false;
    common_hal_mcu_disable_interrupts();

    if (self->mode == IO_MODE_ONEWIRE) {

    } else if (self->mode == IO_MODE_SINGLEWIRE) {

    } else if (self->mode == IO_MODE_SIO) {

        common_hal_digitalio_digitalinout_switch_to_output(&self->pin, true, DRIVE_MODE_OPEN_DRAIN);

        // Atmel SIO is MSB first
        for (int i = 7; i >= 0; i--) {
            sio_write_bit(self, (value >> i) & 1);
        }

        // Look for ACK (logic 0) / NACK (logic 1)
        status = ! sio_read_bit(self);
    }

    common_hal_mcu_enable_interrupts();
    return status;
}

uint8_t shared_module_bitbangio_onewire_read(bitbangio_onewire_obj_t* self,
    uint8_t device_addr, uint8_t * data, size_t len) {

    uint8_t status = 0;
    common_hal_mcu_disable_interrupts();

    if (self->mode == IO_MODE_ONEWIRE) {

    } else if (self->mode == IO_MODE_SINGLEWIRE) {

    } else if (self->mode == IO_MODE_SIO) {
        sio_start_stop(self);
        bool ack = shared_module_bitbangio_onewire_write_byte(self, device_addr);

        if (ack) {
            for (uint32_t i = 0; i < len; i++) {
                shared_module_bitbangio_onewire_read_byte(self, data + i);

                if (i < len - 1) {
                    sio_write_bit(self, false); // Send ACK to signal that more bytes are requested
                } else {
                    sio_write_bit(self, true); // Send NACK to signal end of read
                }
            }
        }

        sio_start_stop(self);
    }

    common_hal_mcu_enable_interrupts();
    return status;
}

uint8_t shared_module_bitbangio_onewire_write(bitbangio_onewire_obj_t *self, uint8_t device_addr,
    uint8_t memory_addr, uint8_t *data, size_t len) {

    uint8_t status = 0;
    common_hal_mcu_disable_interrupts();

    if (self->mode == IO_MODE_ONEWIRE) {

    } else if (self->mode == IO_MODE_SINGLEWIRE) {

    } else if (self->mode == IO_MODE_SIO) {
        sio_start_stop(self);
        bool ack = shared_module_bitbangio_onewire_write_byte(self, device_addr);

        if (ack) {
            ack = shared_module_bitbangio_onewire_write_byte(self, memory_addr);
        }

        if (ack) {
            for (uint32_t i = 0; i < len; i++) {
                if (!shared_module_bitbangio_onewire_write_byte(self, data[i])) {
                    status = 1;
                    break;
                }
            }
        }

        sio_start_stop(self);
    }

    common_hal_mcu_enable_interrupts();
    return status;
}
