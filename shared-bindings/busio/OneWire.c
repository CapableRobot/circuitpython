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

#include <stdint.h>

#include "lib/utils/buffer_helper.h"
#include "lib/utils/context_manager_helpers.h"
#include "py/objproperty.h"
#include "py/runtime.h"
#include "py/runtime0.h"
#include "shared-bindings/microcontroller/Pin.h"
#include "shared-bindings/busio/OneWire.h"
#include "shared-bindings/util.h"

//| .. currentmodule:: busio
//|
//| :class:`OneWire` -- Lowest-level of the Maxim OneWire protocol
//| =================================================================
//|
//| :class:`~busio.OneWire` implements the timing-sensitive foundation of the Maxim
//| (formerly Dallas Semi) OneWire protocol.
//|
//| Protocol definition is here: https://www.maximintegrated.com/en/app-notes/index.mvp/id/126
//|
//| .. class:: OneWire(pin)
//|
//|   Create a OneWire object associated with the given pin. The object
//|   implements the lowest level timing-sensitive bits of the protocol.
//|
//|   :param ~microcontroller.Pin pin: Pin connected to the OneWire bus
//|
//|   Read a short series of pulses::
//|
//|     import busio
//|     import board
//|
//|     onewire = busio.OneWire(board.D7)
//|     onewire.reset()
//|     onewire.write_bit(True)
//|     onewire.write_bit(False)
//|     print(onewire.read_bit())
//|
STATIC mp_obj_t busio_onewire_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *pos_args) {
    mp_arg_check_num(n_args, n_kw, 1, MP_OBJ_FUN_ARGS_MAX, true);
    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw, pos_args + n_args);
    enum { ARG_pin };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_pin, MP_ARG_REQUIRED | MP_ARG_OBJ },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, &kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
    assert_pin(args[ARG_pin].u_obj, false);
    const mcu_pin_obj_t* pin = MP_OBJ_TO_PTR(args[ARG_pin].u_obj);
    assert_pin_free(pin);

    busio_onewire_obj_t *self = m_new_obj(busio_onewire_obj_t);
    self->base.type = &busio_onewire_type;

    common_hal_busio_onewire_construct(self, pin);
    return MP_OBJ_FROM_PTR(self);
}

//|   .. method:: deinit()
//|
//|      Deinitialize the OneWire bus and release any hardware resources for reuse.
//|
STATIC mp_obj_t busio_onewire_deinit(mp_obj_t self_in) {
    busio_onewire_obj_t *self = MP_OBJ_TO_PTR(self_in);
    common_hal_busio_onewire_deinit(self);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(busio_onewire_deinit_obj, busio_onewire_deinit);

//|   .. method:: __enter__()
//|
//|      No-op used by Context Managers.
//|
//  Provided by context manager helper.

//|   .. method:: __exit__()
//|
//|      Automatically deinitializes the hardware when exiting a context. See
//|      :ref:`lifetime-and-contextmanagers` for more info.
//|
STATIC mp_obj_t busio_onewire_obj___exit__(size_t n_args, const mp_obj_t *args) {
    (void)n_args;
    common_hal_busio_onewire_deinit(args[0]);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(busio_onewire___exit___obj, 4, 4, busio_onewire_obj___exit__);

//|   .. method:: reset()
//|
//|     Reset the OneWire bus and read presence
//|
//|     :returns: False when at least one device is present
//|     :rtype: bool
//|
STATIC mp_obj_t busio_onewire_obj_reset(mp_obj_t self_in) {
    busio_onewire_obj_t *self = MP_OBJ_TO_PTR(self_in);
    raise_error_if_deinited(common_hal_busio_onewire_deinited(self));

    return mp_obj_new_bool(common_hal_busio_onewire_reset(self));
}
MP_DEFINE_CONST_FUN_OBJ_1(busio_onewire_reset_obj, busio_onewire_obj_reset);

//|   .. method:: mode(value)
//|
//|     Configures the bus into varios modes
//|         0 -> Dallas / Maxium 1-Wire
//|         1 -> Atmel Single-Wire
//|         2 -> Atmel SI/O
//|
STATIC mp_obj_t busio_onewire_obj_mode(mp_obj_t self_in, mp_obj_t int_obj) {
    busio_onewire_obj_t *self = MP_OBJ_TO_PTR(self_in);
    raise_error_if_deinited(common_hal_busio_onewire_deinited(self));

    common_hal_busio_onewire_mode(self, mp_obj_get_int(int_obj));
    return mp_obj_new_int_from_uint(self->bitbang.mode);
}
MP_DEFINE_CONST_FUN_OBJ_2(busio_onewire_mode_obj, busio_onewire_obj_mode);

//|   .. method:: read_bit()
//|
//|     Read in a bit
//|
//|     :returns: bit state read
//|     :rtype: bool
//|
STATIC mp_obj_t busio_onewire_obj_read_bit(mp_obj_t self_in) {
    busio_onewire_obj_t *self = MP_OBJ_TO_PTR(self_in);
    raise_error_if_deinited(common_hal_busio_onewire_deinited(self));

    return mp_obj_new_bool(common_hal_busio_onewire_read_bit(self));
}
MP_DEFINE_CONST_FUN_OBJ_1(busio_onewire_read_bit_obj, busio_onewire_obj_read_bit);

//|   .. method:: read_byte()
//|
//|     Read a byte off the bus
//|
STATIC mp_obj_t busio_onewire_obj_read_byte(mp_obj_t self_in) {
    busio_onewire_obj_t *self = MP_OBJ_TO_PTR(self_in);
    raise_error_if_deinited(common_hal_busio_onewire_deinited(self));

    uint8_t data = 0;
    common_hal_busio_onewire_read_byte(self, &data);
    return mp_obj_new_int_from_uint(data);
}
MP_DEFINE_CONST_FUN_OBJ_1(busio_onewire_read_byte_obj, busio_onewire_obj_read_byte);

//|   .. method:: readfrom_into(device_address, memory_address, buffer, \*, start=0, end=len(buffer))
//|
//|      Read into ``buffer`` from the slave specified by ``device_address`` and ``memory_address``.
//|      If your read does not require a memory_address (some op-codes do and some do not),
//|      then it the parameter should be set to zero.
//|
//|      The number of bytes read will be the length of ``buffer``.
//|      At least one byte must be read.
//|
//|      If ``start`` or ``end`` is provided, then the buffer will be sliced
//|      as if ``buffer[start:end]``. This will not cause an allocation like
//|      ``buf[start:end]`` will so it saves memory.
//|
//|      :param int device_address: device address
//|      :param int memory_address: memory address (a 0 will cause this field to be omitted in the transaction)
//|      :param bytearray buffer: buffer to write into
//|      :param int start: Index to start writing at
//|      :param int end: Index to write up to but not include
//|
STATIC mp_obj_t busio_onewire_obj_readfrom_into(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_device_address, ARG_buffer, ARG_start, ARG_end };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_device_address,    MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_buffer,     MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_start,      MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_end,        MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = INT_MAX} },
    };
    busio_onewire_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    raise_error_if_deinited(common_hal_busio_onewire_deinited(self));
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[ARG_buffer].u_obj, &bufinfo, MP_BUFFER_WRITE);

    int32_t start = args[ARG_start].u_int;
    uint32_t length = bufinfo.len;
    normalize_buffer_bounds(&start, args[ARG_end].u_int, &length);
    if (length == 0) {
        mp_raise_ValueError(translate("Buffer must be at least length 1"));
    }
    uint8_t status = common_hal_busio_onewire_read(self,
                                                  args[ARG_device_address].u_int,
                                                  ((uint8_t*)bufinfo.buf) + start,
                                                  length);
    if (status != 0) {
        mp_raise_OSError(status);
    }
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(busio_onewire_readfrom_into_obj, 3, busio_onewire_obj_readfrom_into);

//|   .. method:: write_bit(value)
//|
//|     Write out a bit based on value.
//|
STATIC mp_obj_t busio_onewire_obj_write_bit(mp_obj_t self_in, mp_obj_t bool_obj) {
    busio_onewire_obj_t *self = MP_OBJ_TO_PTR(self_in);
    raise_error_if_deinited(common_hal_busio_onewire_deinited(self));

    common_hal_busio_onewire_write_bit(self, mp_obj_is_true(bool_obj));
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(busio_onewire_write_bit_obj, busio_onewire_obj_write_bit);

//|   .. method:: write_byte(value)
//|
//|     Write out a byte based on value and return ACK/NACK
//|
STATIC mp_obj_t busio_onewire_obj_write_byte(mp_obj_t self_in, mp_obj_t int_obj) {
    busio_onewire_obj_t *self = MP_OBJ_TO_PTR(self_in);
    raise_error_if_deinited(common_hal_busio_onewire_deinited(self));

    return mp_obj_new_bool(common_hal_busio_onewire_write_byte(self, mp_obj_get_int(int_obj)));
}
MP_DEFINE_CONST_FUN_OBJ_2(busio_onewire_write_byte_obj, busio_onewire_obj_write_byte);

//|   .. method:: I2C.writeto(address, buffer, \*, start=0, end=len(buffer), stop=True)
//|
//|      Write the bytes from ``buffer`` to the slave specified by ``address``.
//|      Transmits a stop bit if ``stop`` is set.
//|
//|      If ``start`` or ``end`` is provided, then the buffer will be sliced
//|      as if ``buffer[start:end]``. This will not cause an allocation like
//|      ``buffer[start:end]`` will so it saves memory.
//|
//|      Writing a buffer or slice of length zero is permitted, as it can be used
//|      to poll for the existence of a device.
//|
//|      :param int device_address: device address
//|      :param int memory_address: memory address (0 will ignore this field in the transaction)
//|      :param bytearray buffer: buffer containing the bytes to write
//|      :param int start: Index to start writing from
//|      :param int end: Index to read up to but not include
//|
STATIC mp_obj_t busio_onewire_obj_writeto(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_device_address, ARG_memory_address, ARG_buffer, ARG_start, ARG_end };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_device_address,    MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_memory_address,    MP_ARG_REQUIRED  | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_buffer,     MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_start,      MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_end,        MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = INT_MAX} },
    };
    busio_onewire_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    raise_error_if_deinited(common_hal_busio_onewire_deinited(self));

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // get the buffer to write the data from
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[ARG_buffer].u_obj, &bufinfo, MP_BUFFER_READ);

    int32_t start = args[ARG_start].u_int;
    uint32_t length = bufinfo.len;
    normalize_buffer_bounds(&start, args[ARG_end].u_int, &length);

    // do the transfer
    uint8_t status = common_hal_busio_onewire_write(self,
        args[ARG_device_address].u_int, args[ARG_memory_address].u_int,
        ((uint8_t*) bufinfo.buf) + start, length);
    if (status != 0) {
        mp_raise_OSError(status);
    }
    return mp_const_none;

}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(busio_onewire_writeto_obj, 1, busio_onewire_obj_writeto);

STATIC const mp_rom_map_elem_t busio_onewire_locals_dict_table[] = {
    // Methods
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&busio_onewire_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR___enter__), MP_ROM_PTR(&default___enter___obj) },
    { MP_ROM_QSTR(MP_QSTR___exit__), MP_ROM_PTR(&busio_onewire___exit___obj) },
    { MP_ROM_QSTR(MP_QSTR_reset), MP_ROM_PTR(&busio_onewire_reset_obj) },
    { MP_ROM_QSTR(MP_QSTR_mode), MP_ROM_PTR(&busio_onewire_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_bit), MP_ROM_PTR(&busio_onewire_read_bit_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_byte), MP_ROM_PTR(&busio_onewire_read_byte_obj) },
    { MP_ROM_QSTR(MP_QSTR_readfrom_into), MP_ROM_PTR(&busio_onewire_readfrom_into_obj) },
    { MP_ROM_QSTR(MP_QSTR_write_bit), MP_ROM_PTR(&busio_onewire_write_bit_obj) },
    { MP_ROM_QSTR(MP_QSTR_write_byte), MP_ROM_PTR(&busio_onewire_write_byte_obj) },
    { MP_ROM_QSTR(MP_QSTR_writeto), MP_ROM_PTR(&busio_onewire_writeto_obj) },
};
STATIC MP_DEFINE_CONST_DICT(busio_onewire_locals_dict, busio_onewire_locals_dict_table);

const mp_obj_type_t busio_onewire_type = {
    { &mp_type_type },
    .name = MP_QSTR_OneWire,
    .make_new = busio_onewire_make_new,
    .locals_dict = (mp_obj_dict_t*)&busio_onewire_locals_dict,
};
