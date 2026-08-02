// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by an MIT-style license that can be found
// in the LICENSE file.

import serial.device as serial
import serial.registers as serial
import io
import math

/**
Driver for the magnetometer of the LSM303DLHC module.
*/
class Magnetometer:
  static I2C-ADDRESS ::= 0b11110  // 5.1.3.

  // 6. Register mapping.
  static CRA-REG-M_ ::= 0x00
  static CRB-REG-M_ ::= 0x01
  static MR-REG-M_ ::= 0x02
  static OUT-X-H-M_ ::= 0x03
  static OUT-X-L-M_ ::= 0x04
  static OUT-Z-H-M_ ::= 0x05
  static OUT-Z-L-M_ ::= 0x06
  static OUT-Y-H-M_ ::= 0x07
  static OUT-Y-L-M_ ::= 0x08
  static IRA-REG-M_ ::= 0x0A
  static IRB-REG-M_ ::= 0x0B
  static IRC-REG-M_ ::= 0x0C
  static TEMP-OUT-H-M_ ::= 0x31
  static TEMP-OUT-L-M_ ::= 0x32

  static RATE-0-75HZ ::= 0
  static RATE-1-5HZ  ::= 1
  static RATE-3HZ    ::= 2
  static RATE-7-5HZ  ::= 3
  static RATE-15HZ   ::= 4
  static RATE-30HZ   ::= 5
  static RATE-75HZ   ::= 6
  static RATE-220HZ  ::= 7

  static RANGE-1-3G ::= 1
  static RANGE-1-9G ::= 2
  static RANGE-2-5G ::= 3
  static RANGE-4-0G ::= 4
  static RANGE-4-7G ::= 5
  static RANGE-5-6G ::= 6
  static RANGE-8-1G ::= 7

  static GAUSS-TO-MICROTESLA_ ::= 100.0

  reg_ /serial.Registers
  gain-xy_ /int := -1
  gain-z_  /int := -1

  constructor dev/serial.Device:
    reg_ = dev.registers

    // There is no who-am-i register on the magnetometer, but the
    // IRx_REG_M registers seem to be constant (x being A, B and C).
    // Section 6. Table 17.
    value := reg_.read-u24-be IRA-REG-M_
    if value != 0x483433: throw "INVALID_CHIP"

  enable -> none
      --rate  /int = RATE-15HZ
      --range /int = RANGE-1-3G
      :
    if not 0 <= rate < 8: throw "INVALID_RATE"
    if not 1 <= range < 8: throw "INVALID_RANGE"

    // 7.2.1, Table 71.
    // Enable the temperature sensor, and apply the rate.
    // The temperature sensor is disabled by default, but I can't see
    //   any reason why one would want to disable it.
    cra-value := 0b1000_0000
    // Apply the rate.
    cra-value |= rate << 2
    reg_.write-u8 CRA-REG-M_ cra-value

    // 7.2.2, Table 73 and table 75.
    // Most significant bits of CRB_REG_M defines the range.
    reg_.write-u8 CRB-REG-M_ (range << 5)
    // Remember the gains (given by the range).
    if range == RANGE-1-3G:
      gain-xy_ = 1100
      gain-z_ = 980
    else if range == RANGE-1-9G:
      gain-xy_ = 855
      gain-z_ = 760
    else if range == RANGE-2-5G:
      gain-xy_ = 670
      gain-z_ = 600
    else if range == RANGE-4-0G:
      gain-xy_ = 450
      gain-z_ = 400
    else if range == RANGE-4-7G:
      gain-xy_ = 400
      gain-z_ = 355
    else if range == RANGE-5-6G:
      gain-xy_ = 330
      gain-z_ = 295
    else if range == RANGE-8-1G:
      gain-xy_ = 230
      gain-z_ = 205

    // 7.2.3, Table 78.
    // Continuous-conversion mode.
    reg_.write-u8 MR-REG-M_ 0x0

  disable -> none:
    // 7.2.3, Table 78.
    // Sleep mode.
    reg_.write-u8 MR-REG-M_ 0b11

  /**
  Reads the temperature.
  Returns the result in Celsius.
  */
  read-temperature -> float:
    // 7.2.9, Table 86.
    // Unlike the LSM303D, the LSM303DLHC stores temperature left-justified.
    // The value is a left-justified, 12-bit two's complement integer.
    // 8 steps per degree. This means that there are 3 fractional bits.
    // If we just wanted to return an integer temperature value we could
    //   return `value >> 7`.
    // TODO(florian): check that the least 4 significant bits are equal to 0.
    //   Shouldn't matter too much if they aren't.
    value := reg_.read-i16-be TEMP-OUT-H-M_
    return value * (1.0 / 128.0)  // Let the compiler constant-fold the division.

  /**
  Reads the magnetic field.
  The returned values are in microtesla.
  If a value is out of range, +-$float.INFINITY is used. In this case
    changing the range (see $enable) might be an option to allow the
    sensor to measure the magnetic field.
  */
  read -> math.Point3f:
    raw := read-raw_
    x := raw[0]
    y := raw[1]
    z := raw[2]

    x-converted := x * GAUSS-TO-MICROTESLA_ / gain-xy_
    y-converted := y * GAUSS-TO-MICROTESLA_ / gain-xy_
    z-converted := z * GAUSS-TO-MICROTESLA_ / gain-z_

    // Check for saturation.
    if not -2048 <= x <= 2047: x-converted = x.sign * float.INFINITY
    if not -2048 <= y <= 2047: y-converted = y.sign * float.INFINITY
    if not -2048 <= z <= 2047: z-converted = z.sign * float.INFINITY

    return math.Point3f
        x-converted
        y-converted
        z-converted

  /**
  Reads the raw magnetic field values.
  */
  read --raw -> List:
    if not raw: throw "INVALID_ARGUMENT"
    return read-raw_

  read-raw_ -> List:
    bytes := reg_.read-bytes OUT-X-H-M_ 6
    return [
      io.BIG-ENDIAN.int16 bytes 0,
      io.BIG-ENDIAN.int16 bytes 4,
      io.BIG-ENDIAN.int16 bytes 2,
    ]
