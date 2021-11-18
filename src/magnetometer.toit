// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

import serial.device as serial
import serial.registers as serial
import math

/**
Driver for the magnetometer of the LSM303DLHC module.
*/
class Magnetometer:
  static I2C_ADDRESS ::= 0b11110  // 5.1.3.

  // 6. Register mapping.
  static CRA_REG_M_ ::= 0x00
  static CRB_REG_M_ ::= 0x01
  static MR_REG_M_ ::= 0x02
  static OUT_X_H_M_ ::= 0x03
  static OUT_X_L_M_ ::= 0x04
  static OUT_Z_H_M_ ::= 0x05
  static OUT_Z_L_M_ ::= 0x06
  static OUT_Y_H_M_ ::= 0x07
  static OUT_Y_L_M_ ::= 0x08
  static IRA_REG_M_ ::= 0x0A
  static IRB_REG_M_ ::= 0x0B
  static IRC_REG_M_ ::= 0x0C
  static TEMP_OUT_H_M_ ::= 0x31
  static TEMP_OUT_L_M_ ::= 0x32

  static RATE_0_75HZ ::= 0
  static RATE_1_5HZ  ::= 1
  static RATE_3HZ    ::= 2
  static RATE_7_5HZ  ::= 3
  static RATE_15HZ   ::= 4
  static RATE_30HZ   ::= 5
  static RATE_75HZ   ::= 6
  static RATE_220HZ  ::= 7

  static RANGE_1_3G ::= 1
  static RANGE_1_9G ::= 2
  static RANGE_2_5G ::= 3
  static RANGE_4_0G ::= 4
  static RANGE_4_7G ::= 5
  static RANGE_5_6G ::= 6
  static RANGE_8_1G ::= 7

  static GAUSS_TO_MICROTESLA_ ::= 100.0

  reg_ /serial.Registers
  gain_xy_ /int := -1
  gain_z_  /int := -1

  constructor dev/serial.Device:
    reg_ = dev.registers

    // There is no who-am-i register on the magnetometer, but the
    // IRx_REG_M registers seem to be constant (x being A, B and C).
    // Section 6. Table 17.
    value := reg_.read_u24_be IRA_REG_M_
    if value != 0x483433: throw "INVALID_CHIP"

  enable -> none
      --rate  /int = RATE_15HZ
      --range /int = RANGE_1_3G
      :
    if not 0 <= rate < 8: throw "INVALID_RATE"
    if not 1 <= range < 8: throw "INVALID_RANGE"

    // 7.2.1, Table 71.
    // Enable the temperature sensor, and apply the rate.
    // The temperature sensor is disabled by default, but I can't see
    //   any reason why one would want to disable it.
    cra_value := 0b1000_0000
    // Apply the rate.
    cra_value |= rate << 2
    reg_.write_u8 CRA_REG_M_ cra_value

    // 7.2.2, Table 73 and table 75.
    // Most significant bits of CRB_REG_M defines the range.
    reg_.write_u8 CRB_REG_M_ (range << 5)
    // Remember the gains (given by the range).
    if range == RANGE_1_3G:
      gain_xy_ = 1100
      gain_z_ = 980
    else if range == RANGE_1_9G:
      gain_xy_ = 855
      gain_z_ = 760
    else if range == RANGE_2_5G:
      gain_xy_ = 670
      gain_z_ = 600
    else if range == RANGE_4_0G:
      gain_xy_ = 450
      gain_z_ = 400
    else if range == RANGE_4_7G:
      gain_xy_ = 400
      gain_z_ = 355
    else if range == RANGE_5_6G:
      gain_xy_ = 330
      gain_z_ = 295
    else if range == RANGE_8_1G:
      gain_xy_ = 230
      gain_z_ = 205

    // 7.2.3, Table 78.
    // Continuous-conversion mode.
    reg_.write_u8 MR_REG_M_ 0x0

  disable -> none:
    // 7.2.3, Table 78.
    // Sleep mode.
    reg_.write_u8 MR_REG_M_ 0b11

  /**
  Reads the temperature.
  Returns the result in Celsius.
  */
  read_temperature -> float:
    // 7.2.9, Table 86.
    // 12 bit signed integer shifted by 4. In other words: a 16 bit integer with
    //   the least significant 4 bits not used.
    // 8 steps per degree. This means that there are 3 fractional bits.
    // If we just wanted to return an integer temperature value we could
    //   return `value >> 7`.
    // TODO(florian): check that the least 4 significant bits are equal to 0.
    //   Shouldn't matter too much if they aren't.
    value := reg_.read_i16_be TEMP_OUT_H_M_
    return value * (1.0 / 128.0)  // Let the compiler constant-fold the division.

  /**
  Reads the magnetic field.
  The returned values are in micro-tesla.
  If a value is out of range, +-$float.INFINITY is used. In this case
    changing the range (see $enable) might be an option to allow the
    sensor to measure the magnetic field.
  */
  read -> math.Point3f:
    x := reg_.read_i16_be OUT_X_H_M_
    z := reg_.read_i16_be OUT_Z_H_M_
    y := reg_.read_i16_be OUT_Y_H_M_

    x_converted := x * GAUSS_TO_MICROTESLA_ / gain_xy_
    y_converted := y * GAUSS_TO_MICROTESLA_ / gain_xy_
    z_converted := z * GAUSS_TO_MICROTESLA_ / gain_z_

    // Check for saturation.
    if not -2048 < x < 2047: x_converted = x.sign * float.INFINITY
    if not -2048 < y < 2047: y_converted = y.sign * float.INFINITY
    if not -2048 < z < 2047: z_converted = z.sign * float.INFINITY

    return math.Point3f
        x_converted
        y_converted
        z_converted
