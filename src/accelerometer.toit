// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by an MIT-style license that can be found
// in the LICENSE file.

import serial.device as serial
import serial.registers as serial
import math

/**
Driver for the accelerometer of the LSM303DLHC module.
*/
class Accelerometer:
  static I2C_ADDRESS ::= 0b11001  // 5.1.2.

  /** Sampling rate of 1Hz.
  */
  static RATE_1HZ    ::= 0b0001
  static RATE_10HZ   ::= 0b0010
  static RATE_25HZ   ::= 0b0011
  static RATE_50HZ   ::= 0b0100
  static RATE_100HZ  ::= 0b0101
  static RATE_200HZ  ::= 0b0110
  static RATE_400HZ  ::= 0b0111
  static RATE_1620HZ ::= 0b1000
  static RATE_1344HZ ::= 0b1001
  static RATE_5376HZ ::= 0b1001

  static MODE_NORMAL ::= 0
  static MODE_LOW_POWER ::= 1
  static MODE_HIGH_RESOLUTION ::= 2

  static RANGE_2G  ::= 0
  static RANGE_4G  ::= 1
  static RANGE_8G  ::= 2
  static RANGE_16G ::= 3

  // The LSM303AGR datasheet states that this register always
  // returns 0x33. The LSM303DLHC datasheet doesn't mention this
  // functionality, but it seems to be enabled there as well.
  static WHO_AM_I_ ::= 0x0F

  // 6. Register mapping.
  static CTRL_REG1_A_ ::= 0x20
  static CTRL_REG4_A_ ::= 0x23
  static OUT_X_L_A_ ::= 0x28
  static OUT_X_H_A_ ::= 0x29
  static OUT_Y_L_A_ ::= 0x2A
  static OUT_Y_H_A_ ::= 0x2B
  static OUT_Z_L_A_ ::= 0x2C
  static OUT_Z_H_A_ ::= 0x2D

  /**
  Standard acceleration due to gravity.
  In m/s².
  */
  static GRAVITY_STANDARD_ ::= 9.80665

  reg_ /serial.Registers
  range_ /int := 0

  constructor dev/serial.Device:
    reg_ = dev.registers

    id := reg_.read_u8 WHO_AM_I_
    // The WHO_AM_I_ register is from the LSM303AGR. It seems like the
    // LSM303DLHC behaves the same way.
    // Section 7, Table 26 of the LSM303AGR datasheet.
    if id != 0x33: throw "INVALID_CHIP"


  /**
  Enables the sensor.

  The $mode must be one of:
  - $MODE_NORMAL: normal operation. Provides 10 bits of precision for each axis.
  - $MODE_LOW_POWER: low-power mode. Provides 8 bits of precision.
  - $MODE_HIGH_RESOLUTION: high-resolution mode. Provides 12 bits of precision.

  The $rate parameter defines the frequency at which measurements are taken.
  Valid values for $rate are:
  - $RATE_1HZ
  - $RATE_10HZ
  - $RATE_25HZ
  - $RATE_50HZ
  - $RATE_100HZ
  - $RATE_200HZ
  - $RATE_400HZ
  - $RATE_1344HZ only if $mode is equal to $MODE_NORMAL or $MODE_HIGH_RESOLUTION
  - $RATE_1620HZ only if $mode is equal to $MODE_LOW_POWER
  - $RATE_5376HZ only if $mode is equal to $MODE_LOW_POWER

  The $range parameter defines the measured acceleration range.
  Valid values for $range are:
  - $RANGE_2G: +-2G (19.61 m/s²)
  - $RANGE_4G: +-4G (39.23 m/s²)
  - $RANGE_8G: +-8G (78.45 m/s²)
  - $RANGE_16G: +-16G (156.9 m/s²)
  */
  enable -> none
      --mode  /int = MODE_NORMAL
      --rate  /int = RATE_100HZ
      --range /int = RANGE_2G:

    // 7.1.1. CTRL_REG1_A.
    rate_bits := ?
    // Table 20. Data rate configuration.
    if rate == RATE_1HZ:         rate_bits = 0b0001_0000
    else if rate == RATE_10HZ:   rate_bits = 0b0010_0000
    else if rate == RATE_25HZ:   rate_bits = 0b0011_0000
    else if rate == RATE_50HZ:   rate_bits = 0b0100_0000
    else if rate == RATE_100HZ:  rate_bits = 0b0101_0000
    else if rate == RATE_200HZ:  rate_bits = 0b0110_0000
    else if rate == RATE_400HZ:  rate_bits = 0b0111_0000
    else if rate == RATE_1620HZ and mode == MODE_LOW_POWER: rate_bits = 0b1000_0000
    else if rate == RATE_1344HZ and mode != MODE_LOW_POWER: rate_bits = 0b1001_0000
    else if rate == RATE_5376HZ and mode == MODE_LOW_POWER: rate_bits = 0b1001_0000
    else: throw "INVALID_RATE"

    assert: MODE_NORMAL == 0 and MODE_LOW_POWER == 1 and MODE_HIGH_RESOLUTION == 2
    if not 0 <= mode < 3: throw "INVALID_MODE"

    low_power_bit := mode == MODE_LOW_POWER ? 0b1000 : 0b0000

    // We always enable all three axes.
    axis_bits := 0b111

    reg1_value := rate_bits | low_power_bit | axis_bits

    // 7.1.4. CTRL_REG4_A.
    // Table 27. CTRL_REG4_A description.
    // Set Block data update to 0. (continuous update).
    // Set Big/little endian data selection to 0. (LSB at lower address).
    reg4_value := 0

    if not 0 <= range < 4: throw "INVALID_RANGE"
    reg4_value |= range << 4
    range_ = range

    if mode == MODE_HIGH_RESOLUTION: reg4_value |= 0b1000

    reg_.write_u8 CTRL_REG1_A_ reg1_value
    reg_.write_u8 CTRL_REG4_A_ reg4_value

    sleep --ms=10

  /**
  Disables the accelerometer.
  Initiates a power-down of the peripheral. It is safe to call $enable
    to restart the accelerometer.
  */
  disable:
    // Fundamentally we only care for the rate-bits: as long as they
    // are 0, the device is disabled.
    // It's safe to change the other bits as well.
    reg_.write_u8 CTRL_REG1_A_ 0x00

  /**
  Reads the x, y and z axis.
  The returned values are in in m/s².
  */
  read -> math.Point3f:
    AUTO_INCREMENT_BIT ::= 0b1000_0000
    x := reg_.read_i16_le (OUT_X_L_A_ | AUTO_INCREMENT_BIT)
    y := reg_.read_i16_le (OUT_Y_L_A_ | AUTO_INCREMENT_BIT)
    z := reg_.read_i16_le (OUT_Z_L_A_ | AUTO_INCREMENT_BIT)

    // Section 2.1, table3:
    // The linear acceleration sensitivity depends on the range:
    // - RANGE_2G:   1mg/LSB
    // - RANGE_4G:   2mg/LSB
    // - RANGE_8G:   4mg/LSB
    // - RANGE_16G: 12mg/LSB   // <- Note that the 16G sensitivity is not 8mg/LSB as expected.
    // See the explanation for the LSB (least-significant bit) below.
    SENSITIVITIES ::= #[1, 2, 4, 12]
    sensitivity := SENSITIVITIES[range_]
    x *= sensitivity
    y *= sensitivity
    z *= sensitivity

    // The sensor returns its measurements in the 12 most significant bits.
    // In RANGE_2G mode, we would therefore need to shift down by 4: x >>= 4.
    // However, since we multiply by the standard gravity constant anyway, we include that
    // multiplication there (and let the compiler constant fold the value).
    // Note that high-resolution and low-power mode just provide more or fewer
    // digits, padding to the right (least-significant digits).
    // TODO(florian): verify that the padded digits are 0s (although it shouldn't really matter).
    factor := GRAVITY_STANDARD_ / 1000.0 / (1 << 4)  // Constant folded because it's one expression.
    return math.Point3f
        x * factor
        y * factor
        z * factor

  /**
  Reads the current range setting of the sensor.
  Returns $RANGE_2G, $RANGE_4G, $RANGE_8G or $RANGE_16G.
  */
  read_range -> int:
    reg4 := reg_.read_u8 CTRL_REG4_A_
    return (reg4 >> 4) & 0b11
