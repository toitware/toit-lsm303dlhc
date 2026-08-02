// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by an MIT-style license that can be found
// in the LICENSE file.

import serial.device as serial
import serial.registers as serial
import io
import math

/**
Driver for the accelerometer of the LSM303DLHC module.
*/
class Accelerometer:
  static I2C-ADDRESS ::= 0b11001  // 5.1.2.

  /** Sampling rate of 1Hz.
  */
  static RATE-1HZ    ::= 0b0001
  static RATE-10HZ   ::= 0b0010
  static RATE-25HZ   ::= 0b0011
  static RATE-50HZ   ::= 0b0100
  static RATE-100HZ  ::= 0b0101
  static RATE-200HZ  ::= 0b0110
  static RATE-400HZ  ::= 0b0111
  static RATE-1620HZ ::= 0b1000
  static RATE-1344HZ ::= 0b1001
  static RATE-5376HZ ::= 0b1001

  static MODE-NORMAL ::= 0
  static MODE-LOW-POWER ::= 1
  static MODE-HIGH-RESOLUTION ::= 2

  static RANGE-2G  ::= 0
  static RANGE-4G  ::= 1
  static RANGE-8G  ::= 2
  static RANGE-16G ::= 3

  // The LSM303AGR datasheet states that this register always
  // returns 0x33. The LSM303DLHC datasheet doesn't mention this
  // functionality, but it seems to be enabled there as well.
  static WHO-AM-I_ ::= 0x0F

  // 6. Register mapping.
  static CTRL-REG1-A_ ::= 0x20
  static CTRL-REG4-A_ ::= 0x23
  static OUT-X-L-A_ ::= 0x28
  static OUT-X-H-A_ ::= 0x29
  static OUT-Y-L-A_ ::= 0x2A
  static OUT-Y-H-A_ ::= 0x2B
  static OUT-Z-L-A_ ::= 0x2C
  static OUT-Z-H-A_ ::= 0x2D

  static BDU-BIT_ ::= 1 << 7
  static AUTO-INCREMENT-BIT_ ::= 1 << 7

  /**
  Standard acceleration due to gravity.
  In m/s².
  */
  static GRAVITY-STANDARD_ ::= 9.80665

  reg_ /serial.Registers
  range_ /int := 0

  constructor dev/serial.Device:
    reg_ = dev.registers

    id := reg_.read-u8 WHO-AM-I_
    // The WHO_AM_I_ register is from the LSM303AGR. It seems like the
    // LSM303DLHC behaves the same way.
    // Section 7, Table 26 of the LSM303AGR datasheet.
    if id != 0x33: throw "INVALID_CHIP"


  /**
  Enables the sensor.

  The $mode must be one of:
  - $MODE-NORMAL: normal operation. Provides 10 bits of precision for each axis.
  - $MODE-LOW-POWER: low-power mode. Provides 8 bits of precision.
  - $MODE-HIGH-RESOLUTION: high-resolution mode. Provides 12 bits of precision.

  The $rate parameter defines the frequency at which measurements are taken.
  Valid values for $rate are:
  - $RATE-1HZ
  - $RATE-10HZ
  - $RATE-25HZ
  - $RATE-50HZ
  - $RATE-100HZ
  - $RATE-200HZ
  - $RATE-400HZ
  - $RATE-1344HZ only if $mode is equal to $MODE-NORMAL or $MODE-HIGH-RESOLUTION
  - $RATE-1620HZ only if $mode is equal to $MODE-LOW-POWER
  - $RATE-5376HZ only if $mode is equal to $MODE-LOW-POWER

  The $range parameter defines the measured acceleration range.
  Valid values for $range are:
  - $RANGE-2G: +-2G (19.61 m/s²)
  - $RANGE-4G: +-4G (39.23 m/s²)
  - $RANGE-8G: +-8G (78.45 m/s²)
  - $RANGE-16G: +-16G (156.9 m/s²)
  */
  enable -> none
      --mode  /int = MODE-NORMAL
      --rate  /int = RATE-100HZ
      --range /int = RANGE-2G:

    // 7.1.1. CTRL_REG1_A.
    rate-bits := ?
    // Table 20. Data rate configuration.
    if rate == RATE-1HZ:         rate-bits = 0b0001_0000
    else if rate == RATE-10HZ:   rate-bits = 0b0010_0000
    else if rate == RATE-25HZ:   rate-bits = 0b0011_0000
    else if rate == RATE-50HZ:   rate-bits = 0b0100_0000
    else if rate == RATE-100HZ:  rate-bits = 0b0101_0000
    else if rate == RATE-200HZ:  rate-bits = 0b0110_0000
    else if rate == RATE-400HZ:  rate-bits = 0b0111_0000
    else if rate == RATE-1620HZ and mode == MODE-LOW-POWER: rate-bits = 0b1000_0000
    else if rate == RATE-1344HZ and mode != MODE-LOW-POWER: rate-bits = 0b1001_0000
    else if rate == RATE-5376HZ and mode == MODE-LOW-POWER: rate-bits = 0b1001_0000
    else: throw "INVALID_RATE"

    assert: MODE-NORMAL == 0 and MODE-LOW-POWER == 1 and MODE-HIGH-RESOLUTION == 2
    if not 0 <= mode < 3: throw "INVALID_MODE"

    low-power-bit := mode == MODE-LOW-POWER ? 0b1000 : 0b0000

    // We always enable all three axes.
    axis-bits := 0b111

    reg1-value := rate-bits | low-power-bit | axis-bits

    // 7.1.4. CTRL_REG4_A.
    // Table 27. CTRL_REG4_A description.
    // Set Big/little endian data selection to 0. (LSB at lower address).
    reg4-value := 0
    // Prevent an update while output bytes are being read.
    reg4-value |= BDU-BIT_

    if not 0 <= range < 4: throw "INVALID_RANGE"
    reg4-value |= range << 4
    range_ = range

    if mode == MODE-HIGH-RESOLUTION: reg4-value |= 0b1000

    reg_.write-u8 CTRL-REG1-A_ reg1-value
    reg_.write-u8 CTRL-REG4-A_ reg4-value

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
    reg_.write-u8 CTRL-REG1-A_ 0x00

  /**
  Reads the x, y and z axis.
  The returned values are in in m/s².
  */
  read -> math.Point3f:
    raw := read-raw_
    x := raw[0]
    y := raw[1]
    z := raw[2]

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
    factor := GRAVITY-STANDARD_ / 1000.0 / (1 << 4)  // Constant folded because it's one expression.
    return math.Point3f
        x * factor
        y * factor
        z * factor

  /**
  Reads the current range setting of the sensor.
  Returns $RANGE-2G, $RANGE-4G, $RANGE-8G or $RANGE-16G.
  */
  read-range -> int:
    reg4 := reg_.read-u8 CTRL-REG4-A_
    return (reg4 >> 4) & 0b11

  read-raw_ -> List:
    bytes := reg_.read-bytes (OUT-X-L-A_ | AUTO-INCREMENT-BIT_) 6
    return [
      io.LITTLE-ENDIAN.int16 bytes 0,
      io.LITTLE-ENDIAN.int16 bytes 2,
      io.LITTLE-ENDIAN.int16 bytes 4,
    ]
