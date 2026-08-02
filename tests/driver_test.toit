// Copyright (C) 2026 Toit contributors.
// Use of this source code is governed by an MIT-style license that can be found
// in the LICENSE file.

import expect show *
import io
import serial.device show Device
import serial.registers show Registers

import lsm303dlhc.accelerometer show Accelerometer
import lsm303dlhc.magnetometer show Magnetometer

WHO-AM-I ::= 0x0f
CTRL-REG4-A ::= 0x23
OUT-X-L-A ::= 0x28
OUT-X-H-M ::= 0x03
TEMP-OUT-H-M ::= 0x31

main:
  test-bdu
  test-accelerometer-burst
  test-magnetometer-burst
  test-temperature
  test-saturation-boundaries

test-bdu:
  registers := accelerometer-registers
  accelerometer := Accelerometer (FakeDevice registers)
  accelerometer.enable
  expect-equals 0x80 registers[CTRL-REG4-A]

test-accelerometer-burst:
  registers := accelerometer-registers
  accelerometer := Accelerometer (FakeDevice registers)
  accelerometer.enable
  registers.set-i16-le OUT-X-L-A (100 << 4)
  registers.set-i16-le (OUT-X-L-A + 2) (-200 << 4)
  registers.set-i16-le (OUT-X-L-A + 4) (300 << 4)
  registers.reset-read-log

  acceleration := accelerometer.read
  expect-close 0.980665 acceleration.x
  expect-close -1.96133 acceleration.y
  expect-close 2.941995 acceleration.z
  expect-equals 1 registers.read-calls
  expect-equals 0xa8 registers.last-read-register
  expect-equals 6 registers.last-read-count

test-magnetometer-burst:
  registers := magnetometer-registers
  magnetometer := Magnetometer (FakeDevice registers)
  registers.set-i16-be OUT-X-H-M 0x1234
  registers.set-i16-be (OUT-X-H-M + 2) 0x2345  // Z precedes Y in the register map.
  registers.set-i16-be (OUT-X-H-M + 4) -1_234
  registers.reset-read-log

  raw := magnetometer.read --raw
  expect-list-equals [0x1234, -1_234, 0x2345] raw
  expect-equals 1 registers.read-calls
  expect-equals OUT-X-H-M registers.last-read-register
  expect-equals 6 registers.last-read-count

test-temperature:
  registers := magnetometer-registers
  magnetometer := Magnetometer (FakeDevice registers)

  // The signed 12-bit value is left-justified; +8 steps is +1 degree.
  registers.set-i16-be TEMP-OUT-H-M (8 << 4)
  expect-close 1.0 magnetometer.read-temperature

  registers.set-i16-be TEMP-OUT-H-M (-8 << 4)
  expect-close -1.0 magnetometer.read-temperature

test-saturation-boundaries:
  registers := magnetometer-registers
  magnetometer := Magnetometer (FakeDevice registers)
  magnetometer.enable
  registers.set-i16-be OUT-X-H-M -2048
  registers.set-i16-be (OUT-X-H-M + 2) 0
  registers.set-i16-be (OUT-X-H-M + 4) 2047

  field := magnetometer.read
  expect-close (-2048 * 100.0 / 1100) field.x
  expect-close (2047 * 100.0 / 1100) field.y
  expect-close 0.0 field.z

  registers.set-i16-be OUT-X-H-M -4096
  field = magnetometer.read
  expect-equals -float.INFINITY field.x

expect-close expected/num actual/num:
  expect (expected - actual).abs < 0.000_001

accelerometer-registers -> FakeRegisters:
  result := FakeRegisters
  result[WHO-AM-I] = 0x33
  return result

magnetometer-registers -> FakeRegisters:
  result := FakeRegisters
  result[0x0a] = 0x48
  result[0x0b] = 0x34
  result[0x0c] = 0x33
  return result

class FakeDevice implements Device:
  registers_/Registers

  constructor .registers_:

  registers -> Registers:
    return registers_

  read amount/int -> ByteArray:
    throw "UNIMPLEMENTED"

  write bytes/ByteArray -> none:
    throw "UNIMPLEMENTED"

class FakeRegisters extends Registers:
  bytes_/ByteArray := ByteArray 0x80
  read-calls/int := 0
  last-read-register/int := -1
  last-read-count/int := -1

  read-bytes register/int count/int -> ByteArray:
    read-calls++
    last-read-register = register
    last-read-count = count
    address := register & 0x7f
    result := ByteArray count
    count.repeat:
      result[it] = bytes_[address + it]
    return result

  write-bytes register/int data/ByteArray -> none:
    address := register & 0x7f
    data.size.repeat:
      bytes_[address + it] = data[it]

  operator [] index/int -> int:
    return bytes_[index]

  operator []= index/int value/int -> none:
    bytes_[index] = value

  set-i16-le register/int value/int -> none:
    io.LITTLE-ENDIAN.put-int16 bytes_ register value

  set-i16-be register/int value/int -> none:
    io.BIG-ENDIAN.put-int16 bytes_ register value

  reset-read-log:
    read-calls = 0
    last-read-register = -1
    last-read-count = -1
