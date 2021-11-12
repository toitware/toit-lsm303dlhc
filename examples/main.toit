// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by a Zero-Clause BSD license that can
// be found in the EXAMPLES_LICENSE file.

import gpio
import i2c
import lsm303dlhc show *

main:
  bus := i2c.Bus
    --sda=gpio.Pin 21
    --scl=gpio.Pin 22

  accelerometer := Accelerometer (bus.device Accelerometer.I2C_ADDRESS)
  magnetometer := Magnetometer (bus.device Magnetometer.I2C_ADDRESS)

  accelerometer.enable
  acceleration := accelerometer.read
  print "Acceleration (in m/s²): $acceleration"

  magnetometer.enable
  field := magnetometer.read
  print "Magnetic field (in micro-tesla): $field"

  temp := magnetometer.read_temperature
  print "Temperature (in C): $temp"
