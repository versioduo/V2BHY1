// © Kay Sievers <kay@versioduo.com>, 2022
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Arduino.h>
#include <V23D.h>
#include <Wire.h>

// XYZ – ENU (East-North-Up), right handed.
class V2BHY1 {
public:
  constexpr V2BHY1(TwoWire *i2c, uint8_t pin_interrupt) : _pin_interrupt(pin_interrupt), _i2c(i2c) {}

  void begin();
  void reset();
  void loop();

  // Use magnetometer, orient towards magnetic north, normalized.
  V23D::Quaternion getGeoOrientation();

  // Do not use magnetometer, relative orientation only, normalized.
  V23D::Quaternion getOrientation();

  // Gravity vector.
  V23D::Vector3 getGravity();

  uint16_t getRAMVersion();
  uint8_t getProductID();
  uint8_t getRevisionID();

private:
  uint8_t _pin_interrupt;
  TwoWire *_i2c;
};
