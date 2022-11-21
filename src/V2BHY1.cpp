// © Kay Sievers <kay@versioduo.com>, 2022
// SPDX-License-Identifier: Apache-2.0

#include "V2BHY1.h"

extern "C" {
#include "bhy/bhy_firmware.h"
#include "bhy/bhy_support.h"
#include "bhy/bhy_uc_driver.h"
}

extern TwoWire *i2c;

static class Sensor {
public:
  enum class State { Init, WaitForInit, Setup, Running, _count } state;
  bhy_data_quaternion_t rotation;
  bhy_data_quaternion_t game;
  bhy_data_vector_t gravity;

  void reset() {
    state    = {};
    rotation = {.w{16384}};
    game     = {.w{16384}};
    gravity  = {};
  }
} Sensor;

void fifoDataHandler(bhy_data_generic_t *data, bhy_virtual_sensor_t sensor_id) {
  switch (sensor_id) {
    case VS_ID_ROTATION_VECTOR_WAKEUP:
      Sensor.rotation = data->data_quaternion;
      break;

    case VS_ID_GAME_ROTATION_VECTOR_WAKEUP:
      Sensor.game = data->data_quaternion;
      break;

    case VS_ID_GRAVITY_WAKEUP:
      Sensor.gravity = data->data_vector;
      break;
  }
}

static class {
public:
  volatile bool pending;

  void reset() {
    pending          = false;
    _bytes_left      = 0;
    _bytes_remaining = 0;
  }

  bool hasData() const {
    return _bytes_remaining > 0;
  }

  void processEvents() {
    pending = false;

    bhy_read_fifo(_data + _bytes_left, sizeof(_data) - _bytes_left, &_bytes_read, &_bytes_remaining);
    _bytes_read += _bytes_left;
    _pos         = _data;
    _packet_type = BHY_DATA_TYPE_PADDING;

    for (;;) {
      if (bhy_parse_next_fifo_packet(&_pos, &_bytes_read, &_packet, &_packet_type) != BHY_SUCCESS)
        break;

      if (_bytes_read <= (_bytes_remaining > 0 ? sizeof(bhy_data_generic_t) : 0))
        break;
    }

    _bytes_left = 0;
    if (_bytes_remaining == 0)
      return;

    while (_bytes_left < _bytes_read)
      _data[_bytes_left++] = *(_pos++);
  }

private:
  uint8_t _data[300]{};
  uint8_t *_pos{};
  uint8_t _bytes_left{};
  uint16_t _bytes_remaining{};
  uint16_t _bytes_read{};
  bhy_data_generic_t _packet{};
  bhy_data_type_t _packet_type{};
} FIFO;

static void fifoInterruptHandler(void) {
  FIFO.pending = true;
}

void V2BHY1::begin() {
  i2c = _i2c;
  attachInterrupt(_pin_interrupt, fifoInterruptHandler, RISING);
  reset();
}

void V2BHY1::reset() {
  Sensor.reset();
}

void V2BHY1::loop() {
  switch (Sensor.state) {
    case Sensor::State::Init:
      bhy_driver_init(bhy_firmware_bmm150);
      FIFO.reset();
      Sensor.state = Sensor::State::WaitForInit;
      break;

    case Sensor::State::WaitForInit:
      if (!FIFO.pending)
        return;

      Sensor.state = Sensor::State::Setup;
      break;

    case Sensor::State::Setup:
      bhy_install_sensor_callback(VS_TYPE_ROTATION_VECTOR, VS_WAKEUP, fifoDataHandler);
      bhy_enable_virtual_sensor(VS_TYPE_ROTATION_VECTOR, VS_WAKEUP, 100, 0, VS_FLUSH_NONE, 0, 0);

      bhy_install_sensor_callback(VS_TYPE_GAME_ROTATION_VECTOR, VS_WAKEUP, fifoDataHandler);
      bhy_enable_virtual_sensor(VS_TYPE_GAME_ROTATION_VECTOR, VS_WAKEUP, 100, 0, VS_FLUSH_NONE, 0, 0);

      bhy_install_sensor_callback(VS_TYPE_GRAVITY, VS_WAKEUP, fifoDataHandler);
      bhy_enable_virtual_sensor(VS_TYPE_GRAVITY, VS_WAKEUP, 100, 0, VS_FLUSH_NONE, 0, 0);

      Sensor.state = Sensor::State::Running;
      break;

    case Sensor::State::Running:
      if (!FIFO.pending && !FIFO.hasData())
        return;

      FIFO.processEvents();
      break;
  }
}

V23D::Quaternion V2BHY1::getGeoOrientation() {
  return V23D::Quaternion((float)Sensor.rotation.w / 16384.f,
                          (float)Sensor.rotation.x / 16384.f,
                          (float)Sensor.rotation.y / 16384.f,
                          (float)Sensor.rotation.z / 16384.f);
}

V23D::Quaternion V2BHY1::getOrientation() {
  return V23D::Quaternion((float)Sensor.game.w / 16384.f,
                          (float)Sensor.game.x / 16384.f,
                          (float)Sensor.game.y / 16384.f,
                          (float)Sensor.game.z / 16384.f);
}

V23D::Vector3 V2BHY1::getGravity() {
  return V23D::Vector3((float)Sensor.gravity.x / 8192.f,
                       (float)Sensor.gravity.y / 8192.f,
                       (float)Sensor.gravity.z / 8192.f);
}

uint16_t V2BHY1::getRAMVersion() {
  uint16_t version{};
  bhy_get_ram_version(&version);
  return version;
}

uint8_t V2BHY1::getProductID() {
  uint8_t id{};
  bhy_get_product_id(&id);
  return id;
}

uint8_t V2BHY1::getRevisionID() {
  uint8_t id{};
  bhy_get_revision_id(&id);
  return id;
}
