/*******************************************************************************
** MIT License
**
** Copyright(c) 2021 QuartzYan https://github.com/QuartzYan
**
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files(the "Software"), to deal
** in the Software without restriction, including without limitation the rights
** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
** copies of the Software, and to permit persons to whom the Software is
** furnished to do so, subject to the following conditions :
**
** The above copyright notice and this permission notice shall be included in all
** copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
** AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
** OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
** SOFTWARE.
*******************************************************************************/

#ifndef SCOUT_DRIVER_H
#define SCOUT_DRIVER_H

#include <serial/serial.h>
#include <iostream>
#include <sstream>
#include <thread>
#include <string>
#include <vector>
#include <mutex>

#include "QThread.h"

#define SOF_HEAD_1    0x5A
#define SOF_HEAD_2    0xA5

#define MAX_X_SPEED   1.5
#define MAX_TH_SPEED  0.5235
#define WHEEL_RADIUS  0.16459
#define WHEEL_BASE    0.498

#define SCOUT_CMD_BUF_LEN    32

class ScoutDriver : public QThread
{
public:
  typedef enum : uint8_t{
    CONST_OFF = 0x00,
    CONST_ON = 0x01,
    BREATH = 0x02,
    CUSTOM = 0x03,
  }LIGHT_MODE;

  typedef struct {
    float driver_voltage;
    float driver_temperature;
    uint8_t driver_state;
  }ScoutDriverState;

  typedef struct {
    uint8_t mode;
    uint8_t custom_value;
  }ScoutLightState;

  typedef struct scout_motor_state{
    uint8_t id;
    float current;
    float rpm;
    float temperature;
    double motor_pose;
    scout_motor_state() : motor_pose(0) {}
  }ScoutMotorState;

  typedef struct {
    uint8_t base_state;
    uint8_t control_mode;
    float battery_voltage;
    uint16_t fault_code;

    float vx;
    float vth;

    bool light_control_enabled;
    ScoutLightState front_light_state;
    ScoutLightState rear_light_state;

    std::array<ScoutMotorState, 4> motor_states;
    //std::array<ScoutDriverState, 4> driver_states;
  }ScoutStatus;

public:
  ScoutDriver(const std::string &portName);
  ~ScoutDriver();

  // do not allow copy
  ScoutDriver(const ScoutDriver &scout) = delete;
  ScoutDriver &operator=(const ScoutDriver &scout) = delete;

  bool setSpeed(const float &vx, const float &vth);
  bool setLightMode(LIGHT_MODE front_mode, uint8_t front_custom_value, LIGHT_MODE rear_mode, uint8_t rear_custom_value);
  
  ScoutStatus getScoutStatus() { std::lock_guard<std::mutex> lck(status_mtx_); return scout_status;};

protected:
  void run();
  inline void stop();
  bool readFrame(const uint8_t data);
  static uint8_t checkSum(uint8_t *data, uint8_t len);

private:
  bool readFlage_;
  std::mutex mtx_;
  std::mutex status_mtx_;
  serial::Serial com_;
  ScoutStatus scout_status;
  uint8_t RcvData[SCOUT_CMD_BUF_LEN] = {0,};
};

#endif // SCOUT_DRIVER_H