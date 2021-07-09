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

#include <boost/format.hpp>
#include <serial/serial.h>
#include <iostream>
#include <sstream>
#include <thread>
#include <string>
#include <vector>
#include <mutex>

#include "QThread.h"

#define SOF_HEAD_1        0x5A
#define SOF_HEAD_2        0xA5

#define MAX_X_SPEED       1.5
#define MAX_TH_SPEED      0.5235
#define WHEEL_RADIUS      0.16459
#define WHEEL_LENGTH      0.582
#define WHEEL_BASE        0.498
#define REDUCTION_RATIO   40.0
#define ENCODER_LINES     2500.0

#define SCOUT_CMD_BUF_LEN    32

class ScoutDriver : public QThread
{
public:
  typedef struct { //cmd_id:0xa5
    uint8_t soc;
    uint8_t soh;
    float voltage;
    float current;
    float temperature;
  }ScoutBMS;

  typedef struct { //cmd_id:0xa4
    uint8_t swa;
    uint8_t swb;
    uint8_t swc;
    uint8_t swd;
    float right_rl_axis;
    float right_ud_axis;
    float left_ud_axis;
    float left_rl_axis;
    float left_vra;
  }ScoutTeleop;

  typedef struct { //cmd_id:0xa3
    std::string controller_hw_version;
    std::string controller_sw_version;
    std::string driver_hw_version;
    std::string driver_sw_version;
  }ScoutVersion;

  typedef struct { //cmd_id:0xa2
    float left_odom;  // m
    float right_odom; // m
  }ScoutOdom;

  typedef struct { //cmd_id:0xa1
    bool enable;
    uint8_t front_light_mode;
    uint8_t front_light_brightness;
    uint8_t rear_light_mode;
    uint8_t rear_light_brightness;
  }ScoutLightState;

  typedef struct { //cmd_id:0x07---0x0a
    uint8_t id;
    float driver_voltage;
    float driver_temperature;
    float motor_temperature;
    uint8_t driver_state;
  }ScoutDriverState;

  typedef struct ScoutMotorState{ //cmd_id:0x03---0x06
    uint8_t id;
    float rpm;
    float current;
    int32_t encoder;
    double motor_pose;
    ScoutMotorState() : motor_pose(0) {}
  }ScoutMotorState;

  typedef struct {
    uint8_t base_state;   // cmd_id:0x01
    uint8_t control_mode;
    float battery_voltage;
    uint8_t error_code;

    float vx; // cmd_id:0x02
    float vth;

    std::array<ScoutMotorState, 4> motor_states;    //cmd_id:0x03---0x06
    std::array<ScoutDriverState, 4> driver_states;  //cmd_id:0x07---0x0a
    ScoutLightState light_state;                    //cmd_id:0xa1
    ScoutOdom odom;                                 //cmd_id:0xa2
    ScoutVersion version;                           //cmd_id:0xa3
    ScoutTeleop teleop;                             //cmd_id:0xa4
    ScoutBMS bms;                                   //cmd_id:0xa5
  }ScoutStatus;

public:
  ScoutDriver(const std::string &portName);
  ~ScoutDriver();

  // do not allow copy
  ScoutDriver(const ScoutDriver &scout) = delete;
  ScoutDriver &operator=(const ScoutDriver &scout) = delete;

  bool setSpeed(const float &vx, const float &vth);
  bool setLightMode(uint8_t front_mode, uint8_t front_custom_value, uint8_t rear_mode, uint8_t rear_custom_value);
  
  ScoutStatus getScoutStatus() { std::lock_guard<std::mutex> lck(status_mtx_); return scout_status;};

protected:
  void run();
  void stop();
  bool readFrame(const uint8_t data);
  static uint8_t checkSum(uint8_t *data, uint8_t len);
  bool setSerialEnable();

private:
  bool readFlage_;
  std::mutex mtx_;
  std::mutex status_mtx_;
  serial::Serial com_;
  ScoutStatus scout_status;
  uint8_t RcvData[SCOUT_CMD_BUF_LEN] = {0,};
  inline bool checkSerial();
};

#endif // SCOUT_DRIVER_H