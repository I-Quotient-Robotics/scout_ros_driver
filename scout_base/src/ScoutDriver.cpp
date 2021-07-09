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

#include "ScoutDriver.h"

ScoutDriver::ScoutDriver(const std::string &portName)
  : readFlage_(false)
{

  try
  {
    com_.setPort(portName);
    com_.setBaudrate(115200);
    com_.setBytesize(serial::eightbits);
    com_.setParity(serial::parity_none);
    com_.setStopbits(serial::stopbits_one);
    com_.setFlowcontrol(serial::flowcontrol_none);
    serial::Timeout serial_timeout = serial::Timeout::simpleTimeout(50);
    com_.setTimeout(serial_timeout);
    com_.open();
    //com_.setRTS(false);
    //com_.setDTR(false);
  }
  catch (serial::IOException &e)
  {
    std::cout << "Unable to open serial port:" << portName << std::endl;
    return;
  }

  std::cout << "open serial port:" << portName << " successful!!"<< std::endl;

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  //set serial enable model
  setSerialEnable();
  //start thread
  start();
}

ScoutDriver::~ScoutDriver()
{
  stop();
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  com_.close();
}

bool ScoutDriver::setSerialEnable()
{
  std::lock_guard<std::mutex> lck(mtx_);
  if (!checkSerial())
  {
    return false;
  }

  static uint8_t frame_id = 0x00;
  uint8_t sendBuf[SCOUT_CMD_BUF_LEN] = {
    0,
  };

  sendBuf[0] = SOF_HEAD_1;  // SOF 1
  sendBuf[1] = SOF_HEAD_2;  // SOF 2
  sendBuf[2] = 0x05;        // frame_len
  sendBuf[3] = 0x55;        // CMD_TYPE
  sendBuf[4] = 0x02;        // CMD_ID
  sendBuf[5] = 0x02;        // 0x02 enable serial
  sendBuf[6] = frame_id;    // frame_id
  sendBuf[7] = checkSum(sendBuf, 12);  // check_sum
  
  size_t bufLen = sendBuf[2] + 3;
  size_t writeLen = com_.write(sendBuf, bufLen);
  if (bufLen != writeLen)
  {
    std::cout << "Failed to send message!!" << std::endl;
    com_.flushOutput();
    return false;
  }
  frame_id++;
  return true;
}

bool ScoutDriver::setSpeed(const float &vx, const float &vth)
{
  std::lock_guard<std::mutex> lck(mtx_);
  if (!checkSerial())
  {
    return false;
  }

  static uint8_t frame_id = 0x00;
  uint8_t sendBuf[SCOUT_CMD_BUF_LEN] = {
      0,
  };

  sendBuf[0] = SOF_HEAD_1;  // SOF 1
  sendBuf[1] = SOF_HEAD_2;  // SOF 2
  sendBuf[2] = 0x0a;        // frame_len
  sendBuf[3] = 0x55;        // CMD_TYPE
  sendBuf[4] = 0x01;        // CMD_ID

  int16_t vx_mm = int16_t(vx * 1000.0);     // mm/s
  int16_t vth_mrad = int16_t(vth * 1000.0); // 0.001rad/s

  //std::cout << vth_mrad << std::endl;
  
  sendBuf[5] = uint8_t((vx_mm & 0xff00) >> 8);
  sendBuf[6] = uint8_t(vx_mm & 0x00ff);

  sendBuf[7] = uint8_t((vth_mrad & 0xff00) >> 8);
  sendBuf[8] = uint8_t(vth_mrad & 0x00ff);

  sendBuf[9] = 0x00;
  sendBuf[10] = 0x00;

  sendBuf[11] = frame_id;                 // frame_id
  sendBuf[12] = checkSum(sendBuf, 12);    // check_sum

  size_t bufLen = sendBuf[2] + 3;
  size_t writeLen = com_.write(sendBuf, bufLen);
  if (bufLen != writeLen)
  {
    std::cout << "Failed to send message!!" << std::endl;
    com_.flushOutput();
    return false;
  }
  frame_id++;
  return true;
}
  
bool ScoutDriver::setLightMode(uint8_t front_mode, uint8_t front_custom_value, uint8_t rear_mode, uint8_t rear_custom_value)
{
  std::lock_guard<std::mutex> lck(mtx_);
  if (!checkSerial())
  {
    return false;
  }

  static uint8_t frame_id = 0x00;
  uint8_t sendBuf[SCOUT_CMD_BUF_LEN] = {
      0,
  };

  sendBuf[0] = SOF_HEAD_1;  // SOF 1
  sendBuf[1] = SOF_HEAD_2;  // SOF 2
  sendBuf[2] = 0x0a;        // frame_len
  sendBuf[3] = 0x55;        // CMD_TYPE
  sendBuf[4] = 0x04;        // CMD_ID

  sendBuf[5] = 0x01;                // light control enable
  sendBuf[6] = front_mode;          // front light mode
  sendBuf[7] = front_custom_value;  // front light custom velue
  sendBuf[8] = rear_mode;           // rear light mode
  sendBuf[9] = rear_custom_value;   // rear light custom velue
  sendBuf[10] = 0x00;               // null

  sendBuf[11] = frame_id;               // frame_id
  sendBuf[12] = checkSum(sendBuf, 12);  // check_sum

  size_t bufLen = sendBuf[2] + 3;
  size_t writeLen = com_.write(sendBuf, bufLen);
  if (bufLen != writeLen)
  {
    std::cout << "Failed to send message!!" << std::endl;
    com_.flushOutput();
    return false;
  }
  frame_id++;
  return true;
}

void ScoutDriver::run()
{
  readFlage_ = true;
  uint8_t buf[256] = {
    0,
  };
  size_t date_len = 0;
  while (readFlage_)
  {
    mtx_.lock();
    date_len = com_.read(buf, 256);
    //std::cout << date_len << "\n";
    for (size_t i = 0; i < date_len; i++)
    {
      if (readFrame(buf[i]))
      {
        status_mtx_.lock();
        switch (RcvData[0])
        {
        case 0x01:              //scout base state return msg
          scout_status.base_state = RcvData[1];
          scout_status.control_mode = RcvData[2];
          scout_status.battery_voltage = float(uint16_t((RcvData[3] << 8) | RcvData[4]) / 10.0);
          scout_status.error_code = RcvData[5];
          break;
        case 0x02:              //scout cmd msg return msg
          scout_status.vx = float(int16_t((RcvData[1] << 8) | RcvData[2]) / 1000.0);
          scout_status.vth = float(int16_t((RcvData[3] << 8) | RcvData[4]) / 1000.0);
          break;
        case 0x03:              //scout motor 1 state return msg, right
          scout_status.motor_states[0].id = 0x01;
          scout_status.motor_states[0].rpm = float(int16_t((RcvData[1] << 8) | RcvData[2]) * -1.0);
          scout_status.motor_states[0].current = float(uint16_t((RcvData[3] << 8) | RcvData[4]) / 10.0);
          scout_status.motor_states[0].encoder = -int32_t((RcvData[5] << 24) | (RcvData[6] << 16) | (RcvData[7] << 8) | RcvData[8]);
          scout_status.motor_states[0].motor_pose = double(scout_status.motor_states[0].encoder / (ENCODER_LINES * REDUCTION_RATIO)) * 2 * 3.1415926;
          break;
        case 0x04:              //scout motor 2 state return msg, left
          scout_status.motor_states[1].id = 0x02;
          scout_status.motor_states[1].rpm = float(int16_t((RcvData[1] << 8) | RcvData[2]) * -1.0);
          scout_status.motor_states[1].current = float(uint16_t((RcvData[3] << 8) | RcvData[4]) / 10.0);
          scout_status.motor_states[1].encoder = -int32_t((RcvData[5] << 24) | (RcvData[6] << 16) | (RcvData[7] << 8) | RcvData[8]);
          scout_status.motor_states[1].motor_pose = double(scout_status.motor_states[1].encoder / (ENCODER_LINES * REDUCTION_RATIO)) * 2 * 3.1415926;
          break;
        case 0x05:              //scout motor 3 state return msg, left
          scout_status.motor_states[2].id = 0x03;
          scout_status.motor_states[2].rpm = float(int16_t((RcvData[1] << 8) | RcvData[2]));
          scout_status.motor_states[2].current = float(uint16_t((RcvData[3] << 8) | RcvData[4]) / 10.0);
          scout_status.motor_states[2].encoder = int32_t((RcvData[5] << 24) | (RcvData[6] << 16) | (RcvData[7] << 8) | RcvData[8]);
          scout_status.motor_states[2].motor_pose = double(scout_status.motor_states[2].encoder / (ENCODER_LINES * REDUCTION_RATIO)) * 2 * 3.1415926;
          break;
        case 0x06:              //scout motor 4 state return msg, right
          scout_status.motor_states[3].id = 0x04;
          scout_status.motor_states[3].rpm = float(int16_t((RcvData[1] << 8) | RcvData[2]));
          scout_status.motor_states[3].current = float(uint16_t((RcvData[3] << 8) | RcvData[4]) / 10.0);
          scout_status.motor_states[3].encoder = int32_t((RcvData[5] << 24) | (RcvData[6] << 16) | (RcvData[7] << 8) | RcvData[8]);
          scout_status.motor_states[3].motor_pose = double(scout_status.motor_states[3].encoder / (ENCODER_LINES * REDUCTION_RATIO)) * 2 * 3.1415926;
          break;
        case 0x07:              //scout motor 1 state return msg
          scout_status.driver_states[0].id = 0x01;
          scout_status.driver_states[0].driver_voltage = float(uint16_t((RcvData[1] << 8) | RcvData[2]) / 10.0);
          scout_status.driver_states[0].driver_temperature = float(uint16_t((RcvData[3] << 8) | RcvData[4]));
          scout_status.driver_states[0].motor_temperature = RcvData[5];
          scout_status.driver_states[0].driver_state = RcvData[6];
          break;
        case 0x08:              //scout motor 2 state return msg
          scout_status.driver_states[1].id = 0x02;
          scout_status.driver_states[1].driver_voltage = float(uint16_t((RcvData[1] << 8) | RcvData[2]) / 10.0);
          scout_status.driver_states[1].driver_temperature = float(uint16_t((RcvData[3] << 8) | RcvData[4]));
          scout_status.driver_states[1].motor_temperature = RcvData[5];
          scout_status.driver_states[1].driver_state = RcvData[6];
          break;
        case 0x09:              //scout motor 3 state return msg
          scout_status.driver_states[2].id = 0x03;
          scout_status.driver_states[2].driver_voltage = float(uint16_t((RcvData[1] << 8) | RcvData[2]) / 10.0);
          scout_status.driver_states[2].driver_temperature = float(uint16_t((RcvData[3] << 8) | RcvData[4]));
          scout_status.driver_states[2].motor_temperature = RcvData[5];
          scout_status.driver_states[2].driver_state = RcvData[6];
          break;
        case 0x0a:              //scout motor 4 state return msg
          scout_status.driver_states[3].id = 0x04;
          scout_status.driver_states[3].driver_voltage = float(uint16_t((RcvData[1] << 8) | RcvData[2]) / 10.0);
          scout_status.driver_states[3].driver_temperature = float(uint16_t((RcvData[3] << 8) | RcvData[4]));
          scout_status.driver_states[3].motor_temperature = RcvData[5];
          scout_status.driver_states[3].driver_state = RcvData[6];
          break;
        case 0xa1:              //scout light state
          scout_status.light_state.enable = (RcvData[1]) ? true : false;
          scout_status.light_state.front_light_mode = RcvData[2];
          scout_status.light_state.front_light_brightness = RcvData[3];
          scout_status.light_state.rear_light_mode = RcvData[4];
          scout_status.light_state.rear_light_brightness = RcvData[5];
          break;
        case 0xa2:              //scout odom
          scout_status.odom.left_odom = float(int32_t((RcvData[1] << 24) | (RcvData[2] << 16) | (RcvData[3] << 8) | RcvData[4]) / 1000.0);
          scout_status.odom.right_odom = float(int32_t((RcvData[5] << 24) | (RcvData[6] << 16) | (RcvData[7] << 8) | RcvData[8]) / 1000.0);
          break;
        case 0xa3:              //scout version
          scout_status.version.controller_hw_version = str(boost::format("v%1%.%2%") % int(RcvData[1]) % int(RcvData[2]));
          scout_status.version.driver_hw_version = str(boost::format("v%1%.%2%") % int(RcvData[3]) % int(RcvData[4]));
          scout_status.version.controller_sw_version = str(boost::format("v%1%.%2%") % int(RcvData[5]) % int(RcvData[6]));
          scout_status.version.driver_sw_version = str(boost::format("v%1%.%2%") % int(RcvData[7]) % int(RcvData[8]));
          break;
        case 0xa4:              //scout teleop
          scout_status.teleop.swa = uint8_t(RcvData[1] & 0b00000011);
          scout_status.teleop.swb = uint8_t((RcvData[1] & 0b00001100) >> 2);
          scout_status.teleop.swc = uint8_t((RcvData[1] & 0b00110000) >> 4);
          scout_status.teleop.swd = uint8_t((RcvData[1] & 0b11000000) >> 6);
          scout_status.teleop.right_rl_axis = float(int8_t(RcvData[2]) / 100.0);
          scout_status.teleop.right_ud_axis = float(int8_t(RcvData[3]) / 100.0);
          scout_status.teleop.left_ud_axis = float(int8_t(RcvData[4]) / 100.0);
          scout_status.teleop.left_rl_axis = float(int8_t(RcvData[5]) / 100.0);
          scout_status.teleop.left_vra = float(int8_t(RcvData[6]) / 100.0);
          break;
        case 0xa5:              //scout bms
          scout_status.bms.soc = RcvData[1];
          scout_status.bms.soh = RcvData[2];
          scout_status.bms.voltage = float(uint16_t((RcvData[3] << 8) | RcvData[4]) / 10.0);
          scout_status.bms.current = float(int16_t((RcvData[5] << 8) | RcvData[6]) / 10.0);
          scout_status.bms.temperature = float(int16_t((RcvData[7] << 8) | RcvData[8]) / 10.0);
          break;                                                  
        default:
          std::cout << "cmd type error!" << std::endl;
          break;
        }
        status_mtx_.unlock();
      }
    }
    mtx_.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

bool ScoutDriver::readFrame(const uint8_t data)
{
  static uint8_t rcvBuf_[32] = {0,};
  static uint32_t frameLength_ = 0;
  static uint32_t rcvIndex_ = 0;

  rcvBuf_[rcvIndex_] = data;
  rcvIndex_++;

  if(rcvIndex_ < 5)
  {
    if(rcvIndex_ == 1 && rcvBuf_[0] == SOF_HEAD_1)      //SOF 1
    {
      return false;
    }
    else if(rcvIndex_ == 2 && rcvBuf_[1] == SOF_HEAD_2) //SOF 2
    {
      return false;
    }
    else if(rcvIndex_ == 3 && rcvBuf_[2] > 0)           //frame_Len
    {
      frameLength_ = data + 3;
    }
    else if(rcvIndex_ == 4 && rcvBuf_[3] == 0xAA)       //CMD_TYPE
    {
      return false;
    }
    else
    {
      rcvIndex_ = 0;
      frameLength_ = 0;
      return false;
    }
  }
  else
  {
    if(rcvIndex_ < frameLength_)
    {
      RcvData[rcvIndex_ - 5] = data;
      return false;
    }
    else
    {
      if(data == checkSum(rcvBuf_, (frameLength_-1)))
      {
        rcvIndex_ = 0;
        frameLength_ = 0;
        return true;
      }
      else
      {
        rcvIndex_ = 0;
        frameLength_ = 0;
        return false;
      }
    }
  }
  return false;
}

void ScoutDriver::stop()
{
  std::lock_guard<std::mutex> lck(mtx_);
  readFlage_ = false;
  //join();
}

uint8_t ScoutDriver::checkSum(uint8_t *data, uint8_t len)
{
  uint8_t checksum = 0x00;
  for(uint8_t i = 0 ; i < len; i++)
  {
    checksum += data[i];
  }
  return checksum;
}

bool ScoutDriver::checkSerial()
{
  if (!com_.isOpen())
  {
    std::cout << "serial not open!!" << std::endl;
    return false;
  }

  com_.flushOutput();
  return true;
}