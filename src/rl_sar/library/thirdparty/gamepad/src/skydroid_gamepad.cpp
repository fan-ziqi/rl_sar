#include "skydroid_gamepad.h"
#include <cstdlib>
#include <stdio.h>
#include <string.h>
#include <bitset>


/**
 * @brief Constructor for the SkydroidGamepad class.
 *
 * Initializes the Skydroid controller with the specified UDP port for data reception.
 *
 * @param port The UDP port to use for receiving controller data.
 */
SkydroidGamepad::SkydroidGamepad(int port) : Gamepad(port) {
  button_status_.clear();
  memset(&keys_, 0, sizeof(keys_));
}

/**
 * @brief Updates the Skydroid controller data buffer and keys.
 *
 * @param buffer The data buffer to be updated.
 * @param keys The Skydroid controller keys to be updated.
 * @return True if the data is valid and updated, false otherwise.
 */
bool SkydroidGamepad::UpdateData(std::vector<uint8_t>& buffer, SkydroidKeys& keys) {
  SkydroidGamepadData data;
  timespec timestamp;
  memcpy(&data, buffer.data(), buffer.size()*sizeof(uint8_t));
  // Perform data validity check in the child class
  if (DataIsValid(data)) {
    std::bitset<kSkydroidButtonSize> keys_value_bit(0);
    int16_t keys_ch[kSkydroidButtonSize];
    memcpy(keys_ch, data.buttons, sizeof(keys_ch));
    for(int i = 0; i < kSkydroidButtonSize; i++){
      keys_value_bit[i] = keys_ch[i];
    }
    clock_gettime(CLOCK_MONOTONIC,&timestamp);
    keys.time_stamp = (timestamp.tv_sec-start_time_.tv_sec)*1.e3 + double(timestamp.tv_nsec - start_time_.tv_nsec)/1.e6;
    keys.keys_value = keys_value_bit.to_ulong();
    keys.left_axis_x = data.left_axis_x/(float)kJoystickRange;
    keys.left_axis_y = data.left_axis_y/(float)kJoystickRange;
    keys.right_axis_x = data.right_axis_x/(float)kJoystickRange;
    keys.right_axis_y = data.right_axis_y/(float)kJoystickRange;

    keys.sw1 = data.switch_keys[0];
    keys.sw2 = data.switch_keys[1];
    keys.sw3 = data.switch_keys[2];
    keys.sw4 = data.switch_keys[3];

    return true;
  }
  return false;
}

/**
 * @brief Checks if the received data from the Skydroid controller is valid.
 *
 * @param data The controller data to be validated.
 * @return True if the data is valid, false otherwise.
 */
bool SkydroidGamepad::DataIsValid(const SkydroidGamepadData& data) {

  // 1. check STX   
  if (data.stx[0] != kHeader[0] || data.stx[1] != kHeader[1]) {
    return false;
  }

  // 2. check device ID
  if (data.id != (uint8_t)GamepadType::kSkydroid) {
    return false;
  }
  
  // 3. CRC16
  if(data.crc16 != CalculateCrc16(data.data, data.data_len)){
    return false;
  }

  return true;  
}


