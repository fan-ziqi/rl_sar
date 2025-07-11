#ifndef SKYDROID_GAMEPAD_H
#define SKYDROID_GAMEPAD_H

#include <iostream>
#include <ostream>
#include "gamepad.h"

/**
 * @brief A specialized class for Skydroid gamepad.
 *
 * This class specializes the generic Gamepad class for Skydroid gamepad.
 */
class SkydroidGamepad : public Gamepad<SkydroidKeys> {
public:
  /**
   * @brief Constructor for the SkydroidGamepad class.
   *
   * Initializes the Skydroid gamepad with the specified UDP port for data reception.
   *
   * @param port The UDP port to use for receiving gamepad data.
   */
  SkydroidGamepad(int port = kDefaultPort);

  // /**
  //  * @brief Overloaded output stream operator for SkydroidGamepad.
  //  *
  //  * Allows printing SkydroidGamepad information to an output stream.
  //  *
  //  * @param o The output stream.
  //  * @param is The SkydroidGamepad object.
  //  * @return Reference to the output stream.
  //  */
  // friend std::ostream& operator<<(std::ostream& o, SkydroidGamepad& is);

protected:
  /**
   * @brief Updates the Skydroid gamepad data buffer and keys.
   *
   * @param buffer The data buffer to be updated.
   * @param keys The Skydroid gamepad keys to be updated.
   * @return True if the data is valid and updated, false otherwise.
   */
  bool UpdateData(std::vector<uint8_t>& buffer, SkydroidKeys& keys) override;

  /**
   * @brief Checks if the received data from the Skydroid gamepad is valid.
   *
   * @param data The gamepad data to be validated.
   * @return True if the data is valid, false otherwise.
   */
  bool DataIsValid(const SkydroidGamepadData& data);
};

#endif // SKYDROID_GAMEPAD_H
