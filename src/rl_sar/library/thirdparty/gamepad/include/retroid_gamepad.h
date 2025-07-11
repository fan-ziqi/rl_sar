#ifndef RETROID_GAMEPAD_H
#define RETROID_GAMEPAD_H

#include <iostream>
#include <ostream>
#include "gamepad.h"

/**
 * @brief A specialized class for RETROID gamepad.
 *
 * This class specializes the generic Gamepad class for RETROID gamepad.
 */
class RetroidGamepad : public Gamepad<RetroidKeys> {
public:
  /**
   * @brief Constructor for the RetroidGamepad class.
   *
   * Initializes the Retroid gamepad with the specified UDP port for data reception.
   *
   * @param port The UDP port to use for receiving gamepad data.
   */
  RetroidGamepad(int port = kDefaultPort);

  // /**
  //  * @brief Overloaded output stream operator for RetroidGamepad.
  //  *
  //  * Allows printing RetroidGamepad information to an output stream.
  //  *
  //  * @param o The output stream.
  //  * @param is The RetroidGamepad object.
  //  * @return Reference to the output stream.
  //  */
  // friend std::ostream& operator<<(std::ostream& o, RetroidGamepad& is);

protected:
  /**
   * @brief Updates the RETROID gamepad data buffer and keys.
   *
   * @param buffer The data buffer to be updated.
   * @param keys The RETROID gamepad keys to be updated.
   * @return True if the data is valid and updated, false otherwise.
   */
  bool UpdateData(std::vector<uint8_t>& buffer, RetroidKeys& keys) override;

  /**
   * @brief Checks if the received data from the RETROID gamepad is valid.
   *
   * @param data The gamepad data to be validated.
   * @return True if the data is valid, false otherwise.
   */
  bool DataIsValid(const RetroidGamepadData& data);
};

#endif // RETROID_GAMEPAD_H
