/**
 * @file example_retroid.cpp
 * @brief 
 * @author mazunwang
 * @version 1.0
 * @date 2024-03-27
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */

#include <iostream>

#include "gamepad.h"
#include "retroid_gamepad.h"
#include "gamepad_keys.h"

#include "fmt/format.h"
#include "fmt/core.h"
#include "fmt/chrono.h"

void InitialRetroidKeys(RetroidGamepad& is){
  is.button_status_.clear();
  std::string pressed  = fmt::format("\x1b[32m{:^20}\x1b[0m", "pressed");
  std::string released = fmt::format("\x1b[37m{:^20}\x1b[0m", "released");
  is.button_status_.push_back(released);
  is.button_status_.push_back(pressed);
}

/**
 * @brief Overloaded output stream operator for RetroidGamepad.
 *
 * Allows printing RetroidGamepad information to an output stream.
 *
 * @param o The output stream.
 * @param is The RetroidGamepad object.
 * @return Reference to the output stream.
 */
std::ostream& operator<<(std::ostream& o, RetroidGamepad& is){
  RetroidKeys keys = is.GetKeys();
  o << "\033c";
  std::string  s = fmt::format(
	"┌{0:─^{1}}┐\n"
	"│{2: ^{1}}│\n"
	"└{0:─^{1}}┘\n", "", 86 ,"Joystick Device: Retroid(Lite3)");
  o<<s ;
  s = fmt::format(
    "┌{2:─^{1}}┐┌{3:─^{1}}┐┌{4:─^{1}}┐┌{5:─^{1}}┐\n"
    "│{6: ^{1}}││{7: ^{1}}││{8: ^{1}}││{9: ^{1}}│\n"
    "└{0:─^{1}}┘└{0:─^{1}}┘└{0:─^{1}}┘└{0:─^{1}}┘\n", 
    "", 20, "L2", "L1", "R1", "R2", 
    is.button_status_[keys.L2], is.button_status_[keys.L1], is.button_status_[keys.R1], is.button_status_[keys.R2]);
  o<<s ;
  s = fmt::format(
    "┌{2:─^{1}}┐┌{3:─^{1}}┐┌{4:─^{1}}┐┌{5:─^{1}}┐\n"
    "│{6: ^{1}}││{7: ^{1}}││{8: ^{1}}││{9: ^{1}}│\n"
    "└{0:─^{1}}┘└{0:─^{1}}┘└{0:─^{1}}┘└{0:─^{1}}┘\n", 
    "", 20, "X", "Y", "A", "B", 
    is.button_status_[keys.X], is.button_status_[keys.Y], is.button_status_[keys.A], is.button_status_[keys.B]);
  o<<s ;

  s = fmt::format(
    "┌{2:─^{1}}┐┌{3:─^{1}}┐┌{4:─^{1}}┐┌{5:─^{1}}┐\n"
    "│{6: ^{1}}││{7: ^{1}}││{8: ^{1}}││{9: ^{1}}│\n"
    "└{0:─^{1}}┘└{0:─^{1}}┘└{0:─^{1}}┘└{0:─^{1}}┘\n",
    "", 20, "up", "down", "left", "right", 
    is.button_status_[keys.up], is.button_status_[keys.down], is.button_status_[keys.left], is.button_status_[keys.right]);
  o<<s ;



  s = fmt::format(
    "┌{2:─^{1}}┐┌{3:─^{1}}┐┌{4:─^{1}}┐┌{5:─^{1}}┐\n"
    "│{6: ^{1}}││{7: ^{1}}││{8: ^{1}}││{9: ^{1}}│\n"
    "└{0:─^{1}}┘└{0:─^{1}}┘└{0:─^{1}}┘└{0:─^{1}}┘\n", 
    "", 20, "select", "start", "left_axis_button", "right_axis_button", 
    is.button_status_[keys.select], is.button_status_[keys.start], is.button_status_[keys.left_axis_button], is.button_status_[keys.right_axis_button]);
  // fmt::print(s);
  o<<s ;
  s = fmt::format(
    "┌{2:─^{1}}┐┌{3:─^{1}}┐┌{4:─^{1}}┐┌{5:─^{1}}┐\n"
    "│{6: ^{1}}││{7: ^{1}}││{8: ^{1}}││{9: ^{1}}│\n"
    "└{0:─^{1}}┘└{0:─^{1}}┘└{0:─^{1}}┘└{0:─^{1}}┘\n", 
    "", 20, "left_axis_x", "left_axis_y", "right_axis_x", "right_axis_y", 
    keys.left_axis_x, keys.left_axis_y, keys.right_axis_x, keys.right_axis_y);
  o<<s ;
  // o<<str_button ;
  return o;
}


int main(int argc, char* argv[]) {
  RetroidGamepad rc(12121);
  InitialRetroidKeys(rc);

  rc.StartDataThread();

  rc.updateCallback_ = [&](int32_t count){
    std::cout << rc << std::endl;
  };
    
  for(int i = 0; i < 100000; i ++){
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  return 0;
}
