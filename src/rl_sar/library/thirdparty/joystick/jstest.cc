#include <unistd.h>
#include <iostream>
#include <map>
#include "joystick.h"

#define GAMEPAD_TYPE 1 // 1: XBOX, 0: SWITCH
#define MAX_AXES_VALUE 32768
#define MIN_AXES_VALUE -32768
using namespace std;

typedef union
{
  struct
  {
    uint8_t R1 : 1;
    uint8_t L1 : 1;
    uint8_t start : 1;
    uint8_t select : 1;
    uint8_t R2 : 1;
    uint8_t L2 : 1;
    uint8_t F1 : 1;
    uint8_t F2 : 1;
    uint8_t A : 1;
    uint8_t B : 1;
    uint8_t X : 1;
    uint8_t Y : 1;
    uint8_t up : 1;
    uint8_t right : 1;
    uint8_t down : 1;
    uint8_t left : 1;
  } components;
  uint16_t value;
} xKeySwitchUnion;

int main(int argc, char **argv)
{
  // Create an instance of Joystick
  Joystick joystick("/dev/input/js0");

  // Ensure that it was found and that we can use it
  if (!joystick.isFound())
  {
    printf("open failed.\n");
    exit(1);
  }

  xKeySwitchUnion unitree_key;
  map<string, int> AxisId =
      {
          {"LX", 0}, // Left stick axis x
          {"LY", 1}, // Left stick axis y
          {"RX", 3}, // Right stick axis x
          {"RY", 4}, // Right stick axis y
          {"LT", 2}, // Left trigger
          {"RT", 5}, // Right trigger
          {"DX", 6}, // Directional pad x
          {"DY", 7}, // Directional pad y
      };

  map<string, int> ButtonId =
      {
          {"X", 2},
          {"Y", 3},
          {"B", 1},
          {"A", 0},
          {"LB", 4},
          {"RB", 5},
          {"SELECT", 6},
          {"START", 7},
      };

  while (true)
  {

    // Attempt to sample an event from the joystick
    joystick.getState();

    unitree_key.components.R1 = joystick.button_[ButtonId["RB"]];
    unitree_key.components.L1 = joystick.button_[ButtonId["LB"]];
    unitree_key.components.start = joystick.button_[ButtonId["START"]];
    unitree_key.components.select = joystick.button_[ButtonId["SELECT"]];
    unitree_key.components.R2 = (joystick.axis_[AxisId["RT"]] > 0);
    unitree_key.components.L2 = (joystick.axis_[AxisId["LT"]] > 0);
    unitree_key.components.F1 = 0;
    unitree_key.components.F2 = 0;
    unitree_key.components.A = joystick.button_[ButtonId["A"]];
    unitree_key.components.B = joystick.button_[ButtonId["B"]];
    unitree_key.components.X = joystick.button_[ButtonId["X"]];
    unitree_key.components.Y = joystick.button_[ButtonId["Y"]];
    unitree_key.components.up = (joystick.axis_[AxisId["DY"]] < 0);
    unitree_key.components.right = (joystick.axis_[AxisId["DX"]] > 0);
    unitree_key.components.down = (joystick.axis_[AxisId["DY"]] > 0);
    unitree_key.components.left = (joystick.axis_[AxisId["DX"]] < 0);

    cout << unitree_key.value << endl;

    // Restrict rate
    usleep(10000);
  }
  return 0;
};
