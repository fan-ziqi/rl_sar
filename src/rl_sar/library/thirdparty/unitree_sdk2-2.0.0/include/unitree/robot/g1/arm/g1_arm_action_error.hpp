#pragma once

#include <unitree/common/decl.hpp>

namespace unitree {
namespace robot {
namespace g1 {

UT_DECL_ERR(UT_ROBOT_ARM_ACTION_ERR_ARMSDK, 7400, "armsdk is occupied.")
UT_DECL_ERR(UT_ROBOT_ARM_ACTION_ERR_HOLDING, 7401, "The arm is holding. Only release is allowed.")
UT_DECL_ERR(UT_ROBOT_ARM_ACTION_ERR_INVALID_ACTION_ID, 7402, "Invalid action id.")
UT_DECL_ERR(UT_ROBOT_ARM_ACTION_ERR_INVALID_FSM_ID, 7404, "Invalid fsm id.")

}  // namespace g1
}  // namespace robot
}  // namespace unitree