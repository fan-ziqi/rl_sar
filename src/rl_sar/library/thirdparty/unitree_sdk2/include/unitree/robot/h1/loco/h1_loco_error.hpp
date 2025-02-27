#ifndef __UT_ROBOT_H1_LOCO_ERROR_HPP__
#define __UT_ROBOT_H1_LOCO_ERROR_HPP__

#include <unitree/common/decl.hpp>

namespace unitree {
namespace robot {
namespace h1 {
UT_DECL_ERR(UT_ROBOT_LOCO_ERR_LOCOSTATE_NOT_AVAILABLE, 8301,
            "LocoState not available.")
UT_DECL_ERR(UT_ROBOT_LOCO_ERR_INVALID_FSM_ID, 8302, "Invalid fsm id.")
} // namespace h1
} // namespace robot
} // namespace unitree

#endif // __UT_ROBOT_H1_LOCO_ERROR_HPP__
