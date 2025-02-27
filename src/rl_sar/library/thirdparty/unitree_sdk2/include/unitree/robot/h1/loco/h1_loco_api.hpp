#ifndef __UT_ROBOT_H1_LOCO_API_HPP__
#define __UT_ROBOT_H1_LOCO_API_HPP__

#include <unitree/common/json/jsonize.hpp>
#include <variant>

namespace unitree {
namespace robot {
namespace h1 {
/*service name*/
const std::string LOCO_SERVICE_NAME = "loco";

/*api version*/
const std::string LOCO_API_VERSION = "2.0.0.0";

/*api id*/
const int32_t ROBOT_API_ID_LOCO_GET_FSM_ID = 8001;
const int32_t ROBOT_API_ID_LOCO_GET_FSM_MODE = 8002;
const int32_t ROBOT_API_ID_LOCO_GET_BALANCE_MODE = 8003;
const int32_t ROBOT_API_ID_LOCO_GET_SWING_HEIGHT = 8004;
const int32_t ROBOT_API_ID_LOCO_GET_STAND_HEIGHT = 8005;
const int32_t ROBOT_API_ID_LOCO_GET_PHASE = 8006; // deprecated

const int32_t ROBOT_API_ID_LOCO_SET_FSM_ID = 8101;
const int32_t ROBOT_API_ID_LOCO_SET_BALANCE_MODE = 8102;
const int32_t ROBOT_API_ID_LOCO_SET_SWING_HEIGHT = 8103;
const int32_t ROBOT_API_ID_LOCO_SET_STAND_HEIGHT = 8104;
const int32_t ROBOT_API_ID_LOCO_SET_VELOCITY = 8105;
const int32_t ROBOT_API_ID_LOCO_SET_PHASE = 8106;

using LocoCmd =
    std::map<std::string, std::variant<int, float, std::vector<float>>>;

class JsonizeDataVecFloat : public common::Jsonize {
public:
  JsonizeDataVecFloat() {}
  ~JsonizeDataVecFloat() {}

  void fromJson(common::JsonMap &json) { common::FromJson(json["data"], data); }

  void toJson(common::JsonMap &json) const {
    common::ToJson(data, json["data"]);
  }

  std::vector<float> data;
};

class JsonizeVelocityCommand : public common::Jsonize {
public:
  JsonizeVelocityCommand() {}
  ~JsonizeVelocityCommand() {}

  void fromJson(common::JsonMap &json) {
    common::FromJson(json["velocity"], velocity);
    common::FromJson(json["duration"], duration);
  }

  void toJson(common::JsonMap &json) const {
    common::ToJson(velocity, json["velocity"]);
    common::ToJson(duration, json["duration"]);
  }

  std::vector<float> velocity;
  float duration;
};

} // namespace h1
} // namespace robot
} // namespace unitree

#endif // __UT_ROBOT_H1_LOCO_API_HPP__
