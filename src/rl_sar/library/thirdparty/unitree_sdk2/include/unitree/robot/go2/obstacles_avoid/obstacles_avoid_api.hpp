#ifndef __UT_ROBOT_GO2_OBSTACLES_AVOID_API_HPP__
#define __UT_ROBOT_GO2_OBSTACLES_AVOID_API_HPP__

#include <unitree/common/json/jsonize.hpp>

namespace unitree
{
namespace robot
{
namespace go2
{
const std::string ROBOT_OBSTACLES_AVOID_SERVICE_NAME = "obstacles_avoid";
const std::string ROBOT_OBSTACLES_AVOID_API_VERSION = "1.0.0.1";

const int32_t ROBOT_API_ID_OBSTACLES_AVOID_SWITCH_SET = 1001;
const int32_t ROBOT_API_ID_OBSTACLES_AVOID_SWITCH_GET = 1002;

class ObstaclesAvoidSwitchSetParameter : public common::Jsonize
{
public:
    void fromJson(common::JsonMap& json)
    {
        common::FromJson(json["enable"], mEnable);
    }

    void toJson(common::JsonMap& json) const
    {
        common::ToJson(mEnable, json["enable"]);
    }

    bool mEnable = true;
};

class ObstaclesAvoidSwitchGetData : public common::Jsonize
{
public:
    void fromJson(common::JsonMap& json)
    {
        common::FromJson(json["enable"], mEnable);
    }

    void toJson(common::JsonMap& json) const
    {
        common::ToJson(mEnable, json["enable"]);
    }

    bool mEnable = true;
};

}
}
}

#endif//__UT_ROBOT_GO2_OBSTACLES_AVOID_API_HPP__
