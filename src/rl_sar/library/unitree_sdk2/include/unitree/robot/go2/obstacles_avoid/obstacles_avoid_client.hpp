#ifndef __UT_ROBOT_GO2_OBSTACLES_AVOID_CLIENT_HPP__
#define __UT_ROBOT_GO2_OBSTACLES_AVOID_CLIENT_HPP__

#include <unitree/robot/client/client.hpp>

namespace unitree
{
namespace robot
{
namespace go2
{
class ObstaclesAvoidClient : public Client
{
 public:
     ObstaclesAvoidClient();
     ~ObstaclesAvoidClient();

     void Init();

     int32_t SwitchSet(bool enable);
     int32_t SwitchGet(bool& enable);
};
}
}
}

#endif//__UT_ROBOT_GO2_OBSTACLES_AVOID_CLIENT_HPP__
