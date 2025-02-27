#ifndef __UT_ROBOT_GO2_SPORT_CLIENT_HPP__
#define __UT_ROBOT_GO2_SPORT_CLIENT_HPP__

#include <unitree/robot/client/client.hpp>

namespace unitree
{
namespace robot
{
namespace go2
{
/*
 * PathPoint
 */
struct stPathPoint
{
  float timeFromStart;
  float x;
  float y;
  float yaw;
  float vx;
  float vy;
  float vyaw;
};

typedef struct stPathPoint PathPoint;

/*
 * SportClient
 */
class SportClient : public Client
{
public:
    explicit SportClient(bool enableLease = false);
    ~SportClient();

    void Init();

    /*
     * @brief Damp
     * @api: 1001
     */
    int32_t Damp();

    /*
     * @brief BalanceStand
     * @api: 1002
     */
    int32_t BalanceStand();

    /*
     * @brief StopMove
     * @api: 1003
     */
    int32_t StopMove();

    /*
     * @brief StandUp
     * @api: 1004
     */
    int32_t StandUp();

    /*
     * @brief StandDown
     * @api: 1005
     */
    int32_t StandDown();

    /*
     * @brief RecoveryStand
     * @api: 1006
     */
    int32_t RecoveryStand();

    /*
     * @brief Euler
     * @api: 1007
     */
    int32_t Euler(float roll, float pitch, float yaw);

    /*
     * @brief Move
     * @api: 1008
     */
    int32_t Move(float vx, float vy, float vyaw);

    /*
     * @brief Sit
     * @api: 1009
     */
    int32_t Sit();

    /*
     * @brief RiseSit
     * @api: 1010
     */
    int32_t RiseSit();

    /*
     * @brief SwitchGait
     * @api: 1011
     */
    int32_t SwitchGait(int d);

    /*
     * @brief Trigger
     * @api: 1012
     */
    int32_t Trigger();

    /*
     * @brief BodyHeight
     * @api: 1013
     */
    int32_t BodyHeight(float height);

    /*
     * @brief FootRaiseHeight
     * @api: 1014
     */
    int32_t FootRaiseHeight(float height);

    /*
     * @brief SpeedLevel
     * @api: 1015
     */
    int32_t SpeedLevel(int level);

    /*
     * @brief Hello
     * @api: 1016
     */
    int32_t Hello();

    /*
     * @brief Stretch
     * @api: 1017
     */
    int32_t Stretch();

    /*
     * @brief TrajectoryFollow
     * @api: 1018
     */
    int32_t TrajectoryFollow(std::vector<PathPoint>& path);

    // /*
    //  * @brief GetBodyHeight
    //  * @api: 1024
    //  */
    // int32_t GetBodyHeight();

    // /*
    //  * @brief GetFootRaiseHeight
    //  * @api: 1025
    //  */
    // int32_t GetFootRaiseHeight(float& height);

    // /*
    //  * @brief GetSpeedLevel
    //  * @api: 1026
    //  */
    // int32_t GetSpeedLevel(int&);

    /*
     * @brief SwitchJoystick
     * @api: 1027
     */
    int32_t SwitchJoystick(bool flag);

    /*
     * @brief ContinuousGait
     * @api: 1019
     */
    int32_t ContinuousGait(bool flag);

    /*
     * @brief Wallow
     * @api: 1021
     */
    int32_t Wallow();

    // /*
    //  * @brief Content
    //  * @api: 1020
    //  */
    // int32_t Content();

    /*
     * @brief Heart
     * @api: 1036
     */
    int32_t Heart();

    /*
     * @brief Pose
     * @api: 1028
     */
    int32_t Pose(bool flag);

    /*
     * @brief Scrape
     * @api: 1029
     */
    int32_t Scrape();

    /*
     * @brief FrontFlip
     * @api: 1030
     */
    int32_t FrontFlip();

    /*
     * @brief FrontJump
     * @api: 1031
     */
    int32_t FrontJump();

    /*
     * @brief FrontPounce
     * @api: 1032
     */
    int32_t FrontPounce();

    /*
     * @brief Dance1
     * @api: 1022
     */
    int32_t Dance1();

    /*
     * @brief Dance2
     * @api: 1023
     */
    int32_t Dance2();

    /*
     * @brief Dance3
     * @api: 1037
     */
    int32_t Dance3();

    /*
     * @brief Dance4
     * @api: 1038
     */
    int32_t Dance4();

    /*
     * @brief HopSpinLeft
     * @api: 1039
     */
    int32_t HopSpinLeft();
  
    /*
     * @brief HopSpinRight
     * @api: 1040
     */
    int32_t HopSpinRight();

    /*
     * @brief WiggleHips
     * @api: 1033
     */
    int32_t WiggleHips();

    /*
     * @brief GetState
     * @api: 1034
     */
    int32_t GetState(const std::vector<std::string>& _vector, std::map<std::string, std::string>& _map);

    /*
     * @brief EconomicGait
     * @api: 1035
     */
    int32_t EconomicGait(bool flag);

};
}
}
}

#endif//__UT_ROBOT_GO2_SPORT_CLIENT_HPP__
