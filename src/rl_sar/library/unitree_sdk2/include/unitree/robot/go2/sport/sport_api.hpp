#ifndef __UT_ROBOT_GO2_SPORT_API_HPP__
#define __UT_ROBOT_GO2_SPORT_API_HPP__

#include <unitree/common/json/jsonize.hpp>

namespace unitree
{
namespace robot
{
namespace go2
{
/*service name*/
const std::string ROBOT_SPORT_SERVICE_NAME = "sport";

/*api version*/
const std::string ROBOT_SPORT_API_VERSION = "1.0.0.1";

/*api id*/
const int32_t ROBOT_SPORT_API_ID_DAMP               = 1001;
const int32_t ROBOT_SPORT_API_ID_BALANCESTAND       = 1002;
const int32_t ROBOT_SPORT_API_ID_STOPMOVE           = 1003;
const int32_t ROBOT_SPORT_API_ID_STANDUP            = 1004;
const int32_t ROBOT_SPORT_API_ID_STANDDOWN          = 1005;
const int32_t ROBOT_SPORT_API_ID_RECOVERYSTAND      = 1006;
const int32_t ROBOT_SPORT_API_ID_EULER              = 1007;
const int32_t ROBOT_SPORT_API_ID_MOVE               = 1008;
const int32_t ROBOT_SPORT_API_ID_SIT                = 1009;
const int32_t ROBOT_SPORT_API_ID_RISESIT            = 1010;
const int32_t ROBOT_SPORT_API_ID_SWITCHGAIT         = 1011;
const int32_t ROBOT_SPORT_API_ID_TRIGGER            = 1012;
const int32_t ROBOT_SPORT_API_ID_BODYHEIGHT         = 1013;
const int32_t ROBOT_SPORT_API_ID_FOOTRAISEHEIGHT    = 1014;
const int32_t ROBOT_SPORT_API_ID_SPEEDLEVEL         = 1015;
const int32_t ROBOT_SPORT_API_ID_HELLO              = 1016;
const int32_t ROBOT_SPORT_API_ID_STRETCH            = 1017;
const int32_t ROBOT_SPORT_API_ID_TRAJECTORYFOLLOW   = 1018;
const int32_t ROBOT_SPORT_API_ID_CONTINUOUSGAIT     = 1019;
// const int32_t ROBOT_SPORT_API_ID_CONTENT            = 1020;
const int32_t ROBOT_SPORT_API_ID_WALLOW             = 1021;
const int32_t ROBOT_SPORT_API_ID_DANCE1             = 1022;
const int32_t ROBOT_SPORT_API_ID_DANCE2             = 1023;
// const int32_t ROBOT_SPORT_API_ID_GETBODYHEIGHT      = 1024;
// const int32_t ROBOT_SPORT_API_ID_GETFOOTRAISEHEIGHT = 1025;
// const int32_t ROBOT_SPORT_API_ID_GETSPEEDLEVEL      = 1026;
const int32_t ROBOT_SPORT_API_ID_SWITCHJOYSTICK     = 1027;
const int32_t ROBOT_SPORT_API_ID_POSE               = 1028;
const int32_t ROBOT_SPORT_API_ID_SCRAPE             = 1029;
const int32_t ROBOT_SPORT_API_ID_FRONTFLIP          = 1030;
const int32_t ROBOT_SPORT_API_ID_FRONTJUMP          = 1031;
const int32_t ROBOT_SPORT_API_ID_FRONTPOUNCE        = 1032;
const int32_t ROBOT_SPORT_API_ID_WIGGLEHIPS         = 1033;
const int32_t ROBOT_SPORT_API_ID_GETSTATE           = 1034;
const int32_t ROBOT_SPORT_API_ID_ECONOMICGAIT       = 1035;
const int32_t ROBOT_SPORT_API_ID_HEART              = 1036;
const int32_t ROBOT_SPORT_API_ID_DANCE3             = 1037;
const int32_t ROBOT_SPORT_API_ID_DANCE4             = 1038;
const int32_t ROBOT_SPORT_API_ID_HOPSPINLEFT        = 1039;
const int32_t ROBOT_SPORT_API_ID_HOPSPINRIGHT       = 1040;


}
}
}

#endif //__UT_ROBOT_GO2_SPORT_API_HPP__
