#ifndef __UT_LOG_INITOR_HPP__
#define __UT_LOG_INITOR_HPP__

#include <unitree/common/log/log_policy.hpp>
#include <unitree/common/log/log_store.hpp>
#include <unitree/common/log/log_logger.hpp>

namespace unitree
{
namespace common
{
class LogInitor
{
public:
    static LogInitor* Instance()
    {
        static LogInitor inst;
        return &inst;
    }

    void Init(const std::string& configFileName);
    Logger* GetLogger(const std::string& tag);

    void Final();

private:
    LogInitor();
    void ParseConf(Any json);
    void InitLogger();

private:
    bool mInited;
    std::set<std::string> mStoreNames;
    std::vector<LogPolicyPtr> mPolicis;
    std::vector<LogStorePolicyPtr> mStorePolicis;
    std::map<std::string, LoggerPtr> mLoggerMap;
    Mutex mLock;
};

void LogInit(const std::string& configFileName = "");
void LogFinal();

Logger* GetLogger(const std::string& tag);

}
}

#endif//__UT_LOG_INITOR_HPP__
