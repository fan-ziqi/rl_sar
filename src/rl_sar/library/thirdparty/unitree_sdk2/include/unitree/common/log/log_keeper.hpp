#ifndef __UT_LOG_FILE_KEEPER_H__
#define __UT_LOG_FILE_KEEPER_H__

#include <unitree/common/log/log_policy.hpp>
#include <unitree/common/log/log_writer.hpp>

namespace unitree
{
namespace common
{
class LogKeeper
{
public:
    enum
    {
        ROLLING_WAIT_MICROSEC  = 1000000
    };

    LogKeeper(LogStorePolicyPtr storePolicyPtr);
    ~LogKeeper();

    void SetWriter(LogWriterPtr writerPtr);
    void AppendDataSize(int64_t len);

private:
    void Rolling();

    void OpenFile();
    void CloseFile();
    bool CheckFile();

    void ThreadRolling();

    bool IsNeedToRolling(int64_t len);
    void CheckRolling();

    std::string MakeRegexExpress();

private:
    bool mQuit;
    int64_t mFileSize;
    std::string mFileName;
    std::string mDirectory;
    FilePtr mFilePtr;
    LogStorePolicyPtr mStorePolicyPtr;
    LogWriterPtr mWriterPtr;
    ThreadPtr mThreadPtr;
    MutexCond mMutexCond;
};

typedef std::shared_ptr<LogKeeper> LogKeeperPtr;

}
}

#endif//__UT_LOG_FILE_KEEPER_H__
