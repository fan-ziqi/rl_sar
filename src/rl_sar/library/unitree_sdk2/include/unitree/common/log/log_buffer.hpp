#ifndef __UT_LOG_BUFFER_HPP__
#define __UT_LOG_BUFFER_HPP__

#include <unitree/common/log/log_decl.hpp>

namespace unitree
{
namespace common
{
class LogBuffer
{
public:
    LogBuffer();

    bool Append(const std::string& s);

    const std::string& Get();
    void Get(std::string& s);

    void Clear();

    bool Empty();

private:
    std::string mData;
};

typedef std::shared_ptr<LogBuffer> LogBufferPtr;

class LogBlockBuffer
{
public:
    typedef std::shared_ptr<LogBuffer> LOG_BUFFER_PTR;

    LogBlockBuffer();
    ~LogBlockBuffer();

    bool Append(const std::string& s);

    /*
     * Get reference of data. [UNSAFE]
     */
    const std::string& Get();
    void Get(std::string& s);

    /*
     * Clear data.
     */
    void Clear(bool lock = false);

    void Exchange();

private:
    volatile bool mR;
    std::vector<LOG_BUFFER_PTR> mChain;
    Mutex mLock;
};

typedef std::shared_ptr<LogBlockBuffer> LogBlockBufferPtr;
}
}

#endif//__UT_LOG_BUFFER_HPP__
