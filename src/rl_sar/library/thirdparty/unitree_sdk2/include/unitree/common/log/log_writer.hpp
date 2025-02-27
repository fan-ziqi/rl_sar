#ifndef __LOG_WRITER_HPP__
#define __LOG_WRITER_HPP__

#include <unitree/common/log/log_buffer.hpp>

namespace unitree
{
namespace common
{
class LogWriter
{
public:
    LogWriter() :
        mFd(UT_FD_INVALID)
    {}

    virtual ~LogWriter()
    {}

    virtual void SetFd(int32_t fd)
    {
        mFd = fd;
    }

    virtual int32_t GetFd()
    {
        return mFd;
    }

    virtual void Write(const std::string& s) = 0;

protected:
    int32_t mFd;
};

typedef std::shared_ptr<LogWriter> LogWriterPtr;

class LogDirectWriter : public LogWriter
{
public:
    LogDirectWriter();
    LogDirectWriter(int32_t fd);

    virtual ~LogDirectWriter();

    void SetFd(int32_t fd);
    void Write(const std::string& s);

protected:
    Mutex mLock;
};

class LogStdoutWriter : public LogDirectWriter
{
public:
    LogStdoutWriter() :
        LogDirectWriter(UT_FD_STDOUT)
    {}

    ~LogStdoutWriter()
    {}
};

class LogStderrWriter : public LogDirectWriter
{
public:
    LogStderrWriter() :
        LogDirectWriter(UT_FD_STDERR)
    {}

    ~LogStderrWriter()
    {}
};

class LogBufferWriter : public LogWriter
{
public:
    explicit LogBufferWriter(uint64_t writeIntervalMicrosec = 100000);

    ~LogBufferWriter();

    void SetFd(int32_t fd);
    void Write(const std::string& s);

private:
    LogBufferPtr mBufferPtr;
    Mutex mLock;
};

class LogAsyncBufferWriter : public LogWriter
{
public:
    explicit LogAsyncBufferWriter(uint64_t writeIntervalMicrosec = 10000);

    ~LogAsyncBufferWriter();

    void SetFd(int32_t fd);
    void Write(const std::string& s);

private:
    void DoWrite();
    bool WriteFD(const std::string& s);

private:
    LogBlockBufferPtr mBufferPtr;
    ThreadPtr mThreadPtr;
    Mutex mLock;
};
}
}

#endif//__LOG_WRITER_HPP__
