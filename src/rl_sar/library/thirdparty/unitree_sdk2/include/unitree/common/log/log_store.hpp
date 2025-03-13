#ifndef __LOG_STORE_HPP__
#define __LOG_STORE_HPP__

#include <unitree/common/log/log_writer.hpp>
#include <unitree/common/log/log_keeper.hpp>

namespace unitree
{
namespace common
{
class LogStore
{
public:
    virtual void Append(const std::string& s) = 0;
};

typedef std::shared_ptr<LogStore> LogStorePtr;

class LogStdoutStore : public LogStore
{
public:
    LogStdoutStore();

    void Append(const std::string& s);

protected:
    LogWriterPtr mWriterPtr;
};

typedef std::shared_ptr<LogStdoutStore> LogStdoutStorePtr;

class LogStderrStore : public LogStore
{
public:
    LogStderrStore();

    void Append(const std::string& s);

protected:
    LogWriterPtr mWriterPtr;
};

typedef std::shared_ptr<LogStderrStore> LogStderrStorePtr;

class LogFileStore : public LogStore
{
public:
    LogFileStore(LogKeeperPtr keeperPtr);

    void Append(const std::string& s);

private:
    LogKeeperPtr mKeeperPtr;
    LogWriterPtr mWriterPtr;
};

typedef std::shared_ptr<LogFileStore> LogFileStorePtr;

class LogFileAsyncStore : public LogStore
{
public:
    enum
    {
        WRITE_INTERVAL = 100000,
    };

    LogFileAsyncStore(LogKeeperPtr keeperPtr);

    void Append(const std::string& s);

private:
    LogWriterPtr mWriterPtr;
    LogKeeperPtr mKeeperPtr;
};

typedef std::shared_ptr<LogFileAsyncStore> LogFileAsyncStorePtr;

}
}

#endif//__LOG_STORE_HPP__
