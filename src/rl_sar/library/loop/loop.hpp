#ifndef LOOP_H
#define LOOP_H

#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <functional>

typedef std::function<void()> Callback;

class Loop {
 public:
  Loop(std::string name, float period, int bindCPU = -1)
      : _name(name), _period(period), _bindCPU(bindCPU) {}
  ~Loop() {
    if (_isrunning) {
      shutdown();  // Ensure the loop is stopped when the object is destroyed
    }
  }

  void start() {
    _isrunning = true;
    _thread = std::thread([this]() {
        if (_bindCPU >= 0) {
          std::lock_guard<std::mutex> lock(_printMutex);
          std::cout << "[Loop Start] named: " << _name << ", period: " << _period * 1000 << " (ms), run at cpu: " << _bindCPU << std::endl;
        } else {
          std::lock_guard<std::mutex> lock(_printMutex);
          std::cout << "[Loop Start] named: " << _name << ", period: " << _period * 1000 << " (ms), cpu unspecified" << std::endl;
        }
        entryFunc();
    });  // Start the loop in a new thread
  }

  void shutdown() {
      _isrunning = false;
      if (_thread.joinable()) {
        _thread.join();  // Wait for the loop thread to finish
        std::lock_guard<std::mutex> lock(_printMutex);
        std::cout << "[Loop End] named: " << _name << std::endl;
      }
  }

  virtual void functionCB() = 0;

 protected:
  void entryFunc() {
    while (_isrunning) {
      functionCB();  // Call the overridden functionCB in a loop
      std::this_thread::sleep_for(std::chrono::duration<float>(_period));  // Wait for the specified period
    }
  }

  std::string _name;
  float _period;
  int _bindCPU;
  bool _isrunning = false;
  std::thread _thread;
  static std::mutex _printMutex;
};

std::mutex Loop::_printMutex;

class LoopFunc : public Loop {
 public:
  LoopFunc(std::string name, float period, const Callback& cb)
      : Loop(name, period), _fp(cb) {}

  LoopFunc(std::string name, float period, int bindCPU, const Callback& cb)
      : Loop(name, period, bindCPU), _fp(cb) {}

  void functionCB() override {
    {
      std::lock_guard<std::mutex> lock(_printMutex);
      (_fp)();  // Call the provided callback function
    }
  }

 private:
  Callback _fp;
};

#endif
