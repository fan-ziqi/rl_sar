#ifndef FSM_CORE_HPP
#define FSM_CORE_HPP

#include <iostream>
#include <string>
#include <unordered_map>
#include <memory>

class FSMState
{
public:
    FSMState(std::string name) : _stateName(std::move(name)) {}
    virtual ~FSMState() = default;

    virtual void enter() = 0;
    virtual void run() = 0;
    virtual void exit() = 0;
    virtual std::string checkChange() { return _stateName; }

    const std::string &getStateName() const { return _stateName; }

protected:
    std::string _stateName;
};

class FSM
{
public:
    FSM() : _currentState(nullptr), _nextState(nullptr), _mode(Mode::NORMAL) {}

    void addState(std::shared_ptr<FSMState> state)
    {
        _states[state->getStateName()] = state;
    }

    void setInitialState(const std::string &name)
    {
        _currentState = _states.at(name);
        _currentState->enter();
        _nextState = _currentState;
    }

    void run()
    {
        if (!_currentState)
            return;

        if (_mode == Mode::NORMAL)
        {
            _currentState->run();
            std::string next = _currentState->checkChange();
            if (next != _currentState->getStateName())
            {
                _mode = Mode::CHANGE;
                _nextState = _states.at(next);
                std::cout << std::endl << "[FSM]  Switch from " << _currentState->getStateName() << " to " << _nextState->getStateName() << std::endl;
            }
        }
        else if (_mode == Mode::CHANGE)
        {
            _currentState->exit();
            _currentState = _nextState;
            _currentState->enter();
            _mode = Mode::NORMAL;
            _currentState->run();
        }
    }

    enum class Mode
    {
        NORMAL,
        CHANGE
    };

    std::unordered_map<std::string, std::shared_ptr<FSMState>> _states;
    std::shared_ptr<FSMState> _currentState;
    std::shared_ptr<FSMState> _nextState;
    Mode _mode;
};

#endif // FSM_CORE_HPP
