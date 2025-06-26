#ifndef FSM_CORE_HPP
#define FSM_CORE_HPP

#include <iostream>
#include <string>
#include <unordered_map>
#include <memory>
#include <vector>

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

class FSMStateFactory
{
public:
    virtual ~FSMStateFactory() = default;
    virtual std::shared_ptr<FSMState> createState(void *context, const std::string &stateName) = 0;
    virtual std::string getType() const = 0;
    virtual std::vector<std::string> getSupportedStates() const = 0;
    virtual std::string getInitialState() const = 0;
};

class FSMManager
{
public:
    static FSMManager &getInstance()
    {
        static FSMManager instance;
        return instance;
    }

    void registerFactory(std::shared_ptr<FSMStateFactory> factory)
    {
        if (factory)
        {
            std::string type = factory->getType();
            factories_[type] = factory;
            std::cout << "[FSMManager] Registered type: " << type << std::endl;
        }
    }

    std::shared_ptr<FSM> createFSM(const std::string &type, void *context)
    {
        auto it = factories_.find(type);
        if (it == factories_.end())
        {
            std::cout << "[FSMManager] Error: Unsupported type: " << type << std::endl;
            return nullptr;
        }
        auto factory = it->second;
        auto stateNames = factory->getSupportedStates();
        if (stateNames.empty())
        {
            std::cout << "[FSMManager] Error: No states registered for type: " << type << std::endl;
            return nullptr;
        }
        auto fsm = std::make_shared<FSM>();
        for (const auto &stateName : stateNames)
        {
            auto state = factory->createState(context, stateName);
            if (state)
                fsm->addState(state);
        }
        fsm->setInitialState(factory->getInitialState());
        std::cout << "[FSMManager] FSM created for type: " << type << std::endl;
        return fsm;
    }

    bool isTypeSupported(const std::string &type) const
    {
        return factories_.find(type) != factories_.end();
    }

    std::vector<std::string> getSupportedTypes() const
    {
        std::vector<std::string> types;
        for (const auto &pair : factories_)
            types.push_back(pair.first);
        return types;
    }

private:
    FSMManager() = default;
    std::unordered_map<std::string, std::shared_ptr<FSMStateFactory>> factories_;
};

#define CONCATENATE_DETAIL(x, y) x##y
#define CONCATENATE(x, y) CONCATENATE_DETAIL(x, y)
#define REGISTER_FSM_FACTORY(FactoryClass, initialStateName) \
    namespace { \
        const bool CONCATENATE(registered_fsm_factory_, __COUNTER__) = []() { \
            FSMManager::getInstance().registerFactory(std::make_shared<FactoryClass>(initialStateName)); \
            return true; \
        }(); \
    }

#endif // FSM_CORE_HPP
