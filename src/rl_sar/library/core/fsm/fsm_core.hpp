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
    FSMState(std::string name) : state_name_(std::move(name)) {}
    virtual ~FSMState() = default;

    virtual void Enter() = 0;
    virtual void Run() = 0;
    virtual void Exit() = 0;
    virtual std::string CheckChange() { return state_name_; }

    const std::string &GetStateName() const { return state_name_; }

protected:
    std::string state_name_;
};

class FSM
{
public:
    FSM() : current_state_(nullptr), next_state_(nullptr), mode_(Mode::NORMAL) {}

    void AddState(std::shared_ptr<FSMState> state)
    {
        states_[state->GetStateName()] = state;
    }

    void SetInitialState(const std::string &name)
    {
        current_state_ = states_.at(name);
        current_state_->Enter();
        next_state_ = current_state_;
        std::cout << "[FSM] Set initial state: " << name << std::endl;
    }

    void RequestStateChange(const std::string& state_name)
    {
        if (states_.find(state_name) != states_.end() && current_state_ && current_state_->GetStateName() != state_name)
        {
            next_state_ = states_.at(state_name);
            mode_ = Mode::CHANGE;
            std::cout << std::endl << "\033[0;34m[FSM]\033[0m Request switch from " << current_state_->GetStateName() << " to " << next_state_->GetStateName() << std::endl;
        }
    }

    void Run()
    {
        if (!current_state_)
            return;

        if (mode_ == Mode::NORMAL)
        {
            current_state_->Run();
            std::string next = current_state_->CheckChange();
            if (next != current_state_->GetStateName())
            {
                mode_ = Mode::CHANGE;
                next_state_ = states_.at(next);
                std::cout << std::endl << "\033[0;34m[FSM]\033[0m Switch from " << current_state_->GetStateName() << " to " << next_state_->GetStateName() << std::endl;
            }
        }
        else if (mode_ == Mode::CHANGE)
        {
            current_state_->Exit();
            current_state_ = next_state_;
            current_state_->Enter();
            mode_ = Mode::NORMAL;
            current_state_->Run();
        }
    }

    enum class Mode
    {
        NORMAL,
        CHANGE
    };

    std::unordered_map<std::string, std::shared_ptr<FSMState>> states_;
    std::shared_ptr<FSMState> current_state_;
    std::shared_ptr<FSMState> next_state_;
    Mode mode_;
};

class FSMFactory
{
public:
    virtual ~FSMFactory() = default;
    virtual std::shared_ptr<FSMState> CreateState(void *context, const std::string &state_name) = 0;
    virtual std::string GetType() const = 0;
    virtual std::vector<std::string> GetSupportedStates() const = 0;
    virtual std::string GetInitialState() const = 0;
};

class FSMManager
{
public:
    static FSMManager &GetInstance()
    {
        static FSMManager instance;
        return instance;
    }

    void RegisterFactory(std::shared_ptr<FSMFactory> factory)
    {
        if (factory)
        {
            std::string type = factory->GetType();
            factories_[type] = factory;
            std::cout << "[FSMManager] Registered type: " << type << std::endl;
        }
    }

    std::shared_ptr<FSM> CreateFSM(const std::string &type, void *context)
    {
        auto it = factories_.find(type);
        if (it == factories_.end())
        {
            std::cout << "\033[0;31m[FSMManager]\033[0m Error: Unsupported type: " << type << std::endl;
            return nullptr;
        }
        auto factory = it->second;
        auto state_names = factory->GetSupportedStates();
        if (state_names.empty())
        {
            std::cout << "\033[0;31m[FSMManager]\033[0m Error: No states registered for type: " << type << std::endl;
            return nullptr;
        }
        auto fsm = std::make_shared<FSM>();
        for (const auto &state_name : state_names)
        {
            auto state = factory->CreateState(context, state_name);
            if (state)
                fsm->AddState(state);
        }
        fsm->SetInitialState(factory->GetInitialState());
        std::cout << "[FSMManager] FSM created for type: " << type << std::endl;
        return fsm;
    }

    bool IsTypeSupported(const std::string &type) const
    {
        return factories_.find(type) != factories_.end();
    }

    std::vector<std::string> GetSupportedTypes() const
    {
        std::vector<std::string> types;
        for (const auto &pair : factories_)
            types.push_back(pair.first);
        return types;
    }

private:
    FSMManager() = default;
    std::unordered_map<std::string, std::shared_ptr<FSMFactory>> factories_;
};

#define CONCATENATE_DETAIL(x, y) x##y
#define CONCATENATE(x, y) CONCATENATE_DETAIL(x, y)
#define REGISTER_FSM_FACTORY(FactoryClass, initialStateName) \
    namespace { \
        const bool CONCATENATE(registered_fsm_factory_, __COUNTER__) = []() { \
            FSMManager::GetInstance().RegisterFactory(std::make_shared<FactoryClass>(initialStateName)); \
            return true; \
        }(); \
    }

#endif // FSM_CORE_HPP
