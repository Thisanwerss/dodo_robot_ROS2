#include "dodo_bringup/robot_state_machine.hpp"

namespace dodo_bringup
{

RobotStateMachine::RobotStateMachine() : current_state_(RobotState::INACTIVE) 
{
    initTransitionTable();
}

RobotState RobotStateMachine::getCurrentState() const 
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_state_;
}

std::string RobotStateMachine::getCurrentStateString() const 
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return stateToString(current_state_);
}

bool RobotStateMachine::processEvent(StateEvent event) 
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (isValidTransition(current_state_, event)) {
        RobotState new_state = transition_table_[current_state_][event];
        transitionTo(new_state);
        return true;
    }
    
    return false;
}

void RobotStateMachine::registerTransitionCallback(TransitionCallback callback) 
{
    transition_callbacks_.push_back(callback);
}

std::string RobotStateMachine::stateToString(RobotState state) 
{
    switch (state) {
        case RobotState::INACTIVE:
            return "INACTIVE";
        case RobotState::INITIALIZING:
            return "INITIALIZING";
        case RobotState::STANDBY:
            return "STANDBY";
        case RobotState::MANUAL:
            return "MANUAL";
        case RobotState::AUTONOMOUS:
            return "AUTONOMOUS";
        case RobotState::ERROR:
            return "ERROR";
        case RobotState::EMERGENCY_STOP:
            return "EMERGENCY_STOP";
        default:
            return "UNKNOWN";
    }
}

std::string RobotStateMachine::eventToString(StateEvent event) 
{
    switch (event) {
        case StateEvent::BOOT_COMPLETE:
            return "BOOT_COMPLETE";
        case StateEvent::ACTIVATION:
            return "ACTIVATION";
        case StateEvent::DEACTIVATION:
            return "DEACTIVATION";
        case StateEvent::MANUAL_CONTROL:
            return "MANUAL_CONTROL";
        case StateEvent::AUTO_CONTROL:
            return "AUTO_CONTROL";
        case StateEvent::ERROR_DETECTED:
            return "ERROR_DETECTED";
        case StateEvent::ERROR_RESOLVED:
            return "ERROR_RESOLVED";
        case StateEvent::EMERGENCY:
            return "EMERGENCY";
        case StateEvent::EMERGENCY_CLEARED:
            return "EMERGENCY_CLEARED";
        default:
            return "UNKNOWN";
    }
}

void RobotStateMachine::initTransitionTable() 
{
    // Transitions from INACTIVE
    transition_table_[RobotState::INACTIVE][StateEvent::BOOT_COMPLETE] = RobotState::INITIALIZING;
    
    // Transitions from INITIALIZING
    transition_table_[RobotState::INITIALIZING][StateEvent::ACTIVATION] = RobotState::STANDBY;
    transition_table_[RobotState::INITIALIZING][StateEvent::ERROR_DETECTED] = RobotState::ERROR;
    transition_table_[RobotState::INITIALIZING][StateEvent::EMERGENCY] = RobotState::EMERGENCY_STOP;
    
    // Transitions from STANDBY
    transition_table_[RobotState::STANDBY][StateEvent::DEACTIVATION] = RobotState::INACTIVE;
    transition_table_[RobotState::STANDBY][StateEvent::MANUAL_CONTROL] = RobotState::MANUAL;
    transition_table_[RobotState::STANDBY][StateEvent::AUTO_CONTROL] = RobotState::AUTONOMOUS;
    transition_table_[RobotState::STANDBY][StateEvent::ERROR_DETECTED] = RobotState::ERROR;
    transition_table_[RobotState::STANDBY][StateEvent::EMERGENCY] = RobotState::EMERGENCY_STOP;
    
    // Transitions from MANUAL
    transition_table_[RobotState::MANUAL][StateEvent::DEACTIVATION] = RobotState::STANDBY;
    transition_table_[RobotState::MANUAL][StateEvent::AUTO_CONTROL] = RobotState::AUTONOMOUS;
    transition_table_[RobotState::MANUAL][StateEvent::ERROR_DETECTED] = RobotState::ERROR;
    transition_table_[RobotState::MANUAL][StateEvent::EMERGENCY] = RobotState::EMERGENCY_STOP;
    
    // Transitions from AUTONOMOUS
    transition_table_[RobotState::AUTONOMOUS][StateEvent::DEACTIVATION] = RobotState::STANDBY;
    transition_table_[RobotState::AUTONOMOUS][StateEvent::MANUAL_CONTROL] = RobotState::MANUAL;
    transition_table_[RobotState::AUTONOMOUS][StateEvent::ERROR_DETECTED] = RobotState::ERROR;
    transition_table_[RobotState::AUTONOMOUS][StateEvent::EMERGENCY] = RobotState::EMERGENCY_STOP;
    
    // Transitions from ERROR
    transition_table_[RobotState::ERROR][StateEvent::ERROR_RESOLVED] = RobotState::STANDBY;
    transition_table_[RobotState::ERROR][StateEvent::EMERGENCY] = RobotState::EMERGENCY_STOP;
    
    // Transitions from EMERGENCY_STOP
    transition_table_[RobotState::EMERGENCY_STOP][StateEvent::EMERGENCY_CLEARED] = RobotState::ERROR;
}

void RobotStateMachine::transitionTo(RobotState new_state) 
{
    RobotState old_state = current_state_;
    current_state_ = new_state;
    
    // Call all registered callbacks
    for (const auto& callback : transition_callbacks_) {
        callback(old_state, new_state);
    }
}

bool RobotStateMachine::isValidTransition(RobotState from, StateEvent event) const 
{
    // Check if the state exists in the transition table
    auto state_it = transition_table_.find(from);
    if (state_it == transition_table_.end()) {
        return false;
    }
    
    // Check if the event exists for this state
    auto event_it = state_it->second.find(event);
    if (event_it == state_it->second.end()) {
        return false;
    }
    
    return true;
}

} // namespace dodo_bringup