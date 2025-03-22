#ifndef ROBOT_STATE_MACHINE_HPP
#define ROBOT_STATE_MACHINE_HPP

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <map>
#include <string>
#include <vector>
#include <mutex>

namespace dodo_robot
{

// Define all possible robot states
enum class RobotState {
    INACTIVE,      // Robot is not active
    INITIALIZING,  // Robot is initializing systems
    STANDBY,       // Robot is ready but not moving
    MANUAL,        // Robot is under manual control
    AUTONOMOUS,    // Robot is operating autonomously
    ERROR,         // Robot is in error state
    EMERGENCY_STOP // Robot is in emergency stop
};

// Define all possible state transition events
enum class StateEvent {
    BOOT_COMPLETE,     // Initialization finished
    ACTIVATION,        // Activate the robot
    DEACTIVATION,      // Deactivate the robot
    MANUAL_CONTROL,    // Switch to manual control
    AUTO_CONTROL,      // Switch to autonomous control
    ERROR_DETECTED,    // Error detected
    ERROR_RESOLVED,    // Error has been resolved
    EMERGENCY,         // Emergency condition detected
    EMERGENCY_CLEARED  // Emergency condition cleared
};

// Type definition for transition callbacks
using TransitionCallback = std::function<void(RobotState, RobotState)>;

class RobotStateMachine {
public:
    RobotStateMachine();
    virtual ~RobotStateMachine() = default;

    // Get current state
    RobotState getCurrentState() const;
    
    // Get current state as string
    std::string getCurrentStateString() const;
    
    // Process an event and potentially change state
    bool processEvent(StateEvent event);
    
    // Register a callback for state transitions
    void registerTransitionCallback(TransitionCallback callback);
    
    // Convert RobotState to string
    static std::string stateToString(RobotState state);
    
    // Convert StateEvent to string
    static std::string eventToString(StateEvent event);

private:
    // Current robot state
    RobotState current_state_;
    
    // Mutex for thread-safe state access
    mutable std::mutex state_mutex_;
    
    // Transition callbacks
    std::vector<TransitionCallback> transition_callbacks_;
    
    // Define valid state transitions
    std::map<RobotState, std::map<StateEvent, RobotState>> transition_table_;
    
    // Initialize the state transition table
    void initTransitionTable();
    
    // Execute state transition
    void transitionTo(RobotState new_state);
    
    // Check if a transition is valid
    bool isValidTransition(RobotState from, StateEvent event) const;
};

} // namespace dodo_robot

#endif // ROBOT_STATE_MACHINE_HPP