# Robotics-Path-Planning-03-Behavior-Planning.
Udacity Self-Driving Car Engineer Nanodegree: Behavior Planning.

## Implement a Behavior Planner

### Implement the ``choose_next_state`` method in the ``vehicle.cpp`` class. 

1.successor_states() - Uses the current state to return a vector of possible successor states for the finite state machine.

- "KL" - Keep Lane
- "LCL" / "LCR"- Lane Change Left / Lane Change Right
- "PLCL" / "PLCR" - Prepare Lane Change Left / Prepare Lane Change Right

```cpp
vector<string> Vehicle::successor_states() {
    /*
    Provides the possible next states given the current state for the FSM 
    discussed in the course, with the exception that lane changes happen 
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    vector<string> states;
    states.push_back("KL");
    string state = this->state;
    if(state.compare("KL") == 0) {
        states.push_back("PLCL");
        states.push_back("PLCR");
    } else if (state.compare("PLCL") == 0) {
        if (lane != lanes_available - 1) {
            states.push_back("PLCL");
            states.push_back("LCL");
        }
    } else if (state.compare("PLCR") == 0) {
        if (lane != 0) {
            states.push_back("PLCR");
            states.push_back("LCR");
        }
    }
    //If state is "LCL" or "LCR", then just return "KL"
    return states;
}
```

2.generate_trajectory(string state, map<int, vector<Vehicle>> predictions) 

Returns a vector of Vehicle objects representing a vehicle trajectory, given a state and predictions. 

Note that trajectory vectors might have size 0 if no possible trajectory exists for the state.

3.calculate_cost(Vehicle vehicle, map<int, vector<Vehicle>> predictions, vector<Vehicle> trajectory)
