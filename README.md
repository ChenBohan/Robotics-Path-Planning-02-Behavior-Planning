# Robotics-Path-Planning-03-Behavior-Planning.
Udacity Self-Driving Car Engineer Nanodegree: Behavior Planning.

## Implement a Behavior Planner

### 1.Initialization

1.1.We initialize the roads and vehicles.
```cpp
Road road = Road(SPEED_LIMIT, TRAFFIC_DENSITY, LANE_SPEEDS)
Vehicle ego = Vehicle(lane_num, s, this->lane_speeds[lane_num], 0);
Vehicle vehicle = Vehicle(l,s,lane_speed,0);
```
1.2.We calculate the next road state.
```cpp
    while(it != this->vehicles.end())
    {
        int v_id = it->first;
        vector<Vehicle> preds = it->second.generate_predictions();
        predictions[v_id] = preds;
        it++;
    }
	it = this->vehicles.begin();
	while(it != this->vehicles.end())
    {
    	int v_id = it->first;
        if(v_id == ego_key)
        {   
        	vector<Vehicle> trajectory = it->second.choose_next_state(predictions);
        	it->second.realize_next_state(trajectory);
        }
        else {
            it->second.increment(1);
        }
        it++;
    }
```


### 2.Calculate the next states of vehicles. 

2.1.successor_states() - Uses the current state to return a vector of possible successor states for the finite state machine.

- "KL" - Keep Lane
- "LCL" / "LCR"- Lane Change Left / Lane Change Right
- "PLCL" / "PLCR" - Prepare Lane Change Left / Prepare Lane Change Right

```cpp
vector<string> Vehicle::successor_states() {

	vector<string> states;
	states.push_back("KL");
	string state = this->state;
	if (state.compare("KL") == 0) {
		states.push_back("PLCL");
		states.push_back("PLCR");
	}
	else if (state.compare("PLCL") == 0) {
		if (lane != lanes_available - 1) {
			states.push_back("PLCL");
			states.push_back("LCL");
		}
	}
	else if (state.compare("PLCR") == 0) {
		if (lane != 0) {
			states.push_back("PLCR");
			states.push_back("LCR");
		}
	}
	//If state is "LCL" or "LCR", then just return "KL"
	return states;
}
```

2.2.Given a possible next state, generate the appropriate trajectory to realize the next state.

```cpp
vector<Vehicle> Vehicle::generate_trajectory(string state, map<int, vector<Vehicle>> predictions) {

	vector<Vehicle> trajectory;
	if (state.compare("CS") == 0) {
		trajectory = constant_speed_trajectory();
	}
	else if (state.compare("KL") == 0) {
		trajectory = keep_lane_trajectory(predictions);
	}
	else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
		trajectory = lane_change_trajectory(state, predictions);
	}
	else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
		trajectory = prep_lane_change_trajectory(state, predictions);
	}
	return trajectory;
}
```

### 3.Trajectory generation

3.1Returns a vector of Vehicle objects representing a vehicle trajectory, given a state and predictions. 

3.1.1.Generate a constant speed trajectory.

3.1.2.Generate a keep lane trajectory.

3.1.3.Generate a trajectory preparing for a lane change.

```cpp
vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {

	float new_s;
	float new_v;
	float new_a;
	Vehicle vehicle_behind;
	int new_lane = this->lane + lane_direction[state];
	vector<Vehicle> trajectory = { Vehicle(this->lane, this->s, this->v, this->a, this->state) };
	vector<float> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);

	if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
		//Keep speed of current lane so as not to collide with car behind.
		new_s = curr_lane_new_kinematics[0];
		new_v = curr_lane_new_kinematics[1];
		new_a = curr_lane_new_kinematics[2];
	}
	else {
		vector<float> best_kinematics;
		vector<float> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
		//Choose kinematics with lowest velocity.
		if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
			best_kinematics = next_lane_new_kinematics;
		}
		else {
			best_kinematics = curr_lane_new_kinematics;
		}
		new_s = best_kinematics[0];
		new_v = best_kinematics[1];
		new_a = best_kinematics[2];
	}
	trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, state));
	return trajectory;
}

```

3.2.Generate a lane change trajectory.

```cpp
vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {

	int new_lane = this->lane + lane_direction[state];
	vector<Vehicle> trajectory;
	Vehicle next_lane_vehicle;
	//Check if a lane change is possible (check if another vehicle occupies that spot).
	for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
		next_lane_vehicle = it->second[0];
		if (next_lane_vehicle.s == this->s && next_lane_vehicle.lane == new_lane) {
			//If lane change is not possible, return empty trajectory.
			return trajectory;
		}
	}
	trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a, this->state));
	vector<float> kinematics = get_kinematics(predictions, new_lane);
	trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2], state));
	return trajectory;
}
```

3.3.Gets next timestep kinematics (position, velocity, acceleration for a given lane. Tries to choose the maximum velocity and acceleration, given other vehicle positions and accel/velocity constraints.

```cpp
vector<float> Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int lane) {

	float max_velocity_accel_limit = this->max_acceleration + this->v;
	float new_position;
	float new_velocity;
	float new_accel;
	Vehicle vehicle_ahead;
	Vehicle vehicle_behind;

	if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {
		if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
			new_velocity = vehicle_ahead.v; //must travel at the speed of traffic, regardless of preferred buffer
		}
		else {
			float max_velocity_in_front = (vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.v - 0.5 * (this->a);
			new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed);
		}
	}
	else {
		new_velocity = min(max_velocity_accel_limit, this->target_speed);
	}
	new_accel = new_velocity - this->v; //Equation: (v_1 - v_0)/t = acceleration
	new_position = this->s + new_velocity + new_accel / 2.0;
	return{ new_position, new_velocity, new_accel };
}
```

3.4.Return the best (lowest cost) trajectory corresponding to the next state.

```cpp
vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> predictions) {
	/*
	INPUT: A predictions map. This is a map of vehicle id keys with predicted
	vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
	the vehicle at the current timestep and one timestep in the future.
	OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.
	*/
	vector<string> states = successor_states();
	float cost;
	vector<float> costs;
	vector<string> final_states;
	vector<vector<Vehicle>> final_trajectories;

	for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
		vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
		if (trajectory.size() != 0) {
			cost = calculate_cost(*this, predictions, trajectory);
			costs.push_back(cost);
			final_trajectories.push_back(trajectory);
		}
	}
	vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
	int best_idx = distance(begin(costs), best_cost);
	return final_trajectories[best_idx];
}
```


3.5.``calculate_cost(Vehicle vehicle, map<int, vector<Vehicle>> predictions, vector<Vehicle> trajectory)``

- Included from ``cost.cpp``, computes the cost for a trajectory.

```cpp
float calculate_cost(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, const vector<Vehicle> & trajectory) {
    /*
    Sum weighted cost functions to get total cost for trajectory.
    */
    map<string, float> trajectory_data = get_helper_data(vehicle, trajectory, predictions);
    float cost = 0.0;

    //Add additional cost functions here.
    vector< function<float(const Vehicle & , const vector<Vehicle> &, const map<int, vector<Vehicle>> &, map<string, float> &)>> cf_list = {goal_distance_cost, inefficiency_cost};
    vector<float> weight_list = {REACH_GOAL, EFFICIENCY};
    
    for (int i = 0; i < cf_list.size(); i++) {
        float new_cost = weight_list[i]*cf_list[i](vehicle, trajectory, predictions, trajectory_data);
        cost += new_cost;
    }
    return cost;
}
```
### Choose appropriate weights for the cost functions in ``cost.cpp`` to induce the desired vehicle behavior.

1.Cost increases based on distance of intended lane (for planning a lane change) and final lane of trajectory.

Cost of being out of goal lane also becomes larger as vehicle approaches goal distance.

```cpp
cost = 1 - 2*exp(-(abs(2.0*vehicle.goal_lane - data["intended_lane"] - data["final_lane"]) / distance));
```

2.Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than vehicle's target speed. 

```cpp
float cost = (2.0*vehicle.target_speed - proposed_speed_intended - proposed_speed_final)/vehicle.target_speed;
```

3.Sum weighted cost functions to get total cost for trajectory.
```cpp
float new_cost = weight_list[i]*cf_list[i](vehicle, trajectory, predictions, trajectory_data);
```
