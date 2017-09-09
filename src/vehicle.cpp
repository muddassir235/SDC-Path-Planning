        #include <iostream>
#include "vehicle.h"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(int lane, int s, int v, int a) {

    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    state = "CS";
    max_acceleration = -1;

}

Vehicle::~Vehicle() {}

void Vehicle::update_state(map<int,vector < vector<int> > > predictions) {
	/*
    Updates the "state" of the vehicle by assigning one of the
    following values to 'self.state':

    "KL" - Keep Lane
     - The vehicle will attempt to drive its target speed, unless there is 
       traffic in front of it, in which case it will slow down.

    "LCL" or "LCR" - Lane Change Left / Right
     - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
       behavior for the "KL" state in the new lane.

    INPUTS
    - predictions 
    A dictionary. The keys are ids of other vehicles and the values are arrays
    where each entry corresponds to the vehicle's predicted location at the 
    corresponding timestep. The FIRST element in the array gives the vehicle's
    current position. Example (showing a car with id 3 moving at 2 m/s):

    {
      3 : [
        {"s" : 4, "lane": 0},
        {"s" : 6, "lane": 0},
        {"s" : 8, "lane": 0},
        {"s" : 10, "lane": 0},
      ]
    }

    */
    state = next_state(predictions); // this is an example of how you change state.
    cout<< "State: " <<state <<endl;
}

string Vehicle::next_state(map<int,vector < vector<int> > > predictions){
    vector<string> possibilities = possible_moves();
    vector<double> costs;
    
    for(int i=0; i<possibilities.size(); i++){
        string possibility = possibilities[i];
        vector<Vehicle::Snapshot> traj = trajectory(possibility,  predictions);
        cout<<"Calculating Cost: "<<possibility<<endl;
        costs.push_back(calculate_cost(traj, predictions));
    }
    
    int best = 0;
    
    for(int i=0; i<costs.size();i++){
        if(costs[i]<costs[best]){
            best = i;
        }
    }

    cout<<"Best: "<<possibilities[best]<<endl;
    
    return possibilities[best];
}

vector<string> Vehicle::possible_moves(){
    vector<string> possibilities;

    if(this->lane == 2){    
        possibilities = {
            "KL",
            "LCL"
        };
    }else if(this->lane == 0){
        possibilities = {
            "KL",
            "LCR"
        };
    }else{
        possibilities = {
            "KL",
            "LCL",
            "LCR"
        };
    }
    
    return possibilities;
}

Vehicle::Snapshot Vehicle::snapshot(){
    Vehicle::Snapshot snap;
    
    snap.lane = this->lane;
    snap.s = this->s;
    snap.v = this->v;
    snap.a = this->a;
    snap.state = this->state;
    
    return snap;
    
}

void Vehicle::restore(Vehicle::Snapshot s){
    
    this->lane = s.lane;
    this->s = s.s;
    this->v = s.v;
    this->a = s.a;
    this->state = s.state;
}

vector<Vehicle::Snapshot> Vehicle::trajectory(string possibility, map<int,vector < vector<int> > > predictions){
    Vehicle::Snapshot s = snapshot();
    this->state = possibility;

    vector<Vehicle::Snapshot> trajectory;
    trajectory.push_back(s);
    
    realize_state(predictions);
    
    increment(1);
    this->state = "KL";
    
    for(int i=0; i<5; i++){
        increment(1);
        trajectory.push_back(snapshot());
    }
    
    restore(s);
    
    return trajectory;
}

map<int, vector <vector<int> > > Vehicle::lane_predictions(map<int, vector <vector<int> > > predictions, int lane){
      map<int, vector <vector<int> > > filtered;
      map<int, vector <vector<int> > >::iterator it;

      for(it = predictions.begin(); it!=predictions.end(); ++it){    
          if(it->second[0][1] == lane){
              filtered.insert(std::pair<int,vector <vector<int> >>(it->first, it->second));
          }
      }
      
      return filtered;
}

bool Vehicle::will_collide(Snapshot snap, int s_now, int s_prev){
    int s = snap.s;
    int v = snap.v;
    
    int v_target = s_now - s_prev;
    
    if(s_prev<s)
        return (s_now>=s);
        
    if(s_prev>s)
        return (s_now<=s);
        
    if(s == s_prev)
        return not (v_target > v);
    
    std::cout<<"Value Error"<<endl;
}
  
Vehicle::TrajectoryData Vehicle::useful_data(vector<Vehicle::Snapshot> trajectory, map<int, vector <vector<int> > > predictions){

    vector<Vehicle::Snapshot> t = trajectory;
    
    map<int, vector <vector<int> > > p = predictions;
    
    Vehicle::Snapshot curr_snap = t[0];
    
    Vehicle::Snapshot first_snap = t[1];
    
    Vehicle::Snapshot last_snap = t[t.size()-1];
    
    int last_goal_distance = this->goal_s - last_snap.s;
    
    int last_goal_line_diff = abs(this->goal_lane - last_snap.lane);
    
    cout<<"Goal Lane: "<<this->goal_lane<<endl;
    
    cout<<"Last Snap Lane: "<<last_snap.lane<<endl;
    
    double dt = double(t.size());
    
    int proposed_lane = first_snap.lane;
    
    double avg_speed = (double)(last_snap.s - curr_snap.s)/dt;
    
    vector<int> accelerations;
    
    int closest_approach = 999999;
    
    bool collides = false;
    
    int collides_at = - 1;
    
    map<int, vector <vector<int> > > lane_preds = lane_predictions(p, proposed_lane);
    
    for(int i=1; i<=PLANNING_HORIZON; i++){
        
        Vehicle::Snapshot snap = t[i];
        
        int lane = snap.lane;
        
        int s = snap.s;
        
        int v = snap.v;
        
        int a = snap.a;
        
        accelerations.push_back(a);
        
        map<int, vector <vector<int> > >::iterator it;
        
        for(it=p.begin(); it!=p.end(); ++it){
            
            vector<int> curr_state = it->second[i];
            vector<int> last_state = it->second[i-1];
            
            collides = will_collide(snap, curr_state[0], last_state[0]);
            
            if(collides){
                
                collides_at = i;
                
            }
            
            int distance = curr_state[0] - s;
            
            if(distance<closest_approach){
                
                closest_approach = distance;
                
            }
            
        }
        
    }
    
    int max_acceleration = abs(accelerations[0]);
    
    for(int i=0; i<accelerations.size(); i++){
        
        if(max_acceleration<abs(accelerations[i])){
            
            max_acceleration = accelerations[i];
            
        }
        
    }
    
    double rms_acceleration_sum = 0;
    
    int rms_acceleration_n = accelerations.size();
    
    for(int i=0; i<rms_acceleration_n; i++){
        
        rms_acceleration_sum+=pow(accelerations[i],2);
        
    }
    
    double rms_acceleration = rms_acceleration_sum/rms_acceleration_n;
    
    Vehicle::TrajectoryData data;
    
    
    data.proposed_lane = proposed_lane;
    
    data.avg_speed = avg_speed;
    
    data.max_acceleration = max_acceleration;
    
    data.rms_acceleration = rms_acceleration;
    
    data.closest_approach = closest_approach;
    
    data.end_distance_to_goal = last_goal_distance;
    
    data.end_lanes_from_goal = last_goal_line_diff;
    
    data.collides = collides;
    
    data.collides_at = collides_at;
    
    return data;
    
}

double Vehicle::inefficiency_cost(vector<Vehicle::Snapshot> trajectory, map<int, vector <vector<int> > > predictions, Vehicle::TrajectoryData data){
    
    double speed = data.avg_speed;
    
    double target_speed = (double) this->target_speed;
    
    double diff = target_speed - speed;
    
    double pct = diff/target_speed;
    
    double mx = pow(pct, 2);
    
    return mx*EFFICIENCY;
    
}
  
double Vehicle::buffer_cost(vector<Vehicle::Snapshot> trajectory, map<int, vector <vector<int> > > predictions, Vehicle::TrajectoryData data){
    
    int closest = data.closest_approach;
    
    if(closest == 0)
        return 10*DANGER;
    
    double time_steps_away;
    
    if(data.avg_speed>0.001)
        time_steps_away = closest/data.avg_speed;
    else
        time_steps_away = 999;
        
    if(time_steps_away>DESIRED_BUFFER){
        
        return 0.0;
        
    }
    
    if(time_steps_away<-DESIRED_BUFFER){
        
        return 0.0;
        
    }
    
    double mx = 1.0 - pow(double(time_steps_away)/double(DESIRED_BUFFER), 2);
    
    return mx*DANGER;
    
}
  

double Vehicle::collision_cost(vector<Vehicle::Snapshot> trajectory, map<int, vector <vector<int> > > predictions, Vehicle::TrajectoryData data){
    
    if(data.collides){
        
        double time_till_collision = (double) data.collides_at;
        
        double exponent = pow(time_till_collision,2);
        
        double mx = exp(exponent);
        
        return mx*COLLISION;
        
    }
    
    return 0;
    
}


double Vehicle::calculate_cost(vector<Vehicle::Snapshot> trajectory, map<int, vector <vector<int> > > predictions){
    
    Vehicle::TrajectoryData data = useful_data(trajectory, predictions);
    
    double cost = 0.0;
    
    double inefficiency_cost_ = inefficiency_cost(trajectory, predictions, data);
    cout<<"*Inefficiency cost: "<< inefficiency_cost_ <<endl;
    cost+=inefficiency_cost_;
    
    double buffer_cost_ = buffer_cost(trajectory, predictions, data);
    cout<<"*Buffer cost: "<< buffer_cost_ << endl;
    cost+=buffer_cost_;
    
    double collision_cost_ = collision_cost(trajectory, predictions, data);
    cout<<"Collision cost: "<<collision_cost_ <<endl;
    cost+=collision_cost_;
    
    cout<<"*******Total cost:: "<<cost<<endl;
    
    return cost;
    
}
  


void Vehicle::configure(vector<int> road_data) {
	/*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle. 
    */
    target_speed = road_data[0];
    lanes_available = road_data[1];
    goal_s = road_data[2];
    goal_lane = road_data[3];
    max_acceleration = road_data[4];
}

void Vehicle::increment(int dt = 1) {

	this->s += this->v * dt;
    this->v += this->a * dt;
}

void Vehicle::realize_state(map<int,vector < vector<int> > > predictions) {
   
	/*
    Given a state, realize it by adjusting acceleration and lane.
    Note - lane changes happen instantaneously.
    */
    string state = this->state;
    if(state.compare("CS") == 0)
    {
    	realize_constant_speed();
    }
    else if(state.compare("KL") == 0)
    {
    	realize_keep_lane(predictions);
    }
    else if(state.compare("LCL") == 0)
    {
    	realize_lane_change(predictions, "L");
    }
    else if(state.compare("LCR") == 0)
    {
    	realize_lane_change(predictions, "R");
    }
}

void Vehicle::realize_constant_speed() {
	a = 0;
}

int Vehicle::_max_accel_for_lane(map<int,vector<vector<int> > > predictions, int lane, int s) {

	int delta_v_til_target = target_speed - v;
    int max_acc = min(max_acceleration, delta_v_til_target);

    map<int, vector<vector<int> > >::iterator it = predictions.begin();
    vector<vector<vector<int> > > in_front;
    while(it != predictions.end())
    {
       
    	int v_id = it->first;
    	
        vector<vector<int> > v = it->second;
        
        if((v[0][0] == lane) && (v[0][1] > s))
        {
        	in_front.push_back(v);

        }
        it++;
    }
    
    if(in_front.size() > 0)
    {
    	int min_s = 1000;
    	vector<vector<int>> leading = {};
    	for(int i = 0; i < in_front.size(); i++)
    	{
    		if((in_front[i][0][1]-s) < min_s)
    		{
    			min_s = (in_front[i][0][1]-s);
    			leading = in_front[i];
    		}
    	}
    	
    	int next_pos = leading[1][1];
    	int my_next = s + this->v;
    	int separation_next = next_pos - my_next;
    	int available_room = separation_next - preferred_buffer;
    	max_acc = min(max_acc, available_room);
    }
    
    return max_acc;

}

void Vehicle::realize_keep_lane(map<int,vector< vector<int> > > predictions) {
	this->a = _max_accel_for_lane(predictions, this->lane, this->s);
}

void Vehicle::realize_lane_change(map<int,vector< vector<int> > > predictions, string direction) {
	int delta = -1;
    if (direction.compare("L") == 0)
    {
    	delta = -1;
    }
    this->lane += delta;
    int lane = this->lane;
    int s = this->s;
    this->a = _max_accel_for_lane(predictions, lane, s);
}