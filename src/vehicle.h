#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

using namespace std;

class Vehicle {
public:

  int PLANNING_HORIZON = 2;

  // priority levels for costs
  long COLLISION  = pow(10, 6);
  long DANGER     = pow(10, 5);
  long REACH_GOAL = pow(10, 5);
  long COMFORT    = 1000;
  long EFFICIENCY = 100000;

  double DESIRED_BUFFER = 1.5;
  
  struct collider{

    bool collision ; // is there a collision?
    int  time; // time collision happens

  };
  
  struct Snapshot{
    
    int lane;
    int s;
    int v;
    int a;
    string state;
    int max_acceleration = 0;
    
  };
  
  struct TrajectoryData{
      
    int proposed_lane;
    double avg_speed;
    int max_acceleration;
    double rms_acceleration;
    int closest_approach;
    int end_distance_to_goal;
    int end_lanes_from_goal;
    bool collides;
    int collides_at;
      
  };

  int L = 1;

  int preferred_buffer = 6; // impacts "keep lane" behavior.

  int lane;

  int s;

  int v;

  int a;

  int target_speed;

  int lanes_available;

  int max_acceleration;

  int goal_lane;

  int goal_s;

  string state;

  /**
  * Constructor
  */
  Vehicle(int lane, int s, int v, int a);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  void update_state(map<int, vector <vector<int> > > predictions);
  
  string next_state(map<int, vector <vector<int> > > predictions);

  vector<string> possible_moves();
  
  Snapshot snapshot();
  
  void restore(Snapshot s);
  
  vector<Snapshot> trajectory(string possibility, map<int, vector <vector<int> > > predictions);
  
  map<int, vector <vector<int> > > lane_predictions(map<int, vector <vector<int> > > predictions, int lane);
  
  bool will_collide(Snapshot snap, int s_now, int s_prev);
  
  TrajectoryData useful_data(vector<Snapshot> trajectory, map<int, vector <vector<int> > > predictions);
  
  double inefficiency_cost(vector<Snapshot> trajectory, map<int, vector <vector<int> > > predictions, TrajectoryData data);
  
  double buffer_cost(vector<Snapshot> trajectory, map<int, vector <vector<int> > > predictions, TrajectoryData data);
  
  double collision_cost(vector<Snapshot> trajectory, map<int, vector <vector<int> > > predictions, TrajectoryData data);
  
  double calculate_cost(vector<Snapshot> trajectroy, map<int, vector <vector<int> > > predictions);
  
  void configure(vector<int> road_data);

  void increment(int dt);

  vector<int> state_at(int t);

  void realize_state(map<int, vector < vector<int> > > predictions);

  void realize_constant_speed();

  int _max_accel_for_lane(map<int,vector<vector<int> > > predictions, int lane, int s);

  void realize_keep_lane(map<int, vector< vector<int> > > predictions);

  void realize_lane_change(map<int,vector< vector<int> > > predictions, string direction);

};

#endif