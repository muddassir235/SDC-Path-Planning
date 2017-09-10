#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "classifier.h"
#include "classifier.cpp"
#include <algorithm>
  
using namespace std;

// for convenience
using json = nlohmann::json;

constexpr double pi() { return M_PI; }
constexpr double MPH_TO_MPS = 2.23694;
constexpr double SIMULATOR_PERIOD = 0.02 /* seconds */;
constexpr double LANE_WIDTH = 4.0 /* m */;
constexpr double SPEED_LIMIT = 49.5 /* mph */;
constexpr double TRACK_LENGTH = 6945.554 /* m */;
constexpr double MAX_ACCEL = 0.224;
constexpr double START_S = 124.834 /* m */;

constexpr double COLLISSION_COST = 1000000;
constexpr double BUFFER_COST = 10000;
constexpr double INEFFICIENCY_COST = 1000;
constexpr double CONJETION_COST = 100;
constexpr double LANE_CHANGE_COST = 20;

constexpr int ID = 0;
constexpr int X = 1;
constexpr int Y = 2;
constexpr int VX = 3;
constexpr int VY = 4;
constexpr int S = 5;  
constexpr int D = 6;

// For converting back and forth between radians and degrees.
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.      
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};
}

/* Tranform from global coordinates to the car's local coordinates */
vector<double> tolocal(double x, double y, double refx, double refy, double refyaw){
  vector<double> local;

  double shift_x = x - refx;
  double shift_y = y - refy;

  local.push_back(/* x */ (shift_x*cos(0-refyaw) - shift_y*sin(0-refyaw)) /* x */);
  local.push_back(/* y */ (shift_x*sin(0-refyaw) + shift_y*cos(0-refyaw)) /* y */);

  return local;
}

/* Transform a list of global (x,y) coordinates to the car's local coordinates */
vector<vector<double>> listtolocal(vector<double> xs, vector<double> ys, double refx, double refy, double refyaw){
  vector<vector<double>> local;

  vector<double> localxs;
  vector<double> localys;

  for(int i=0; i<xs.size(); i++){
    vector<double> transformed = tolocal(xs[i], ys[i], refx, refy, refyaw);
    localxs.push_back(transformed[0] /* x */);
    localys.push_back(transformed[1] /* y */);
  }

  local.push_back(localxs);
  local.push_back(localys);

  return local;
}

/* Tranform car's local coordinates to map's global coordinates. */
vector<double> toglobal(double x, double y, double refx, double refy, double refyaw){
  vector<double> global;

  global.push_back(/* x */ refx + (x*cos(refyaw) - y*sin(refyaw)) /* x */);
  global.push_back(/* y */ refy + (x*sin(refyaw) + y*cos(refyaw)) /* y */);

  return global;
}

vector<vector<double> > Load_State(string file_name)
{
    ifstream in_state_(file_name.c_str(), ifstream::in);
    vector< vector<double >> state_out;
    string line;
    
    
    while (getline(in_state_, line)) 
    {
        istringstream iss(line);
      vector<double> x_coord;
      
      string token;
      while( getline(iss,token,','))
      {
          x_coord.push_back(stod(token));
      }
      state_out.push_back(x_coord);
    }
    return state_out;
}
vector<string> Load_Label(string file_name)
{
    ifstream in_label_(file_name.c_str(), ifstream::in);
    vector< string > label_out;
    string line;
    while (getline(in_label_, line)) 
    {
      istringstream iss(line);
      string label;
      iss >> label;
    
      label_out.push_back(label);
    }
    return label_out;
    
}

/* Get predictions for a certain car */
vector<vector<double>> getpred(GNB gnb, vector<double> car, int horizon, vector<double> map_waypoints_x, vector<double> map_waypoints_y){
  int lane = (int)(car[D])/4;
  vector<double> coords;
  coords.push_back(car[S]);
  coords.push_back(car[D]);

  double car_yaw = atan2(car[VY], car[VX]);
  vector<double> frenet_speed = getFrenet(car[VX], car[VY], car_yaw, map_waypoints_x, map_waypoints_y);
  coords.push_back(frenet_speed[0 /* s_dot */]);
  coords.push_back(frenet_speed[1 /* d_dot */]);

  string prediction = gnb.predict(coords);

  int new_lane = lane;

  // if((prediction.compare("left") == 0) && lane!=0){
  //   new_lane--;
  // }else if((prediction.compare("right")) && lane!=2){
  //   new_lane++;
  // }

  double init_car_s = car[S];
  double car_speed = sqrt(car[VX]*car[VX] + car[VY]*car[VY]);

  vector<vector<double>> predictions;
  for(int i=0; i<horizon; i++){
    int lane_ = lane;
    if(i>1){  
      lane_ = new_lane;
    }
    predictions.push_back({(init_car_s+i*car_speed), (double)lane_, car_speed, car[ID]});
  }
  return predictions;
}

/* Get predictions for all vehicles */
map<int, vector<vector<double>>> get_predictions(GNB gnb, vector<vector<double>> sensor_fusion, vector<double> map_waypoints_x, vector<double> map_waypoints_y){
  map<int, vector<vector<double>>> predictions; 
  for(int i=0;i<sensor_fusion.size();i++){
    vector<vector<double>> prediction = getpred(gnb, sensor_fusion[i], 1 , map_waypoints_x, map_waypoints_y);
    predictions.insert(std::pair<int,vector<vector<double>>>(sensor_fusion[i][ID],prediction));
  }
  return predictions;
}

/* Get the cost for a certain lane */
double get_lane_cost(int lane_index, vector<vector<vector<double>>> lane_data, double curr_s, double curr_ds){
  double start_s = curr_s;

  double collision_cost = 0.0;
  double buffer_cost = 0.0;
  double inefficiency_cost = 0.0;
  double conjetion_cost = 0.0;
  
  vector<map<int, bool>> infront_list;
  vector<vector<int>> ids_list;

  bool collision_happened = false;

  for(int i=0; i<lane_data.size(); i++){
    map<int, bool> infront;
    vector<int> ids;

    double s = start_s+curr_ds*i;
  
    vector<vector<double>> curr_time_image = lane_data[i];
  
    int front_vehicle_index = -1;
    int rear_vehicle_index = -1;
    double front_vehicle_s = 999999;
    double rear_vehicle_s = -999999;
    double front_vehicle_ds = -1;
    double rear_vehicle_ds = -1;

    for(int j=0; j<curr_time_image.size(); j++){
      vector<double> vehicle = curr_time_image[j];
      ids.push_back(vehicle[2 /* ID */]);
  
      if(s<vehicle[0 /* s */]){
        if(vehicle[0 /* s */]<front_vehicle_s){
          front_vehicle_s = vehicle[0 /* s */];
          front_vehicle_ds = vehicle[1 /* ds */];
          front_vehicle_index = j;
        }
        infront.insert(pair<int, bool>(vehicle[2 /* ID */], true));
      }else if(s>vehicle[0 /* s */]){
        if(vehicle[0 /* s */]>rear_vehicle_s){
          rear_vehicle_s = vehicle[0 /* s */];
          rear_vehicle_ds = vehicle[1 /* ds */];
          rear_vehicle_index = j;
        }
        infront.insert(pair<int, bool>(vehicle[2 /* ID */], false));
      }else{
        collision_cost+=COLLISSION_COST;
        collision_happened = true;
      }
    }

    infront_list.push_back(infront);
    ids_list.push_back(ids);

    bool front_vehicle_exists = front_vehicle_index != -1;

    if(front_vehicle_exists){
      if((front_vehicle_s-s) < 15){
        collision_cost+=COLLISSION_COST;
        collision_happened = true;
      }else if((front_vehicle_s-s)<30){
        buffer_cost+=BUFFER_COST;
        double diff =  SPEED_LIMIT - front_vehicle_ds*MPH_TO_MPS;
        double pct = diff/SPEED_LIMIT;
        double mx = pow(pct, 2);
        inefficiency_cost+=mx*INEFFICIENCY_COST;
      }else{
        double gap = front_vehicle_s - s; 
        conjetion_cost+=CONJETION_COST*exp(-0.05*gap);
      }
    }

    bool rear_vehicle_exists = rear_vehicle_index != -1;

    if(rear_vehicle_exists){
      if((s-rear_vehicle_s) < 15){
        collision_cost+=COLLISSION_COST;
        collision_happened = true;
      }else if((s-rear_vehicle_s)<20){
        buffer_cost+=BUFFER_COST;
      }

      if((rear_vehicle_ds-curr_s)>5){
        buffer_cost+=BUFFER_COST;
      }
    }

  }

  map<int, bool> prev_map = infront_list[0];
  vector<int> prev_ids = ids_list[0];
  for(int i=1; i<infront_list.size(); i++){
    vector<int> common;
    vector<int> curr_ids = ids_list[i];
    map<int, bool> curr_map = infront_list[i];
    for(int j=0; j<prev_ids.size(); j++){
      if(std::find(curr_ids.begin(), curr_ids.end(), prev_ids[j]) != curr_ids.end()) {
        common.push_back(prev_ids[j]);
      }
    }

    for(int j=0; j<common.size(); j++){
      int id = common[j];
      if(prev_map.find(id)->second!=curr_map.find(id)->second){
        collision_cost+=COLLISSION_COST;
        collision_happened = true;
      } 
    }
    prev_map = curr_map;
    prev_ids = curr_ids;
  }

    // if(collision_happened){
    //   cout<<"("<<lane_index<<") "<<"COLLISSION_COST: "<<collision_cost<<endl;
    // }

  return collision_cost+buffer_cost+inefficiency_cost+conjetion_cost;
}

vector<int> get_possible_lanes(int curr_lane){
  if(curr_lane == 0){
    return {0, 1};
  }else if(curr_lane == 1){
    return {0, 1, 2};
  }else{
    return {1, 2};
  }
}

int get_best_lane(double curr_s, double curr_ds, int curr_lane, map<int, vector<vector<double>>> predictions){
  vector<int> possible_lanes = get_possible_lanes(curr_lane);

  vector<vector<vector<double>>> preds_list;

  map<int, vector<vector<double>>>::iterator it = predictions.begin();

  while(it != predictions.end()){
    vector<vector<double>> v = it->second;
    preds_list.push_back(v);
    it++;
  }

  vector<vector<vector<double>>> preds_image_list;

  for(int i=0;i<preds_list[0].size(); i++){
    vector<vector<double>> image;
    for(int j=0; j<preds_list.size(); j++){
      vector<double> vehicle_snap = preds_list[j][i];
      image.push_back(vehicle_snap);
    }
    preds_image_list.push_back(image);
  }

  vector<vector<vector<double>>> lane0;
  vector<vector<vector<double>>> lane1;
  vector<vector<vector<double>>> lane2;

  for(int i=0; i<preds_image_list.size(); i++){
    vector<vector<double>> image = preds_image_list[i];
    vector<vector<double>> lane0_image;
    vector<vector<double>> lane1_image;
    vector<vector<double>> lane2_image;
    for(int j=0; j<image.size();j++){
      vector<double> snap = image[j]; 
      if(snap[1 /* lane */] == 0){
        lane0_image.push_back({snap[0 /* s */], snap[2 /* ds */], snap[3 /* ID */]});
      }else if(snap[1 /* lane */] == 1){
        lane1_image.push_back({snap[0 /* s */], snap[2 /* ds */], snap[3 /* ID */]});
      }else{
        lane2_image.push_back({snap[0 /* s */], snap[2 /* ds */], snap[3 /* ID */]});
      }
    }
    lane0.push_back(lane0_image);
    lane1.push_back(lane1_image);
    lane2.push_back(lane2_image);
  }

  vector<vector<vector<vector<double>>>> lane_data;
  lane_data.push_back(lane0);
  lane_data.push_back(lane1);
  lane_data.push_back(lane2);

  vector<double> lane_costs;

  for(int i=0;i<possible_lanes.size(); i++){
    vector<vector<vector<double>>> curr_lane_data = lane_data[possible_lanes[i]];
    lane_costs.push_back(get_lane_cost(possible_lanes[i],curr_lane_data, curr_s, curr_ds));
  }

  for(int i=0;i<possible_lanes.size(); i++){
    if(possible_lanes[i]!=curr_lane){
      lane_costs[i]+=LANE_CHANGE_COST;
    }
  }

  int curr_lane_index = 0;
  for(int i=0; i<possible_lanes.size(); i++){
    if(possible_lanes[i] == curr_lane){
      curr_lane_index = i;
      break;
    }
  }

  int best = curr_lane_index;

  for(int i=0; i<lane_costs.size(); i++){
    if(lane_costs[i]<lane_costs[best]){
      best = i;
    }
  }

  return possible_lanes[best];
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  /* Loading Training Data for Prediction */
  vector< vector<double> > X_train = Load_State("./train_states.txt");
  vector< string > Y_train  = Load_Label("./train_labels.txt");

  GNB gnb = GNB();

  /* Training the Gaussian Naive Bais Classifier */
  gnb.train(X_train, Y_train);

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";


  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  double ref_vel = 0.0 /* The speed the car should move at */;
  double lane = 1 /* The starting lane of the car. */;
  bool changing_lanes = false;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&ref_vel, &lane, &gnb, &changing_lanes](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {  
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
        	double car_x = j[1]["x"];
        	double car_y = j[1]["y"];
        	double car_s = j[1]["s"];
        	double car_d = j[1]["d"];
        	double car_yaw = j[1]["yaw"];
        	double car_speed = j[1]["speed"];

        	// Previous path data given to the Planner
        	auto previous_path_x = j[1]["previous_path_x"];
        	auto previous_path_y = j[1]["previous_path_y"];
        	// Previous path's end s and d values 
        	double end_path_s = j[1]["end_path_s"];
        	double end_path_d = j[1]["end_path_d"];

        	// Sensor Fusion Data, a list of all other cars on the same side of the road.
        	auto sensor_fusion = j[1]["sensor_fusion"];

          /* The size of points left (not utilized) from the previous path */
          int prev_size = previous_path_x.size();

          if(((int)(car_d)/4) == lane){
            changing_lanes = false;
          }

          if(prev_size>0 /* some points in the previous path */) {
            car_s = end_path_s;
          }

          double front_car_exists = false /* Imaginary front car traveling just under speed limit. */;
          double front_car_speed;
          double front_car_s;

          for(int i=0; i<sensor_fusion.size(); i++){
            double d = sensor_fusion[i][D];
            if(d<(2+LANE_WIDTH*lane+2) && d>(2+LANE_WIDTH*lane-2) /* Vehicle is in our lane */){
              double vx = sensor_fusion[i][VX];
              double vy = sensor_fusion[i][VY];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_s = sensor_fusion[i][S];

              check_s+=check_speed*(SIMULATOR_PERIOD*prev_size);

              if((check_s>car_s) && (check_s<(car_s+25))){
                front_car_exists = true;
                front_car_speed = check_speed;
                front_car_s = check_s;
              }
            }
          }

          double acceleration = 0;

          if(front_car_exists){
            front_car_speed*=MPH_TO_MPS;
            double speed_diff = ref_vel - front_car_speed;
            if(speed_diff<MAX_ACCEL){
              ref_vel = front_car_speed;
            }else{
              acceleration = MAX_ACCEL * (1 - exp(-3*speed_diff));
              if(speed_diff>12){
                acceleration = 1;
              }
              ref_vel -= acceleration;
            } 
          }else{
            if(ref_vel<SPEED_LIMIT){
              acceleration = MAX_ACCEL * (1 - exp(-(SPEED_LIMIT - ref_vel)));
              ref_vel+=acceleration;
              if(ref_vel>SPEED_LIMIT){
                ref_vel = SPEED_LIMIT;
              }
            }
          }

        
          if(!changing_lanes){
            map<int, vector<vector<double>>> preds = get_predictions(gnb, sensor_fusion, map_waypoints_x, map_waypoints_y); 
            int new_lane = get_best_lane(car_s-17.65, car_speed, lane, preds);
            if(new_lane != lane){
              changing_lanes=true;
              lane = new_lane;
              cout<<"&&& curr_lane "<<lane<<endl; 
            }
          }



          /* A list of widely spaced (x,y) waypoints. These are spaced at 30m */
          /* Later we will use the spline tool to fill in more waypoints spaced
           * such that for the 20 ms delay (the simulator eats up points at), the
           * car is able to drive just below the speed limit. */
          vector<double> ptsx, ptsy;

          /* Reference x, y and yaw states */
          /* We'll either set the reference point to where the car is or the end
           * end points of the previous path */ 
          double refx = car_x;
          double refy = car_y;
          double refyaw = deg2rad(car_yaw);

          if(prev_size < 2 /* The previous path give by the simulator is empty*/){
            /* Using two points to make the path tangent to the car i.e. avoid any 
             * drastic change in direction */
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }else /* Use the previous path's endpoint as a reference. */ {
            refx = previous_path_x[prev_size-1];
            refy = previous_path_y[prev_size-1];

            double refx_prev = previous_path_x[prev_size-2];
            double refy_prev = previous_path_y[prev_size-2];
            refyaw = atan2(refy-refy_prev, refx-refx_prev);

            ptsx.push_back(refx_prev);
            ptsx.push_back(refx);

            ptsy.push_back(refy_prev);
            ptsy.push_back(refy);
          }

          /* Adding 3 evenly 30m spaced points in Frenet coordinates to our path. */
          vector<double> pt0 = getXY(
            car_s+30 /* 30 meters ahead */, 2+LANE_WIDTH*lane, 
            map_waypoints_s, map_waypoints_x, map_waypoints_y
          );
          vector<double> pt1 = getXY(
            car_s+60 /* 60 meters ahead */, 2+LANE_WIDTH*lane, 
            map_waypoints_s, map_waypoints_x, map_waypoints_y
          );
          vector<double> pt2 = getXY(
            car_s+90 /* 90 meters ahead */, 2+LANE_WIDTH*lane, 
            map_waypoints_s, map_waypoints_x, map_waypoints_y
          );

          ptsx.push_back(pt0[0 /* x */]);
          ptsx.push_back(pt1[0 /* x */]);
          ptsx.push_back(pt2[0 /* x */]);

          ptsy.push_back(pt0[1 /* y */]);
          ptsy.push_back(pt1[1 /* y */]);
          ptsy.push_back(pt2[1 /* y */]);

          vector<vector<double>> local = listtolocal(ptsx, ptsy, refx, refy, refyaw);

          ptsx = local[0 /* x */];
          ptsy = local[1 /* y */];

          /* Spline we'll use for making a path of smooth coordinates */
          tk::spline s;

          /* Setting the points the spline should pass through */
          s.set_points(ptsx, ptsy);

          /* The set of points the simulator actally uses */
        	vector<double> next_x_vals;
        	vector<double> next_y_vals;

          /* Start out with the leftover points from the previous path */
          for(int i = 0; i < previous_path_x.size(); i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          /* Calculate how to breakup spline points so that we travel at our desired velocity. */
          double target_x = 30 /* m */;
          double target_y = s(target_x) /* m */;
          double target_dist = sqrt(pow(target_x,2) + pow(target_y,2)) /* m */;

          double N = target_dist/(SIMULATOR_PERIOD * ref_vel / MPH_TO_MPS);

          double x_add_on = 0;

          /* Fill up the remaining points for our path planner to get a total of 50 points. */
          for(int i=0; i<(50-previous_path_x.size()); i++){
            double x_point = x_add_on+target_x/N /* m */;
            double y_point = s(x_point) /* m */;

            x_add_on = x_point;

            vector<double> global = toglobal(x_point, y_point, refx, refy, refyaw);

            next_x_vals.push_back(global[0 /* x */]);
            next_y_vals.push_back(global[1 /* y */]);
          }

          json msgJson;

        	msgJson["next_x"] = next_x_vals;
        	msgJson["next_y"] = next_y_vals;

        	auto msg = "42[\"control\","+ msgJson.dump()+"]";

        	//this_thread::sleep_for(chrono::milliseconds(1000));
        	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
















































































