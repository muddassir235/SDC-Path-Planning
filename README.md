# CarND-Path-Planning-Project
###Implementation

####Overview:

The implementation was done in 3 Steps (each step depends on the previous):

* Smooth Path Generation
* Driving in a given lane
* Switching lanes when needed.

####Smooth Path Generation:

Smooth paths where generated using the spline tool as suggested in the classroom. By following the Walkthrough given in the classroom I was able to generate a smooth path consisting of 50 points ahead of the vehicle. The points are split such that at all steps the vehicle stays under the speed and acceleration limits. 

The path generation is done in the following steps

* Set anchor points for the spline (The start point/reference will initially be the car's coordinates and after some trajectory generation has been done will be the last point in the previous trajectory). **Note:** The start yaw can be calculated using the start anchor point and a point before it.

  ```C++
  refyaw = atan2(refy-refy_prev, refx-refx_prev);
  ```

  Three more anchor points are generated at `30`, `60` and `90` meters from the start point.

  ```C++
  vector<double> anchor_pt = getXY(
    car_s+ d /* meters ahead */, 2+LANE_WIDTH*lane, 
    map_waypoints_s, map_waypoints_x, map_waypoints_y
  );
  ```

* The anchor points are converted from global `(x,y)` Coordinates to the vehicle's local coordinates.

  ```C++
  /* Tranform from global coordinates to the car's local coordinates */
  vector<double> tolocal(double x, double y, double refx, double refy, double refyaw){
    vector<double> local;

    double shift_x = x - refx;
    double shift_y = y - refy;

    local.push_back(/* x */ (shift_x*cos(0-refyaw) - shift_y*sin(0-refyaw)) /* x */);
    local.push_back(/* y */ (shift_x*sin(0-refyaw) + shift_y*cos(0-refyaw)) /* y */);

    return local;
  }	
  ```

* Check whether there are previous points left over from the last trajectory generation and append them to the new list.

  ```C++
  for(int i = 0; i < previous_path_x.size(); i++){
     next_x_vals.push_back(previous_path_x[i]);
     next_y_vals.push_back(previous_path_y[i]);
  }
  ```

* The anchor points are used to make a spline and then for the set of 50 points for the path are completed using it.

  ```C++
  spline.set_points(ptsx, ptsy); /* Setting Anchor points */
  .....
  double target_x = 30; 
  double target_y = spline(target_x)
  double target_dist = sqrt(pow(target_x,2) + pow(target_y,2)) /* m */;

  ..... // Generation loop 
  {
      double x_point = x_add_on+target_x/N /* m */;
      double y_point = s(x_point) /* m */;
    
    	.......// To global coordinates and add them to the trajectory.
  }
  ```



####Driving in a given Lane:

In order to drive in a given lane we have to avoid collisions with other vehicles in front of us. For that we use the sensor fusion data to extract the vehicle in front of us and then if it is closer to us that a certain range we slow down.

```C++
.......// Inside search loop {
  
  ....... // Other stuff
  
  if((check_s>car_s) && (check_s<(car_s+25))){
    front_car_exists = true;
    front_car_speed = check_speed;
    front_car_s = check_s;
  }
}

......
  
if(front_car_exists){
  double speed_diff = ref_vel - front_car_speed;
  if(speed_diff<MAX_ACCEL){
    ref_vel = front_car_speed /* Drive at the speed of the car in front. */;
  }else{
    acceleration = MAX_ACCEL * (1 - exp(-speed_diff));
    ref_vel -= acceleration /* Slow down */;
  } 
}
```

Moreover if we are accelerating from stand-still, it should smooth such that we don't exceed out acceleration and jerk limits.

```C++
if(ref_vel<SPEED_LIMIT /* Under our desired speed */){
  acceleration = MAX_ACCEL * (1 - exp(-(SPEED_LIMIT - ref_vel))) /* Smooth decay */;
  ref_vel+=acceleration;
  if(ref_vel>SPEED_LIMIT){
    ref_vel = SPEED_LIMIT;
  }
}
```



####Switching Lanes when needed:

In order to decide which lane is the best for us we run through the following steps:

* We predict the positions of the vehicles around us.

  ```c++
  /* Get predictions for a certain car */
  vector<vector<double>> getpred(GNB gnb, vector<double> car, int horizon, vector<double> map_waypoints_x, vector<double> map_waypoints_y){
    int lane = (int)(car[D])/4;
    
    ...... // Some code for prediction I didn't use.
    
    double car_speed = sqrt(car[VX]*car[VX] + car[VY]*car[VY]);

    vector<vector<double>> predictions;
    for(int i=0; i<horizon; i++){
      predictions.push_back({(init_car_s+i*car_speed), (double)lane_, car_speed, car[ID]});
    }
    return predictions;
  }
  ```

* We then split these predictions in their respective lanes. This will allows us to calculate the cost for each lane independently. The predictions are also separated based on the time-step of the prediction 

  > The state of all the vehicle at a certain time step is being referred to as **image** here. 

  ```C++
  int get_best_lane(.../* Other params */..., map<int, vector<vector<double>>> predictions)
    vector<vector<vector<double>>> preds_list /* Covert the map into a list */;
    
    .....// Conversion
      
    /* Images corresponding to each timestep */
    vector<vector<vector<double>>> preds_image_list;
   
    .....// Conversion

    vector<vector<vector<double>>> lane0;
    vector<vector<vector<double>>> lane1;
    vector<vector<vector<double>>> lane2;

    for(/* loop over time images */){
      
      ......
        
      for(int j=0; j<image.size();j++){
        if(snap[1 /* lane */] == 0){
          lane0_image.push_back({snap[0 /* s */], snap[2 /* ds */], snap[3 /* ID */]});
        }else if(snap[1 /* lane */] == 1){
          lane1_image.push_back({snap[0 /* s */], snap[2 /* ds */], snap[3 /* ID */]});
        }else{
          lane2_image.push_back({snap[0 /* s */], snap[2 /* ds */], snap[3 /* ID */]});
        }
      }
      
      lane0.push_back(lane0_image) /* lane 0 timestep images */;
      lane1.push_back(lane1_image) /* lane 1 timestep images */;
      lane2.push_back(lane2_image) /* lane 2 timestep images */;
  }

  ..... // Iterate over list of possible lanes
    
  /* 
   * For each possible lane calculate cost using ```get_lane_cost()``` 
   * and choose the best one. 
   */
  ```

* The we calculate the cost for each  lane and choose the best one.

  ```C++
  /* Cost penalties */
  constexpr double COLLISSION_COST = 1000000;
  constexpr double BUFFER_COST = 10000;
  constexpr double INEFFICIENCY_COST = 1000;
  constexpr double CONJETION_COST = 100;

  /* Get the cost for a certain lane */
  double get_lane_cost(lane_data, curr_s, curr_ds){
    
    double start_s = curr_s;

    /* Initialize all costs */
    double collision_cost = 0.0;
    double buffer_cost = 0.0;
    double inefficiency_cost = 0.0;
    double conjetion_cost = 0.0;
    
    for(/* loop over time images of the current lane */){

      double s = start_s + curr_ds*i /* increment velocity in seconds */;
   
      for(/* loop over all the vehicle at a certain time image */){
   	 if(/* vehicle is in front of us */){
          if(/* vehicle is the closest to us yet */){
            front_vehicle_s = vehicle[0 /* s */];
          }
        }else if(/* vehicle is behind us */){
          if(/* is the closest from behind */){
            rear_vehicle_s = vehicle[0 /* s */];
          }
        }else{
          collision_cost+=COLLISSION_COST /* COLLISION */;
        }
      }

      if(/* Front vehicle closer that 15 meters */){
        collision_cost+=COLLISSION_COST;
      }else if(/* Otherwise if front vehicle is closer than 25 meters*/){
        buffer_cost+=BUFFER_COST;
        pct = (SPEED_LIMIT - front_vehicle velocity)/SPEED_LIMIT;
        mx = pow(pct, 2);
        inefficiency_cost+=mx*INEFFICIENCY_COST /* Adding inefficiency cost 
                                                 * according to the speed of 
                                                 * the vehicle in front. */;
      }else{
        gap = front_vehicle_s - s; 
        conjetion_cost+=CONJETION_COST*exp(-0.05*gap);
      }


      if(/* Rear vehicle is closer than 10 meters */){
        collision_cost+=COLLISSION_COST;
      }else if(/* Otherwise if rear vehicle is closer than 20 meters */){
        buffer_cost+=BUFFER_COST;
      }
    }
  }
  ```

  ​

### Simulator.

You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

