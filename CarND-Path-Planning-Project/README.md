
# CarND-Path-Planning-Project-P1
Udacity Self-Driving Car Nanodegree - Path Planning Project


# Overview

In this project, we need to implement a path planning algorithms to drive a car on a highway on a simulator provided by Udacity([the simulator could be downloaded here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2)). The simulator sends car telemetry information (car's position and velocity) and sensor fusion information about the rest of the cars in the highway (Ex. car id, velocity, position). It expects a set of points spaced in time at 0.02 seconds representing the car's trajectory. The communication between the simulator and the path planner is done using [WebSocket](https://en.wikipedia.org/wiki/WebSocket). The path planner uses the [uWebSockets](https://github.com/uNetworking/uWebSockets) WebSocket implementation to handle this communication. Udacity provides a seed project to start from on this project ([here](https://github.com/udacity/CarND-Path-Planning-Project)).
![simulator](img/framework_small.png)

# Prerequisites

The project has the following dependencies (from Udacity's seed project):

- cmake >= 3.5
- make >= 4.1
- gcc/g++ >= 5.4
- libuv 1.12.0
- Udacity's simulator.

For deitled insturuction follows the instruction on https://github.com/udacity/CarND-Path-Planning-Project.

# Compiling and executing the project

In order to build the project there is a `./build.sh` script on the repo root. It will create the `./build` directory and compile de code. This is an example of the output of this script:

```
> mkdir buil && cd build
> camke ..
> make
-- The C compiler identification is AppleClang 8.0.0.8000042
-- The CXX compiler identification is AppleClang 8.0.0.8000042
-- Check for working C compiler: /Library/Developer/CommandLineTools/usr/bin/cc
-- Check for working C compiler: /Library/Developer/CommandLineTools/usr/bin/cc -- works
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Detecting C compile features
-- Detecting C compile features - done
-- Check for working CXX compiler: /Library/Developer/CommandLineTools/usr/bin/c++
-- Check for working CXX compiler: /Library/Developer/CommandLineTools/usr/bin/c++ -- works
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Detecting CXX compile features
-- Detecting CXX compile features - done
-- Configuring done
-- Generating done
-- Build files have been written to: REPO_ROOT/CarND-Path-Planning-Project-P1/build
Scanning dependencies of target path_planning
[ 50%] Building CXX object CMakeFiles/path_planning.dir/src/main.cpp.o
[100%] Linking CXX executable path_planning
[100%] Built target path_planning
```

The project could be executed directly using `./build/path_planning`

```
> ./path_planning
Listening to port 4567
```

Now the path planner is running and listening on port 4567 for messages from the simulator. Next step is to open Udacity's simulator:


# [Rubic](https://review.udacity.com/#!/rubrics/1020/view) points

## Compilation

### The code compiles correctly.

The code comples correctly.

## Valid trajectories

### The car is able to drive at least 4.23 miles without incident.
I ran the simulator for 5 miles without incidents:

![5 miles](img/pathplanning2.png)


### The car drives according to the speed limit.
The car does not exceed 50 m/hr speed limit.

### Max Acceleration and Jerk are not Exceeded.
The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.

### Car does not have collisions.
No collision detected.

### The car stays in its lane, except for the time between changing lanes.
The car stays in its lane all the time except during lane change manuver.

### The car is able to change lanes
The Ego vehice changes lane only if there is an empty lane either on the right or left lane and the vehicle ahead is slowing the Ego vehcile.

## Reflection

The path planning module is composed of prediction, behaviour planning, and trajectory generation functionalties.

The prediction moudle estimates the possible action other vehicles may take by fusing sensor information and map data. In this project I assumed three possible possible future postions for vehicles in the vicitnity (within 30m) of the Ego car, i.e., the vehicles can be either ahead, rihgt or left of the Ego vehicle (Lines 112-157 ). 

The Behaviour planning module determines the action the ego vehicle needs to take to reach a destination while meeting a constrint (traffice rule, speed limit, ...). In general this moulde is the most challagine and crutical part of every autonomus vehicle. For this project only five behviours are defined: "keep lane", "turn left", "turn right", "speed up" and "speed down". The behaviours are determined by taking into account the prediction of other vehicles, the current lane, and current speed of the Ego vehicle (Lines 158-182 )


The trajectory generation moudle computes intermediate points between the current and immediate desired target locations based on the behaviour selected (Lines 184-289). To make the trajecory smooth, at every iteration we keep the last two previous points and look 30m, 60m and 90m ahead to generate the the spline function (Lines 184-221). By selecting N=50 equal distace points in the S direction, we compute the coorsponding d values using the spline function. To make this calculation s easier,  all points are transfomred into the Ego local coordinate (Lines 223-237)  before they are feed to the spline function and connveted back to golbal coordinate after the calculation (Lines 280-281).


```python

```
