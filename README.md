# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Overview

This project is an implementation of MPC algorithm in C++ to steer a car around the track in Udacity simulator. Using the algorithm, the virtual car can easily and safely ride the driveable portion of the track with realtively high speed.

## Implementation Description

### The Model
I have used a simple kinematic model, meaning that I have ignored tire forces, gravity and mass. For sure, this results in lack of the accuracy, but the model is much more tractable. For reasonable speeds, simple kinematic model shows pretty good results.

The state at time t of the model is described with 6 variables:

* 'x' and 'y' - position of the car
* 'psi' - heading direction of the car
* 'v' - velocity of the car
* 'cte' - cross track error
* 'epsi' - orientation error

In addition the the state, Udacity has provided a constant 'Lf' which is the distance between the car of mass and the front wheels 

The equations for updating the state of model are the following:
![](/result_img/model_equations.png "Model Equations")

The model output variables are:

* 'a' - throttle (acceleration of the car)
* 'delta' - steering angle

### Timestep Length and Elapsed Duration (N & dt)
Prediction horizon is defined by two variables:

* N - the number of points
* dt - the time interval

As the number of points impacts the controller performance, I tried to keep them as low as possible. I have taken proposed in lesson values (N=10, dt=0.1), but to see exactly, what is the influence, I played with those values, incresing/decreasing them.

With increasing 'N' increased the computational cost, the algorithm runs slower and predicts too much unnecessary data for the future. With decrease of N, algorithm can not predict enough data for the future and this cause instabilities, especially on sharp turns.

'dt' parameter is even easier. When I decreased it, the car started to respond slower than it got new actions. With increase of 'dt', the car responded lately to state, which required immediate actions (speaking about same sharp turns).

As a result, I decided to used proposed to use provided values (N=10, dt=0.1) and do further tuning via penalty weights.

### Polynomial Fitting and MPC Preprocessing
Before predicting the next state, waypoints needed to be proprocessed. The process of fitting a polynomial to the waypoints here is advised to be used on vehile coordinates, while I got them in global coordinates. For the required conversion I've used the following formulas:

```
X_vehicle = X_global ∗ cos(​theta) - Y_global * sin(theta)
Y_vehicle = X_global ∗ sin(​theta) + Y_global * cos(theta)
```

where 'X_vehicle' and 'Y_vehicle' are the desired coordinates in vehicle space, 'X_global' and 'Y_global' are the input coordinates in world space, and 'theta' is the angle between the world x-axis and the car x-axis about the world coordinates z-axis, provided it is perpendicular to the car frames x-axis.

### Model Predicted Control with Latency
To deal with actuator latency, state values are computed using the model and delay interval (100ms). These values are used instead of the initial. Without this optimisation, algorithm built bad trajectories and showed oscillations. Then result state was passed to the MPC solver.

### Additional Parameter Tuning
As I have said earlier, I decided to leave N and dt params as provided and tune penalty weights in order to get max accurate results.

I think, there are numerous variants of different param sets to get good results here. My tuning results are as following:

title | value
--- | --- | ---
cross track error penalty weight | 2000 
orientation error penalty weight | 2000 
delta error penalty weight | 10 
speed penalty weight | 10
delta diff error penalty weight | 100
speed diff error penalty weight | 10

The first two values were used as a part of the cost based on the reference state. The next two were used to minimise the use of actuators, and the last two - minimise the value gap between sequential actuations, to result in smoother transaction.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
