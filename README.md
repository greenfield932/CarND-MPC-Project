# Model Predictive Control with latency

This projects describes model predictive control for the vehicle and allows to perform vehicle actuations (steering and throttle) to follow the predefined trajectory, taking in account actuators latency.

## The Model

[image1]: ./images/model.png
[image2]: ./images/constraints.png
[image3]: ./images/metrics.png
[image4]: ./images/short_dt.gif
[image5]: ./images/long_dt.gif
[image6]: ./images/output_video.gif

The model of a vehicle based on global kinematic model:

![alt text][image1]

where x,y - position of the vehicle, psi - orientation of the vehicle, V - speed, delta - steering value, a - trhottle value, Lf - distance between center of gravity and front axle of the vehicle.

In this model steering value and throttle value are actuations applied to the vehicle. The have the following constraints:

![alt text][image2]

To perform model predictive control the follwing metrics were used:

![alt text][image3]

f(x) - y coordinate value for x based on polynomial approximation of the desired trajectory, cte - cross track error, epsi - orientation error

x,y,psi,v,cte,epsi - define a state of the vehicle.

## Timestep Length and Elapsed Duration (N & dt)

N is a tmestep length, dt is elapsed duaration. These parameters define prediction horizon T = N*dt (predicted trajectory for the next T seconds from the current state).
Combination of these parameters should define a reasonable future trajectory. I finished with N=10 and dt=0.1. 
Larger dt results to smooth turns, which is reasonable on higher speeds:

![alt text][image5]

while shorter dt results to less cross track error:

![alt text][image4]

N and dt both define the length of prediction horizon, dt parameter defines how often actuations calculated for each point of the trajectory,
while N defines amount of such calculations and influence on size of optimization problem and solver performance as well.

## Polynomial Fitting and MPC Preprocessing

Polynomial fitting performed on the desired trajectory. This requred to calculate cross track error and orientation error.

Preprocessing include transform x,y,psi values from map coordinate system to the vehicle's coordinate system, so the first point became the origin 
and all next points coordinates recalculated from this point. 
Next rotate all points on 90 degree to aligh zero heading with X axis. This mostly done for optimization solver and polynomial fitting (i.e. numeric optimizations)

## Model Predictive Control with Latency

In real world applications it is often impossible to apply actuations without latency. As result model will not operate properly if not take in account such latencies.
One approach would be running a simulation using the vehicle model starting from the current state for the duration of the latency.
The resulting state from the simulation is the new initial state for MPC.


Here's a [link to my video result](./output_video.mp4)

---

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

