# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

[//]: # (Image References)

[image1]: ./output_imgs/output4.png "Result"

![alt text][image1]

## Video Demo

Here is the MPC ran on [Udacity simulator](https://github.com/udacity/self-driving-car-sim/releases) with 800x600 screen resolution and good graphics quality with 50 MPH.

[Demo Link](https://youtu.be/0U6WFtklg9Y)

## Implementation

### The Model

Global Kinematic Model which is simplifications of dynamic models that ignore tire forces, gravity, and mass.    
This simplification reduces the accuracy of the models, but it also makes them more tractable.    
 
It includes following state, actuators and update equations.    

State: [x,y,ψ,v]    
x,y for vehicle position    
ψ for vehicle orientation    
v for vehicle velocity    

Actuators: [δ,a]    
δ for steering angle.    
a for acceleration (throttle/brake combined).    

Update equations:    

x_[t+1] = x[t] + v[t] * cos(ψ[t]) * dt    
y_[t+1] = y[t] + v[t] * sin(ψ[t]) * dt    
ψ_[t+1] = ψ[t] + v[t] / Lf * delta[t] * dt    
v_[t+1] = v[t] + a[t] * dt    

Lf measures the distance between the front of the vehicle and its center of gravity.     
The larger the vehicle, the slower the turn rate.    

And I also use Cross Track Error (cte) and Orientation Error (epsi) as additional state.    
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt    
epsi[t+1] = ψ[t] - psides[t] + v[t] * delta[t] / Lf * dt    


### Timestep Length and Frequency

N is the number of timesteps in the horizon.     
dt is how much time elapses between actuations.    
And T is the product of two variables, N and dt.    

The goal of Model Predictive Control is to optimize the control inputs: [δ,a].     
An optimizer will tune these inputs until a low cost vector of control inputs is found.     
However, the environment will change enough that it won't make sense to predict any further into the future.    

Larger values of dt result in less frequent actuations, which makes it harder to accurately approximate a continuous reference trajectory.     

Based on above rules, I choose N = 25 and dt = 0.05 at the beginning.    
However, large N is not necessary and too many vector will lead the optimizer to bad result.    
I decrease N from 25 to 15 and keep dt = 0.05 as my final choice.    


### Polynomial Fitting and MPC Preprocessing

Polynomial Fitting:    
First, I transformed from map coordinate into the vehicle coordinate system.    
And then, I fitted the transformed waypoints with third polynomial.    

MPC Preprocessing:    
For the initial vector, the initial vehicle position, orientation is zero because it's on vehicle coordinate system.    
And I used the fitted polynomial with x=0, y=0 to find initial Cross Track Error, Orientation Error.    



### Model Predictive Control with Latency

In a real car, an actuation command won't execute instantly - there will be a delay as the command propagates through the system. A realistic delay might be on the order of 100 milliseconds.    

This is a problem called "latency", and it's a difficult challenge for some controllers to overcome. But a Model Predictive Controller can adapt quite well because we can model this latency in the system.    

In the main.cpp.    
We using 100 millisecond latency.     
```
this_thread::sleep_for(chrono::milliseconds(100));
```    
It means MPC solver is try to optimize the data is 100ms earlier.    

To solve this issue, we can predict the state after 100ms as the input state for the solver.    
For example, x = 0 + v * latency for the state after 100ms.    



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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
  * If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt --with-openblas`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).



## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

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

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
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
