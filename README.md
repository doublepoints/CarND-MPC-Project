# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

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


## Basic Build Instructions and Running

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
5. Run the Udacity Simulator.

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.


## Model Predictive Control
The MPC is an advanced control technique for complex control problems. MPC is an optmization problem to find the best set of control inputs that minimizes the cost functions based on the prediction (dynamical) model. The MPC controller consists of Prediction Horizon, State and Model, Polynomial Fitting and MPC Preprocessing, Constraints and Latency.

### Prediction Horizon and State
The Prediction Horizom is the duration over which future predictions are made. It contains the number of timesteps N in the horizon and the time elapse of each timestep dt. A higher N will result in extra computational cost. In this project, N is set to 10 and dt is 0.1, which can achieve a best result.
The state consists of system variables and errors.It can be expressed as below:
[x,y,psi,v,cte,epsi]

### Model's Update equations
The followind equations updates the prediction model at every timestep:

![equation](http://latex.codecogs.com/gif.latex?x_%28t&plus;1%29%20%3D%20x_t%20&plus;%20v_t%20*%20cos%28%5Cpsi_t%29*dt)

![equation](http://latex.codecogs.com/gif.latex?y_%28t&plus;1%29%20%3D%20y_t%20&plus;%20v_t%20*%20sin%28%5Cpsi_t%29*dt)

![equation](http://latex.codecogs.com/gif.latex?%5Cpsi%20_%28t&plus;1%29%20%3D%20%5Cpsi%20_t%20&plus;%20%5Cfrac%7Bv_t%7D%7BL_f%7D*%20%5Cdelta_t%20*%20dt)

![equation](http://latex.codecogs.com/gif.latex?v_%28t&plus;1%29%20%3D%20v%20_t%20&plus;%20a_t%20*%20dt)

![equation](http://latex.codecogs.com/gif.latex?cte_%28t&plus;1%29%20%3D%20f%28x_t%29%20-%20y_t%20&plus;%20v%20_t%20*%20sin%28e%5Cpsi%20_t%29%20*%20dt)

![equation](http://latex.codecogs.com/gif.latex?e%5Cpsi%20_%28t&plus;1%29%20%3D%20%5Cpsi%20_t%20-%20%5Cpsi%20dest%20&plus;%20%5Cfrac%7Bv_f%7D%7BL_f%7D%20*%20%5Cdelta_t%20*%20dt)

### Model's Update equations

Before fitting the path returned from the simulator, we have to preprocess in order to move the points to the origin (x=0, y=0) and also rotate the path to follow the car orientation.

```c
//Adjust plain path to car coordinates
//Set x, y and psi to zero
for(unsigned int i=0; i < ptsx.size(); i++){      
    //shift car reference angle to 90 degrees
    double shift_x = ptsx[i] -px;
    double shift_y = ptsy[i] -py;
    ptsx[i] = (shift_x * cos(0-psi) - shift_y*sin(0-psi));
    ptsy[i] = (shift_x * sin(0-psi) + shift_y*cos(0-psi));
```
After preprocessing, using ``polyfit`` to fit polynomial (file main.cpp at line 121).

### Constraints(Cost　Function)

The actuators constraints limits the upper and lower bounds of the steering angle and throttle acceleration/brake.

![equation](http://latex.codecogs.com/gif.latex?%5Cdelta%20%5Cepsilon%20%5B-25%5E%7B%5Ccirc%7D%2C%2025%5E%7B%5Ccirc%7D%5D)

![equation](http://latex.codecogs.com/gif.latex?a%20%5Cepsilon%20%5B-1%2C%201%5D)

The MPC cost's error should be minimized. The const function requires the model to predict where the vehicle will go in the future to compute the difference of ideal position and predicted one.

![equation](http://latex.codecogs.com/gif.latex?J%20%3D%20%5Csum%5E%7BN%7D_%7Bt%3D1%7D%5B%28cte_t%20-%20cte_%7Bref%7D%29%5E2%20&plus;%20%28e%5Cpsi_t%20-%20e%5Cpsi_%7Bref%7D%29%5E2%20&plus;%20...%5D)

For this project, we used the following cost functions to tune the controller(file mpc.cpp at line 53):

```c

	//Cost related to the reference state.
	for (unsigned int t = 0; t < N; t++) {
		fg[0] += 10000*CppAD::pow(vars[cte_start + t], 2); 
		fg[0] += 10000*CppAD::pow(vars[epsi_start + t], 2); 
		fg[0] += 5*CppAD::pow(vars[v_start + t] - ref_v, 2); 
	}
	//Minimize the use of actuators.
	for (unsigned int t = 0; t < N - 1; t++) {
		//Increase the cost depending on the steering angle
	 	fg[0] += 250*CppAD::pow((vars[delta_start + t]/(0.436332*Lf))*vars[a_start + t], 2);
		fg[0] += 50*CppAD::pow(vars[delta_start + t], 2);
	}
	//Minimize the value gap between sequential actuations.
	for (unsigned int t = 0; t < N - 2; t++) {
		fg[0] += 5*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2); 
		fg[0] += 5*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2); 
	}
``` 
Moreover, the following figure shows the result of using the cost function above.The iterations is 300.
![](https://raw.githubusercontent.com/doublepoints/CarND-MPC-Project/master/fig/figure_1.png) 

### Latency

In order to deal with the latency, the MPC solver should be called before predicting the next state. The implement is shown below.

```c
dt = 0.1;
x1    = v * cos(0) * dt;
y1    = v * sin(0) * dt;
psi1  = - v/Lf * steer_value * dt;
v1    = throttle_value * dt;
cte1  =   v * sin(epsi1) * dt;
epsi1 = - v * steer_value / Lf * dt;	
```
The following figure is the screenshot of the inplemention.
![](https://raw.githubusercontent.com/doublepoints/CarND-MPC-Project/master/fig/Untitled.png) 
