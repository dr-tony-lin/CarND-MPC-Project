# MPC Control Project
[MPC1]: ./MPC1.png
[MPC2]: ./MPC3.png
[15-1.5]: ./MPC-15-1_5.mp4
[20-1]: ./MPC-20-1.mp4
[20-1.5]: ./MPC-20-1_5.mp4
[25-1]: ./MPC-25-1.mp4
[MPC]: ./MPC.mp4

## Content of the Submission and Usage
This submission includes the following c++ files:
* mpc_main.cpp: the main function that communicates with the simulator and drive the MPC process. It was modified from the original [CarND-MPC-Propject](https://github.com/udacity/CarND-MPC-Project)
* test.cpp: a test program for testing MPC, it was used to generate the graphs
* control/MPC.[h, cpp]: the MPC controller
* utils/Config.[h, cpp]: the Config class for providing hyper parameters from config.json file
* utils/utils.[h, cpp]: utility functions
* utils/Reducer.h: the Reducer class for sum, mean, min, max on a collection of samples.

### Usage
**The MPC Controller**
The PID controller can be launched with the following command:

    ./mpc

The program will listen on port 4567 for an incoming simulator connection. Only one simulator should be connected at anytime, though the program does not prohibit it. To start a new simulator, terminate the existing one first, then start a new one.

To start the simulator:

#### On Windows

    term2_sim9.exe

#### Other platforms:

    ./term2_sim9

#### Build
For Windows, Bash on Ubuntu on Windows should be used. Both gcc and clang can be used to build the program.

To build the program, invoke the following commands on the bash terminal:
```
mkdir build
cd build
cmake ..
make
```

The program can be built to output more information for disgnosis purposes by defining **VERBOSE_OUT** macro.
In addition, the following macros can be defined:

* SKIP_TRAJECTORY: when defined, trajectory will not be shown on the simulator
* COLLECT_DATA: when defined, control data and simulator inputs will be written to stderr in CSV format.
* TRACE_IPOPT: when defined, IPOPT trace will be printed to the stdout.
* EXIT_ON_IPOPT_FAILURE, when defined, IPOPT failure will cause the program to exit

#### Build API Documentation
The documentation for functions, classes and methods are included in the header files in Doxygen format. To generate Api documentation with the included doxygen.cfg:

1. Make sure doxygen is installed
2. cd to the root folder of this project
3. Issue the following command:

    doxygen doxygen.cfg

## The Implementation

### The source code structure
The src folder contains mpc_main.cpp, test.cpp, and two folders:
. control: contains the MPC class
. utils: contains utility classes and functions

#### File names
In this project, I made a class to have its own .h, and .cpp files. More specifically, a class source file contains exactly one class, and has the same name as the class it contains.

#### The libs folder
The libs folder contains Eigen, json.hpp, and matplotlibcpp.h.

## MPC class
This class implements MPC controller. The **run()** method is used to run the MPC control. It performs the following actions:
1. Transform the center line coordinate to the vehicle's coordinate
2. Compute the fitting polynomial coefficients for the transformed center line. The maximal order of the polynomial is configured in the Config class.
3. Compute the initial CTE and epsi of the vehicle at x = 0, y = 0, and pis = 0.
4. Compute the target velocity, and upper and lower bounds of yaw according to how much the center line's angle changes between the current vehicle location and the last point of the line
4. Invoke the **solve()** method that uses [IPOPT](https://projects.coin-or.org/Ipopt) nonlinear optimization.

### The state
The IPOPT state is composed of x, y, psi, v, CTE, and epsi. The initial state is given by 0, 0, 0, current velocity, initial CTE, and initial epsi.

#### Initial CTE
The initial CTS is the y coordinate of the center line at 0:

    polyeval(poly, 0)

Where poly is the fitted polynomial of the center line

#### Initial epsi
The initial epsi is the negative of the derivative of the fitted polynomial at 0 along the positive direction:
    -polypsi(poly, 0, 1.0)

### The Actuators
Two actuators are of interested:

* delat: the steering angle
* a: the acceleration

### IPOPT optimization
The IPOPT optimization's cost constraints are computed as follows:

**vdt** = *v0 * dt*

**psi** = *psi0 + delta0 * vdt / Config::Lf*

**v** = *v0 + a0 * dt*

**constraint of x** = *x1 - (x0 + cos(psi0) * vdt)*

**constraint of y** = *y1 - (y0 + sin(psi0) * vdt)*

**constraint of psi** = *psi1 - psi*

**constraint of v** = *v1 - v*

**constraint of CTE** = *cte1 - ((polyeval(coeffs, x0) - y0) + sin(epsi0) * vdt)*

**constraint of epsi** = *epsi1 - (psi - polypsi(coeffs, x0, 1.0))*

Where:

* x0, x1, y0, y1, psi0, psi1, v0, v1, cte0, cte1, psi0, and psi1 are the x, y, psi, v, CTE, and epsi at previous and current time respectively.
* delta0, and a0 are the current actuator values
* coeffs is the fitting polynomial coefficients of the center line

## Config class
The Config class reads the hyper parameters from the config.json file:
* N: the number of steps, json field "N"
* dt: the duration of each step, json field "dt"
* ipoptTimeout: the IPOPT timeout (in second), json field "ipopt timeout"
* latency: control latency in millie seconds, json field "latency"
* maxPolyOrder: the maximal fitting polynomial order, json field "max poly order"
* maxSteering: Maximal steering angle, json field "max steering"
* maxAcceleration: the maximal acceleration of vehicle, json field "max acceleration"
* maxDeceleration: the maximal deceleration of vehicle, json field "max deceleration"
* maxSpeed: the xaximal speed of vehicle, json field "max speed"
* Lf: Length from the front wheels to the center of the back wheels, json field "Lf"
* epsiPanic: the epsi's panic level. Above this, a bigger penality will be applied, json field "epsi penalize more"
* weights: cost weights: 0: cte, 1: epsi, 2: v, 3: delta, 4: delta delta, 5: not used, a, 7: delta a, 8: large deceleration low velocity, 9: negative speed, 10: out of range epsi, json field "weights"
* steers: Steering angles to map to speed target, json field "steers"
* steerSpeeds: speeds for mapping steering angles to target speed, the last one is for the rest of steering angles, json field "steer speeds"
* yawChanges: road orientation changes to map to speed limit, json field "yaw change"
* yawChangeSpeeds: speed for mapping road orientation changes to speed limit, the last item is for the rest of orientation change, json field "yaw change speed"

## Utility functions
Some utility functions are defined in utils.[h, cpp] for:
* Transformation between map coordinate and vehicle coordinate, **globalToVehicle()**, **vehicleToGlobal()**
* Polynomial fitting, **polyfit()** 
* Evaluating polynomial, **polyeval()**
* Evaluating first order derivatives of a polynomial, **polypsi()**
* Moving vehicle, **moveVehicle()**
* Computing speed limit and target speed, **computeSpeedTarget()**, **computeYawChangeSpeedLimit()**, **computeYawChange()**
* unit conversion like mph(mile per hour) and meter per second (mps), degree and radian, (**MpH2MpS()**, **MpS2MpH()**, **deg2rad()**, **rad2deg()**)
* Angle normalization to [-PI, PI], **normalizeAngle()**, and value clampping, **clamp()**

## Reducer class
This class provides aggregation for a set of samples. This is used for computing MPC processing time for estimating the total latency to be simulated before running MPC.

## MPC Process
Each MPC iteration includes the following steps:
1. Normalize the vehicle orientation from the simulator to [-PI, PI] range
2. Convert speed meter per second
3. Convert the steering angle to map coordinate by negating it
4. Simulate vehicle movement by the estimated total latency.
5. Used the new position, and run the MPC by invoking MPC's run method
6. Delay for the configured latency
7. Send the acutator results from MPC to the simulator

### Simulate vehicle movement
In order to simulate the vehicle movement during the latency, we need to know the current vehicle acceleration in addition to vehicle location, orientation, and steering angle provided by the simulator. Unfortunately the current vehicle acceleration cannot be obtained from throttle easily as it depends on the vehicle and road.

The implementation uses a very simple logic - by multiplying the current throttle with a constant, which is 4 at present. Emprically, this seems fine. Also, I have tried other values between 1 to 10, and did not observe big differences.

The moving model is:

**distance** = *velocity * dt*

**delta_psi** = *steering * dist / Config::Lf*

**psi** = *psi + delta_psi*

**x** = *x + dist * cos(psi)*

**y** = *y + dist * sin(psi)*

**v** = *v + acceleration * dt*

## Determine vehicle speed
Since the center line curve is available, we could determine the maximal speed like a real driver who will accelerate or decelerate according to much turn that needs to be made. This is implemented as follow:
1. The change in orientations between the vehicle location and the last point of the center line is computed first
2. The change is then used to determing the speed limit using the Config's hyper parameter, **yawChanges** and **yawChangesSpeeds**
3. The speed limit and the steering angle are then used to determing the target speed that will be used as the MPC's reference speed

### Determine Yaw bounds
The change in orientation obtained above is also used to determine the yaw's upper and lower bound in MPC:
1. If the change is negative, the bound is [change, 0.1]
2. Otherwise, it is [-0.1, change]

### Determine the acceleration for throttle
The acceleration result produced by IPOPT cannot be used as is, otherwise, the vehicle might run off the road in large turns.

The acceleration that is used to determine the throttle is taken from the minimal of the IPOPT result and the acceleration/deceleration required to reach the target speed.

### MPC cost
The total MPC cost is summed from weighted squares of:
* CTE
* epsi
* velocity
* steering angle
* change in steering angle
* acceleration, change in acceleration
* out of range things like large deceleration at low velocity, negative speed, and out of range epsi

The experiments showed that tuning weights for CTE, epsi, steering angle, and change in steering angle is important for the vehicle's overall stability.

## Hyper parameter tuning
The results of tunning the hyper parameters show the following:

* N: between 20, to 30
* dt: around 0.05
* cte weight: 1 to 1.5
* epsi weight: around 100 for epsi less than the panic level, otherwise 5000
* delta weight: 400 to 800
* delta delta weight: 1000

### N and dt
The N, the number of times steps, and dt, the duration of each time step, parameters determine the event horizon of the MPC in that:

    event horizon = N * dt

N affects the event horizon and the IPOPT time, both are proporotional to N. Chosing a large N will require more processing time, and the result may not improve. Experiements have shown that when event horizon is above 1.5 seconds, instability starts to increase. In some case, it may take IPOPT a lot longer to converge (can be over few hundred millie seconds), and fails.

The values of dt affect the event horizon, and the precision of MPC, the smaller the value, the more accurate result may be obtained, but at the cost of reduced event horizon.

Experiments have found setting N to be between 10 and 30, and dt to be between 0.1 and 0.05 for 1 second of event horizon seem good choices.

## Results

The following videos shows the simulation results with different steps, and CTE weights:

|                                   |       Video              |
|:----------------------------------|:------------------------:|
| N: 15, dt: 0.05, CTE weight: 1.5  |[Video](MPC-15-1_5.mp4)   |
| N: 20, dt: 0.05, CTE weight: 1    |[Video](MPC-20-1.mp4)     |
| N: 20, dt: 0.05, CTE weight: 1.5  |[Video](MPC-20-1_5.mp4)   |
| N: 25, dt: 0.05, CTE weight: 1    |[Video](MPC-25-1.mp4)     |

A video without trajectory for N = 20, and CTE = 1 is [shown here](./MPC.mp4)

The following charts show MPC iterations for different N and DT:

|                 |       Graph              |
|:----------------|:------------------------:|
| N: 10, dt: 0.1  |[Graph 1](10-01-2.png)      |
| N: 20, dt: 0.1  |[Graph 2](20-01-2.png)      |
| N: 30, dt: 0.1  |[Graph 3](30-01-2.png)      |
| N: 10, dt: 0.05 |[Graph 4](10-005-2.png)     |
| N: 20, dt: 0.05 |[Graph 5](20-005-2.png)     |
| N: 30, dt: 0.05 |[Graph 6](30-005-2.png)     |
| N: 40, dt: 0.05 |[Graph 7](40-005-2.png)     |
| N: 50, dt: 0.02 |[Graph 8](50-002.png)       |

The graphs show that the irregularity increases as N increase roughly after passing 1.5 second event horizon that result in large CTE spike at the end like [Graph 6](30-005-2.png) and [Graph 7](40-005-2.png). When the event horizon is too small, large errors will also occur, like [Graph 4](10-005-2.png). 

Also comparing dt = 0.1 and dt=0.05, the former has large CTE errors at the end e.g. [Graph 1](10-01-2.png), [Graph 2](20-01-2.png). This may suggest that smaller dt is preferable given a event horizon if the computing resource permits, [Graph 8](50-002.png) with dt = 0.02 has a much smoother and smaller CTE at the end may be another prove. 

Unfortunately, setting dt = 0.02 will require N = 50 for 1 second of event horizon, and may not always be feasible.

### Discussion
During the tunning, I have observed the followings:
1. CTE weights outside [1, 2] may reduce the vehicle's stability
2. High CTE weight may cause MPC overshot, and subsequent correction will cause the vehicle to oscillate
3. Low CTE will cause the MPC undershot, and the vehicle may run off the road. Also subsequent correction will cause the vehicle to oscillate

This also suggects that allowing a small CTE, as long as it is not too large to cause the vehicle to run off the road or oscillate, can improve the stability of the vehicle.
