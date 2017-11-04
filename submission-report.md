# MPC Control Project
## Content of the Submission and Usage
This submission includes the following c++ files:
* mpc_main.cpp: the main function that communicates with the simulator and drive the MPC process. It was modified from the original [CarND-MPC-Propject](https://github.com/udacity/CarND-MPC-Project)
* test.cpp: a test program for testing MPC, it was used to generate the graphs
* control/MPC.[h, cpp]: the MPC controller
* model/Vehicle.[h, cpp]: a vehicle model class that implements a simple vehicle motion model
* model/RoadGeometry.[h, cpp]: geometry class that models a road segment
* utils/Config.[h, cpp]: the Config class for providing hyper parameters from config.json file
* utils/utils.[h, cpp]: utility functions
* utils/Reducer.h: the Reducer class for sum, mean, min, max on a collection of samples.

### Usage
**The MPC Controller**
The PID controller can be launched with the following command:

    ./mpc [-fast|-stable] [-speed n] [-latency n]

Where:
* -fast: run as fast as possible using config-fast.json
* -stable: run stably at maximal 100 Mph using config-stable.json
* -speed *n*: specify the maximal speed to be *n* Mph
* -latency *n*: specify the latency to be *n* milliseconds instead of 100 milliseconds

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

* PLOT_TRAJECTORY: when defined, trajectory will be shown on the simulator
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

## Vehicle class
The Vehicle class implements a simple vehicle motion model:

**distance** = *velocity * dt*

**delta_psi** = *steering * dist / Config::Lf*

**psi** = *psi + delta_psi*

**x** = *x + dist * cos(psi)*

**y** = *y + dist * sin(psi)*

**v** = *v + acceleration * dt*

In addition, it also provide methods for converting vehicle coordinate system and the global coordinate system, determining the maximal vehicle speed, and computing the throttle value for a given acceleration.

## RoadGemoetry class
This class models the geometry of a road segment, and provide methods for computing the cross track error given a location, orientation given x, maximal turn (orientation change) between two locations, and y coordinate fiven x.

## Config class
The Config class reads the hyper parameters from the config.json file:
* N: the number of steps, json field: "N"
* dt: the duration of each step, json field: "dt"
* ipoptTimeout: the IPOPT timeout (in second), json field: "ipopt timeout"
* latency: control latency in millie seconds, json field: "latency"
* maxFitOrder: the maximal polynomial fitting order for road, json field: "max polynomial fitting order"
* maxFitError: the maximal polynomial fitting error for road, json field: "max polynomial fitting error"
* maxSteering: Maximal steering angle, json field: "max steering"
* maxAcceleration: the maximal acceleration of vehicle, json field: "max acceleration"
* maxDeceleration: the maximal deceleration of vehicle, json field: "max deceleration"
* maxSpeed: the xaximal speed of vehicle, json field: "max speed"
* Lf: Length from the front wheels to the center of the back wheels, json field: "Lf"
* epsiPanic: the epsi's panic level. Above this, a bigger penality will be applied, json field: "epsi panic"
* ctePanic: the cte's panic level. Above this, a bigger penality will be applied, json field: "cte panic"
* steerAdjustmentThresh: simulate human driver to steer more on sharp turns, json field: "steer adjustment threshold",
* steerAdjustmentRatio: the ratio of the road's orientation change to apply to steering angle, json field:  "steer adjustment ratio": 0.022,
* weights: cost weights: 0: cte, 1: epsi, 2: v, 3: delta, 4: delta delta, 5: not used, a, 7: delta a, 8: large deceleration low velocity, 9: negative speed, 10: out of range epsi, json field: "weights"
* steers: Steering angles to map to speed target, json field: "steers"
* steerSpeeds: speeds for mapping steering angles to target speed, the last one is for the rest of steering angles, json field: "steer speeds"
* yawChanges: road orientation changes to map to speed limit, json field: "yaw change"
* yawChangeSpeeds: speed for mapping road orientation changes to speed limit, the last item is for the rest of orientation change, json field: "yaw change speed"

## Utility functions
Some utility functions are defined in utils.[h, cpp] for:
* Transformation between map coordinate and vehicle coordinate, **globalToVehicle()**, **vehicleToGlobal()**
* Polynomial fitting, **polyfit()** 
* Evaluating polynomial, **polyeval()**
* Evaluating first order derivatives of a polynomial, **polyder()**
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
In order to simulate the vehicle movement during the latency, we need to know the current vehicle acceleration in addition to vehicle location, orientation, and steering angle provided by the simulator. Unfortunately the vehicle's acceleration cannot be obtained from throttle easily as it depends on not only the vehicle but the road's slope and condition also.

The implementation uses a very simple logic - by multiplying the current throttle with a constant, which is 6 at present. Experiments have shown that chosing any value between 1 and 10 yielded no significant differences. 6 was picked as an average.

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
The total MPC cost is summed from **weighted squares** of:
* CTE
* epsi
* velocity
* steering angle
* change in steering angle
* acceleration, change in acceleration
* out of range things like large deceleration at low velocity, negative speed, and out of range epsi

The experiments showed that tuning weights for CTE, epsi, steering angle, and change in steering angle is important for the vehicle's overall stability.

## Hyper parameter tuning

### N and dt
The N, the number of times steps, and dt, the duration of each time step, parameters determine the event horizon of the MPC in that:

    event horizon = N * dt

N affects the event horizon and the IPOPT time, both are proporotional to N. Chosing a large N will require more processing time, buit the result may not improve as much. Experiements have shown that a large N will cause IPOPT to timeout.

The values of dt affect the MPC accuracy, when dt is too large, a large MPC error may result.

## Results

The best MPC results were obtained with an event horizon of 1 second, and a dt close to the total latency which is the sum of the MPC computation time and the vehicle control latency. An example video for N = 10 and dt = 0.1 is shown [here](./MPC-trajectory.mp4), and the corresponding video without trajectory is shown [here](./MPC.mp4)

### The effects of N and DT on CTE and ePsi

The following charts show MPC iterations for different N and DT:

|   dt in seconds |       Graph                       |
|:----------------|:---------------------------------:|
| N: 10, dt: 0.1  |[Graph 1](examples/10-01-2.png)    |
| N: 20, dt: 0.1  |[Graph 2](examples/20-01-2.png)    |
| N: 30, dt: 0.1  |[Graph 3](examples/30-01-2.png)    |
| N: 40, dt: 0.1  |[Graph 4](examples/40-01-2.png)    |
| N: 10, dt: 0.05 |[Graph 5](examples/10-005-2.png)   |
| N: 20, dt: 0.05 |[Graph 6](examples/20-005-2.png)   |
| N: 30, dt: 0.05 |[Graph 7](examples/30-005-2.png)   |
| N: 40, dt: 0.05 |[Graph 8](examples/40-005-2.png)   |
| N: 50, dt: 0.05 |[Graph 9](examples/50-005.png)     |
| N: 10, dt: 0.02 |[Graph 10](examples/10-002.png)    |
| N: 20, dt: 0.02 |[Graph 11](examples/20-002.png)    |
| N: 30, dt: 0.02 |[Graph 12](examples/30-002.png)    |
| N: 40, dt: 0.02 |[Graph 13](examples/40-002.png)    |
| N: 50, dt: 0.02 |[Graph 14](examples/50-002.png)    |

The graphs show:

1. The initial CTE and/or ePsi decrease as N increase first, then stop increasing after certain value of N.
2. A smaller event horizon results in a large CTE.
3. Increasing N may improves CTE and ePsi, but not for a small dt

#### Small dt

Also comparing dt = 0.1 seconds with dt=0.05 seconds, the former has smaller CTE and ePsi at the begining. This may suggest that smaller dt is not necessary preferable for a given event horizon.

This is confirmed in my experiments that the vehicle appeared to be increasingly unstable as dt decreases. With a small dt, the vehicle will oscillate a lot. [This video](examples/MPC-50-02-80.mp4) show an example with N = 50 and dt = 0.02 seconds. The instability due to small dt may also be contributed by the fact that the actuators obtained from the first dt are used, and when the value of dt is significantly smaller than the control latency, the actuators will not be able to cover what are required for the entire latency period.

#### Large dt

### Small N

A small N will have a small event horizon and results in larger CTE, this can be seen by comparing videos [N = 10, dt = 0.05 seconds](examples/MPC-10-05-80.mp4]) with [N = 20, dt = 0.05 seconds](examples/MPC-20-05-80.mp4]) where the vehicle in the first video moves very closed to the edge of the road at sharp turns.

Using a dt that is significantly large than the latency may reduce accuracy significantly, and degrade the vehicle's ability to handle sharp turns. This can be shown in [this video](examples/MPC-10-2-80.mp4) whose N = 10, and dt = 0.2 seconds.

#### Large N

Experiments also show that large event horizon due to large N does not improve MPC performance. This can be show in the following videos [N = 20, dt = 0.05 seconds](examples/MPC-20-05-80.mp4]) vs [N = 40, dt = 0.05 seconds](examples/MPC-40-05-80.mp4])

### Cost weights

Weights are relative, increasing one will reduce the importance others. For vehicle control, minimizing CTE, ePsi, and the actuators is the goal of the MPC control. To reduce weight tuning efforts, it is important to tune weights around a reference.

#### Reference weight

CTE's weight has been chosen as the reference, and it has been set to 1.0 in the configuration files. Changing this value has greater impact on the vehicle's overall stability, and is not recommended.

On the other hand, we still want to avoid high CTE to prevent the vehicle to run off the road. This is done by using multiple weights for different CTE ranges. In the MPC implementation, a CTE panic threashold divide the CTE into two range, the normal weight is applied when CTE is below the panic level, and a much high weight is applied when CTE is above the panic level.

#### ePsi's weight

A high ePsi weight reduces the importantance of CTE, and will result in large CTE and poor vehicle stability. Conversely, a low ePsi weight will result in high ePsi errors and poor stability. Experiments have shown that ePsi weights between 80 and 120 yield better result at high speed.

In addition, we still want to avoid high ePsi to prevent the vehicle to run off the road. This is done in the same way as the CTE. In the MPC implementation, a ePsi panic threashold divide the ePsi into two range, the normal weight is applied when ePsi is below the panic level, and a much high weight is applied when ePsi is above the panic level.

[This graph](examples/ePsi-weights.png) shows the effects of ePsi's weight. Weight 1000 has larger CTE errors but smaller ePsi errors comparing to weight 100.

#### Delta acutator's weight

Delta actuator's weight has no significant impact on the vehicle's stability unless it is exceedingly large. This is shown in [this graph](examples/delta-weights.png) where larger CTE errors can be seen with weight 5000.

#### Velocity's weights

The velocity's weight has no significant impact the vehicle's stability. However, setting the weight to 0 will cause the vehicle to stale or reverse. This can be seen in [this graph](examples/velocity-weights.png) where weight 1 and 100 produce the same result, but the vehicle decelerates when weight is 0.

#### Acceleration acuator's weight

The acceleration actuator's weight has no visuable impact on the MPC results, this can be seen from [this graph](examples/accel-weights.png) for weight 0, 100, and 10000.

#### Delta actuator's change weight

The delta actuator's change weight has no significant impact on the MPC results unless it is exceedingly large. This can be seen from [this graph](examples/delta-delta-weights.png) where weight 5000 has slightly larger CTE errors.

#### Acceleration actuator's change weight

The acceleration actuator's change weight has no visuable impact on the MPC results. This can be seen from [this graph](examples/delta-accel-weights.png).