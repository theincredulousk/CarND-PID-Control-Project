# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

PID Controller for vehicle steering on a racetrack. Simulated vehicle is provided with a stream of cross-track error data and no additional spatial / localization data.

## Reflection

PID Control parameter behavior when applied to vehicle steering:
P: 
* Controls the magnitude of steering correction based on deviation from desired trajectory.
* Setting this parameter too high will cause over-correction and overshoot of desired trajectory
* Setting this parameter too low will result in sluggish reaction during rapid trajectory changes (e.g. a turn)

I:
* Acts to dampen over correction and converge output towards a zero-error offset
* Setting this parameter too high will result in over-correction during small periods of sustained error.
* Setting this parameter too low will result in the vehicle tolerating sustained trajectory error for unacceptably long periods.  Can result in following an overly approximated trajectory.

D:
* Acts to control the rate of change of the steering correction over time, and reduce overshoot.  Rapid rates of change in the input (high frequency components) can cause instability.
* Setting this parameter too high will result in over-sensitivity to rapid (or "noisy") swings in input error and correctional output
* Setting this parameter too low will result in overshoot and oscillating output corrections, which is especially undesireable for steering a vehcile


Chosen Parameters:
(P=0.45, I=0.0036, D=11.25)
* Derived from Ziegler–Nichols method to set parameters
* Achieves impressively low average CTE
* "Ride quality" is unrealistic - movement is jittery and aggressive
* May have worked with a low-pass filter applied to input signal for derivate gain
* Instability means laps can only be completed at 50% throttle to maintain traction

(P=0.075, I=0.00005, D=4)
* Derived from experimentation after initial parameters of (0.01, 0.0001, 10)
* Achieves relatively poor average CTE (0.4) but stable/smoother trajectory
* Compromise made against "smoothness" with P gain in order to manuver sharp corners at speed
* Able to complete laps at constant 75% throttle (about 73-75mph)
* With some tweaking the car can make most of the track at 100% throttle, but crashes on the last turn

(P=0.0655, I=0.0004, D=3)
* Derived from heuristic setting Kd and Ki to 0 initially, and then increasing until instability one at a time.  Kd increased until response is not too sluggish while maintaining stability
* Best average CTE achieved over one lap - .038
* Uses basic throttle control. CTE < .5 ? throttle=75% : throttle=50%
* Final parameters chosen as best overall performance


Other Conclusions:

PID control is an ultimately poor choise for master steering control in part due to the inevitable tradeoff between responsiveness and stability.  The nature of the available data (only current error vs. desired trajectory) means that the control algorithm cannot anticipate, e.g. the sharpness or duration of curves, and further is attempting to converge on a "moving target".  The control algorithm can only react, and as such will always be trailing or "chasing" what would have been the optimal output at any given time step.


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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 
5. Run the simulator and select the PID Controller Environment


