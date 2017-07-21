# Unscented Kalman Filter in C++

### Visualizations

Sample Data 1              |  Sample Data 2                     
:-------------------------:|:-------------------------:
<img src="https://github.com/LuLi0077/SDC/blob/master/Kalman_Filters/images/ukf1.png" width="425" height="300">  |  <img src="https://github.com/LuLi0077/SDC/blob/master/Kalman_Filters/images/ukf2.png" width="425" height="300">  


### Dependencies

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


### Catch the Run Away Car

In [Artificial Intelligence for Robotics](https://classroom.udacity.com/courses/cs373/lessons/672478550/concepts/7831886840923), Sebastian's final project was to catch a run away robot moving in a continuous circle with constant velocity. This extra challenge revisits that same problem but with the implementation of an Unscented Kalman Filter.

The run away car in this case will be being sensed by a stationary sensor, that is able to measure both noisy lidar and radar data. The capture vehicle will need to use these measurements to close in on the run away car. To capture the the run away car the capture vehicle needs to come within .1 unit distance of its position. However the capture car and the run away car have the same max velocity, so if the capture vehicle wants to catch the car, it will need to predict where the car will be ahead of time.


### Resources

* [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html)
* Generate your own radar and lidar data, see the [utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities)

