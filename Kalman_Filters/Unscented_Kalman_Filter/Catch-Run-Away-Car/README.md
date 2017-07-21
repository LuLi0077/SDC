# Catch the Run Away Car with Unscented Kalman Filter 

Implement an UKF, and use it to catch an escaped car driving in a circular path.

The run away car will be being sensed by a stationary sensor, that is able to measure both noisy lidar and radar data. The capture vehicle will need to use these measurements to close in on the run away car. To capture the run away car the capture vehicle needs to come within .1 unit distance of its position. However the capture car and the run away car have the same max velocity, so if the capture vehicle wants to catch the car, it will need to predict where the car will be ahead of time.


**INPUT**: values provided by the simulator to the c++ program

* current noiseless position state of the capture vehicle, called hunter

["hunter_x"]

["hunter_y"]

["hunter_heading"]

* get noisy lidar and radar measurments from the run away car.

["lidar_measurement"]

["radar_measurement"]


**OUTPUT**: values provided by the c++ program to the simulator

* best particle values used for calculating the error evaluation

["turn"] <= the desired angle of the capture car "hunter" no limit for the anlge

["dist"] <= the desired distance to move the capture car "hunter" can't move faster than run away car


### Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4
* uWebSocketIO



