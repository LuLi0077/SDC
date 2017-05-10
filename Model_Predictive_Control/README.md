# Model Predictive Control 

Implement Model Predictive Control to drive the car around the track

Model Predictive Control (MPC) involves simulating actuator inputs. This process is done several times resulting in several trajectories. The trajectory associated with the lowest cost is chosen. The simulated trajectories are based off the initial state and the model's dynamics, constraints and, of course, the cost function.

Ultimately only the initial actuations resulting the lowest cost trajectory are executed This brings the vehicle to a new state (now the initial state) and the process is repeated.

Here is the MPC algorithm:

Setup:

1. Define the length of the trajectory, N, and duration of each timestep, dt.
2. Define vehicle dynamics and actuator limitations along with other constraints.
3. Define the cost function.

Loop:

1. At any given time the current state is the input to MPC.
2. The initial state of the trajectory should be constrained to the state passed into the MPC procedure.
3. Call the optimization solver. Given the passed state MPC will simulate taking several trajectories and return the one with the lowest cost. The solver we'll use is called [Ipopt](https://projects.coin-or.org/Ipopt).
4. Back to 1.

### PID Controller

PID controllers are great (given proper tuning) at evaluating an error at a given time and returning a good actuation value. The issue arises in a realtime setting with latency, that is, it's likely the state from which the error was calculated has changed before the actuation is actually performed. This typically ends up with the controller overshooting, the effect gradually worsening with time.

It's possible for a PID controller to compute an actuation value based on a future error, but, without knowledge of the actual model it's unlikely this will be accurate.

### Model Predictive Control

A Model Predictive Controller can plan well into the future, on the order of seconds. It's because this small amount of latency naturally doesn't affect MPC like it does PID control. Additionally due to knowledge of the model, we can pretend the initial state isn't the current state but rather a future state t time from now. Another option would be to select a future actuation rather than the first, which is what's done by default.

Thus, MPC can deal with latency much more effectively than a PID controller, which becomes essential in realtime scenarios.