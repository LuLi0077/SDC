# Kalman Filters for Sensor Fusion

Here's an overview of the [fusion flow](https://www.youtube.com/watch?v=_u8Vk58VqxY):

![FusionFlow](https://github.com/LuLi0077/SDC/blob/master/Kalman_Filters/images/FusionFlow.png)


### [Extended Kalman Filter (EKF)](https://en.wikipedia.org/wiki/Extended_Kalman_filter)

Original                   |  EKF uses a linear appx of h(x)                     
:-------------------------:|:-------------------------:
<img src="https://github.com/LuLi0077/SDC/blob/master/Kalman_Filters/images/EKF1.png" width="425" height="300">  |  <img src="https://github.com/LuLi0077/SDC/blob/master/Kalman_Filters/images/EKF2.png" width="425" height="300">  

In the original graph, follow the arrows from top left to bottom to top right: (1) A Gaussian from 10,000 random values in a normal distribution with a mean of 0. (2) Using a nonlinear function, arctan, to transform each value. (3) The resulting distribution. The second graph shows the output remains a Gaussian after applying a first order Taylor expansion. ([source](https://www.youtube.com/watch?v=nMUd_esBMM8))


The comparison between KF and EKF: ([source](https://www.youtube.com/watch?v=co0ZczAuwdM))

![KF_EKF](https://github.com/LuLi0077/SDC/blob/master/Kalman_Filters/images/KF_EKF.jpg)


### Unscented Kalman Filter

When the state transition and observation models — that is, the predict and update functions f and h — are highly non-linear, the extended Kalman filter can give particularly poor performance. This is because the covariance is propagated through linearization of the underlying non-linear model. The unscented Kalman filter (UKF) uses a deterministic sampling technique known as the unscented transform to pick a minimal set of sample points (called sigma points) around the mean. These sigma points are then propagated through the non-linear functions, from which a new mean and covariance estimate are then formed. The result is a filter which, for certain systems, more accurately estimates the true mean and covariance. This can be verified with Monte Carlo sampling or Taylor series expansion of the posterior statistics. In addition, this technique removes the requirement to explicitly calculate Jacobians, which for complex functions can be a difficult task in itself (i.e., requiring complicated derivatives if done analytically or being computationally costly if done numerically), if not impossible (if those functions are not differentiable). ([source](https://en.wikipedia.org/wiki/Kalman_filter#Unscented_Kalman_filter))

Constant Turn Rate and Velocity Magnitude Model  |  UKF Roadmap                    
:-----------------------------------------------:|:------------------------------------------------:
<img src="https://github.com/LuLi0077/SDC/blob/master/Kalman_Filters/images/CTRV.png" width="425" height="300">  |  <img src="https://github.com/LuLi0077/SDC/blob/master/Kalman_Filters/images/UKF.png" width="425" height="300">  


### Resources

* Udacity self-driving car: Sensor Fusion, Localization, and Control
* [Lectures in the Kalman Filter](http://www.ilectureonline.com/lectures/subject/SPECIAL%20TOPICS/26/190)
* [Sensor Fusion of Camera, GPS and IMU using Fuzzy Adaptive Multiple Motion Models](https://arxiv.org/pdf/1512.02766v1.pdf)