# Kalman Filters for Sensor Fusion

Here's an overview of the [fusion flow](https://www.youtube.com/watch?v=_u8Vk58VqxY):

![FusionFlow](https://github.com/LuLi0077/SDC/blob/master/Kalman_Filters/images/FusionFlow.png)


### [Extended Kalman Filter (EKF)](https://en.wikipedia.org/wiki/Extended_Kalman_filter)

Original                   |  EKF uses a linear appx of h(x)                     
:-------------------------:|:-------------------------:
<img src="https://github.com/LuLi0077/SDC/blob/master/Kalman_Filters/images/EKF1.png" width="375" height="250">  |  <img src="https://github.com/LuLi0077/SDC/blob/master/Kalman_Filters/images/EKF2.png" width="375" height="250">  

In the original graph, follow the arrows from top left to bottom to top right: (1) A Gaussian from 10,000 random values in a normal distribution with a mean of 0. (2) Using a nonlinear function, arctan, to transform each value. (3) The resulting distribution. The second graph shows the output remains a Gaussian after applying a first order Taylor expansion. [Source](https://www.youtube.com/watch?v=nMUd_esBMM8)


The comparison between KF and EKF: [Source](https://www.youtube.com/watch?v=co0ZczAuwdM)

![KF_EKF](https://github.com/LuLi0077/SDC/blob/master/Kalman_Filters/images/KF_EKF.png)


### [Unscented Kalman Filter](https://en.wikipedia.org/wiki/Kalman_filter#Unscented_Kalman_filter)
(WIP)


### Resources

* Udacity self-driving car: Sensor Fusion, Localization, and Control
* [Lectures in the Kalman Filter](http://www.ilectureonline.com/lectures/subject/SPECIAL%20TOPICS/26/190)
* [Sensor Fusion of Camera, GPS and IMU using Fuzzy Adaptive Multiple Motion Models](https://arxiv.org/pdf/1512.02766v1.pdf)