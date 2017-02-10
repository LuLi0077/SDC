# Advanced Lane Detection

Write a software pipeline to identify the lane boundaries in a video from a front-facing camera on a car. The notebook contain all the details is here: `Advanced_Lane_Detection.ipynb`.

The major steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit polynomial to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.


### Camera Calibration

#### 1. Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.

I'm using a set of chessboard images to calibrate the camera. I start by preparing "object points", which will be the (x, y, z) coordinates of the chessboard corners in the world. Here I am assuming the chessboard is fixed on the (x, y) plane at z=0, such that the object points are the same for each calibration image.  Thus, `objp` is just a replicated array of coordinates, and `objpoints` will be appended with a copy of it every time I successfully detect all chessboard corners in a test image. "imgpoint" will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard detection. This is in **Step - 1.b**. 

I then used the output `objpoints` and `imgpoints` to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function.  I applied this distortion correction to the test image using the `cv2.undistort()` function and obtained this result: (in **Step - 1.c**)

![CameraCalibration](https://github.com/LuLi0077/SDC/blob/master/Advanced_Lane_Detection/output_images/CameraCalibration.png)


### Pipeline (test images)

The main pipeline for test images is in **Step - 2.g**.

The process of getting there is as below:


#### 1. Provide an example of a distortion-corrected image.

**Step - 2.a**:

![DistortionCorrected](https://github.com/LuLi0077/SDC/blob/master/Advanced_Lane_Detection/output_images/DistortionCorrected.png)


#### 2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image. Provide an example of a binary image result.

Sobel operator, magnitude and direction of the gradient are tested separately in **Step - 2.b.1-3** and in combination in **Step - 2.b.4**. The combination function selecting pixels where both the x and y gradients meet the threshold criteria, or the gradient magnitude and direction are both within their threshold values.

Various color spaces, channels and different thresholds were tested in **Step - 2.b.5**. S-channel in HLS color space seems to work the best.

Combination of color and gradient thresholding has been used to create a binary image containing likely lane pixels, see **Step - 2.b.6**. 

![ThreshBinary](https://github.com/LuLi0077/SDC/blob/master/Advanced_Lane_Detection/output_images/ThreshBinary.png)

(Left) Original image. (Middle) Stacked image; the green is the gradient threshold component and the blue is the color channel threshold component. (Right) black and white combined thresholded image - this one has combined thresholds into one image.


#### 3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

I first investigated an image where the lane lines are straight, and find four points lying along the lines that after perspective transform using the cv2.getPerspectiveTransform() function, make the lines look straight and vertical from a bird's eye view perspective. This is in **Step - 2.c.1**.

The source and destinaiton points are - 

```
src = np.float32(
    [[(img_size[0] / 2) - 72, img_size[1] / 2 + 110],
    [((img_size[0] / 6) - 10), img_size[1]],
    [(img_size[0] * 5 / 6) + 55, img_size[1]],
    [(img_size[0] / 2 + 75), img_size[1] / 2 + 110]])

dst = np.float32(
    [[(img_size[0] / 4), 0],
    [(img_size[0] / 4), img_size[1]],
    [(img_size[0] * 3 / 4), img_size[1]],
    [(img_size[0] * 3 / 4), 0]])

```

I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image.

![StraightPerspect](https://github.com/LuLi0077/SDC/blob/master/Advanced_Lane_Detection/output_images/StraightPerspect.png)

Those same four source points will now work to transform any image, under the assumption that the road is flat and the camera perspective hasn't changed. Here's when it applies to an image with curved lines: (in **Step - 2.c.2**)

![CurvePerspect](https://github.com/LuLi0077/SDC/blob/master/Advanced_Lane_Detection/output_images/CurvePerspect.png)


#### 4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

I first try to find lines with peaks in a histogram (in **Step - 2.d.1**):

![histogram](https://github.com/LuLi0077/SDC/blob/master/Advanced_Lane_Detection/output_images/histogram.png)

With this histogram I am adding up the pixel values along each column in the image. In my thresholded binary image, pixels are either 0 or 1, so the two most prominent peaks in this histogram will be good indicators of the x-position of the base of the lane lines. I can use that as a starting point for where to search for the lines. From that point, I can use a sliding window, placed around the line centers, to find and follow the lines up to the top of the frame and fit a polynomial function in the y direction. (in **Step - 2.d.2**)

![PolynomialFit](https://github.com/LuLi0077/SDC/blob/master/Advanced_Lane_Detection/output_images/PolynomialFit.png)

Later on, a more robust approach is used to process video since I can skip the sliding windows step once know where the lines are.


#### 5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

Here the idea is to take the measurements of where the lane lines are and estimate how much the road is curving and where the vehicle is located with respect to the center of the lane. The radius of curvature may be given in meters assuming the curve of the road follows a circle. For the position of the vehicle, I assume the camera is mounted at the center of the car and the deviation of the midpoint of the lane from the center of the image is the offset. This is in **Step - 2.e**.


#### 6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

The fit from the rectified image has been warped back onto the original image and plotted to identify the lane boundaries. I implemented this in **Step - 2.f**.  Here is my result on test images:

![Output](https://github.com/LuLi0077/SDC/blob/master/Advanced_Lane_Detection/output_images/Output.png)


### Pipeline (video)

#### 1. Provide a link to your final video output. Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!)

I modified the image processing pipeline that was established to find the lane lines in images as following for processes the video:
* If lanes were found previously, skip the sliding window. 
* If the new image, with previous fit do not have enough pixels to be credible, resume sliding window.
* If one lane line is more clear than the other, use its fit for the other lane line assuming they are parallel
* Compare new and previous fit, if error is above threshold, use the previous fit. Otherwise, use .05/.95 weight for new and previous fit. 

Here's a [link to my video result](https://youtu.be/IzsE6VoQ_bw)


### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project. Where will your pipeline likely fail? What could you do to make it more robust?

The pipeline fails when picking out lane lines using histogram fails, as some roads have old (but visible) lane marks and/or medium dividers that shadows on the road, as in the challenge video. It also doesn't work well when the turns are sharp and views are limited, like in the harder challenge video. I tried to check and control for the distance between two lanes, assuming one is better detected than the other, and leverage the fact the two lanes should be paralell in bird-eye view. I fellowed the tips and tricks in the lecture, but I suspect more robust color masks may help, as well as better region detection. Please share any other ideas you have. Thanks!
