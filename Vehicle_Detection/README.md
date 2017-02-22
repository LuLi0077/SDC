# Vehicle Detection

Write a software pipeline to identify vehicles in a video from a front-facing camera on a car. The notebook contain all the details is here: `Vehicle_Detection.ipynb`.

The major steps of this project are the following:

* Perform a Histogram of Oriented Gradients (HOG) feature extraction on a labeled training set of images and train a classifier Linear SVM classifier.
* Apply a color transform and append binned color features, as well as histograms of color, to HOG feature vector. 
* Implement a sliding-window technique and use trained classifier to search for vehicles in images.
* Run the pipeline on a video stream and create a heat map of recurring detections frame by frame to reject outliers and follow detected vehicles.
* Estimate a bounding box for vehicles detected.


### Feature Extraction

#### a. Read in vehicle and non-vehicle images (`Step - 1.a`)

Sampled images from the following sources: 
* [Annotated Driving Dataset](https://github.com/udacity/self-driving-car/tree/master/annotations): Dataset 1 and 2 (`Step - 1.a.1` - `Step - 1.a.4`)
* [KITTI vision benchmark suite](http://www.cvlibs.net/datasets/kitti/): (`Step - 1.a.5`)
* [GTI vehicle image database](http://www.gti.ssr.upm.es/data/Vehicle_database.html): (`Step - 1.a.5`)

Here is an example of one of each of the `vehicle` and `non-vehicle` classes, after resized to (64, 64):

![SampleImage](https://github.com/LuLi0077/SDC/blob/master/Vehicle_Detection/output_images/sampleimage.png)


#### b. Extract color features (`Step - 1.b`)

* Color histogram: use histograms of pixel intensity as features (`Step - 1.b.1`)

Use the sample vehicle image above as an example and assume RGB color space - 

![histogramfeature](https://github.com/LuLi0077/SDC/blob/master/Vehicle_Detection/output_images/histogramfeature.png)

These, collectively, will be our feature vector for this image.

* Explore color spaces: study the distribution of color values in an image by plotting each pixel in some color space (`Step - 1.b.2`)

Use the sample vehicle image above as an example, take a look at RGB, HSV and YUV color spaces - 

RGB                        |  HSV                      |  YUV
:-------------------------:|:-------------------------:|:-------------------------:
<img src="https://github.com/LuLi0077/SDC/blob/master/Vehicle_Detection/output_images/RGBcolorspace.png" width="250" height="250">  |  <img src="https://github.com/LuLi0077/SDC/blob/master/Vehicle_Detection/output_images/HSVcolorspace.png" width="250" height="250">  |  <img src="https://github.com/LuLi0077/SDC/blob/master/Vehicle_Detection/output_images/YUVcolorspace.png" width="250" height="250"> 

* Spatial binning of color: create the feature vector using cv2.resize().ravel() (`Step - 1.b.3`)


#### c. Extracted HOG features (`Step - 1.c`)

* scikit-image HOG: below is the sample vehicle using HOG parameters of `orientations=8`, `pixels_per_cell=(8, 8)` and `cells_per_block=(2, 2)` (`Step - 1.c.1`)

![hogsample](https://github.com/LuLi0077/SDC/blob/master/Vehicle_Detection/output_images/hogsample.png)

The HOG visualization is not actually the feature vector, but rather, a representation that shows the dominant gradient direction within each cell with brightness corresponding to the strength of gradients in that cell.

* Combine and normalize features: combine color histograms, spatial binning of color and HOG feature then normalize them before training a classifier (`Step - 1.c.2`)

Below shows sample vehicle and non-vehicle images with its raw and normalized features:

![samplefeatures](https://github.com/LuLi0077/SDC/blob/master/Vehicle_Detection/output_images/samplefeatures.png)


### Build a Classifier

Multiple classifiers from sklearn are tested (`Step - 2.b`), as well as LeNet-5 from project [Traffic_Sign_Classification](https://github.com/LuLi0077/SDC/tree/master/Traffic_Sign_Classification) (`Step - 2.c`) using only 8000 images (4000/class). Various color space and HOG parameter combinations are also tested here (`Step - 2.a`), for example: 

For `colorspace = 'RGB'` and HOG paramters (orientations = 9, pixels_per_cell = 8, cells_per_block = 2, hog_channel = "ALL") - 
* Test Accuracy of Nearest Neighbors = 0.83625 ; cost = 2.53
* Test Accuracy of Linear SVM = 0.89625 ; cost = 29.35
* Test Accuracy of RBF SVM = 0.9425 ; cost = 186.4
* Test Accuracy of Gaussian Process = 0.535 ; cost = 1649.1
* Test Accuracy of Decision Tree = 0.766875 ; cost = 36.73
* Test Accuracy of Random Forest = 0.859375 ; cost = 3.51
* Test Accuracy of Neural Net = 0.9375 ; cost = 20.56
* Test Accuracy of AdaBoost = 0.749375 ; cost = 227.97
* Test Accuracy of Naive Bayes = 0.80875 ; cost = 1.13
* Test Accuracy of QDA = 0.555625 ; cost = 81.65

For `colorspace = 'YCrCb'` with the same HOG parameters -
* Test Accuracy of Nearest Neighbors = 0.8275 ; cost = 2.12
* Test Accuracy of Linear SVM = 0.914375 ; cost = 29.25
* Test Accuracy of RBF SVM = 0.950625 ; cost = 183.56
* Test Accuracy of Gaussian Process = 0.4975 ; cost = 1650.2
* Test Accuracy of Decision Tree = 0.800625 ; cost = 32.38
* Test Accuracy of Random Forest = 0.8725 ; cost = 3.33
* Test Accuracy of Neural Net = 0.94875 ; cost = 19.66
* Test Accuracy of AdaBoost = 0.795 ; cost = 197.83
* Test Accuracy of Naive Bayes = 0.82 ; cost = 1.13
* Test Accuracy of QDA = 0.5 ; cost = 80.67


### Sliding Window Search

#### 1. Describe how and identify where in the code you implemented a sliding window search.  How did you decide what scales to search and how much to overlap windows?

I decided to search random window positions at random scales all over the image and came up with this (ok just kidding I didn't actually ;):

A sliding window approach has been implemented, where overlapping tiles in each test image are classified as vehicle or non-vehicle. Some justification has been given for the particular implementation chosen.

![alt text][image3]

#### 2. Show some examples of test images to demonstrate how your pipeline is working.  What did you do to optimize the performance of your classifier?

Ultimately I searched on two scales using YCrCb 3-channel HOG features plus spatially binned color and histograms of color in the feature vector, which provided a nice result.  Here are some example images:

Some discussion is given around how you improved the reliability of the classifier i.e., fewer false positives and more reliable car detections (this could be things like choice of feature vector, thresholding the decision function, hard negative mining etc.)

![alt text][image4]


### Video Implementation

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (somewhat wobbly or unstable bounding boxes are ok as long as you are identifying the vehicles most of the time with minimal false positives.)
Here's a [link to my video result](./project_video.mp4)


#### 2. Describe how (and identify where in your code) you implemented some kind of filter for false positives and some method for combining overlapping bounding boxes.

A method, such as requiring that a detection be found at or near the same position in several subsequent frames, (could be a heat map showing the location of repeat detections) is implemented as a means of rejecting false positives, and this demonstrably reduces the number of false positives. Same or similar method used to draw bounding boxes (or circles, cubes, etc.) around high-confidence detections where multiple overlapping detections occur.

I recorded the positions of positive detections in each frame of the video.  From the positive detections I created a heatmap and then thresholded that map to identify vehicle positions.  I then used `scipy.ndimage.measurements.label()` to identify individual blobs in the heatmap.  I then assumed each blob corresponded to a vehicle.  I constructed bounding boxes to cover the area of each blob detected.  

Here's an example result showing the heatmap from a series of frames of video, the result of `scipy.ndimage.measurements.label()` and the bounding boxes then overlaid on the last frame of video:

##### Here are six frames and their corresponding heatmaps:

![alt text][image5]

##### Here is the output of `scipy.ndimage.measurements.label()` on the integrated heatmap from all six frames:
![alt text][image6]

##### Here the resulting bounding boxes are drawn onto the last frame in the series:
![alt text][image7]


### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.  

As an optional challenge Once you have a working pipeline for vehicle detection, add in your lane-finding algorithm from the last project to do simultaneous lane-finding and vehicle detection!
