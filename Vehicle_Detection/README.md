# Vehicle Detection

Write a software pipeline to identify vehicles in a video from a front-facing camera on a car. The notebook contain all the details is here: `Vehicle_Detection.ipynb`.

The major steps of this project are the following:

* Perform a Histogram of Oriented Gradients (HOG) feature extraction on a labeled training set of images and train a classifier Linear SVM classifier.
* Apply a color transform and append binned color features, as well as histograms of color, to HOG feature vector. 
* Implement a sliding-window technique and use trained classifier to search for vehicles in images.
* Run the pipeline on a video stream and create a heat map of recurring detections frame by frame to reject outliers and follow detected vehicles.
* Estimate a bounding box for vehicles detected.


### Feature Extraction

#### a. Read in vehicle and non-vehicle images (`.ipynb`: `Step - 1.a`)

Sampled images from the following sources: 
* [Annotated Driving Dataset](https://github.com/udacity/self-driving-car/tree/master/annotations): Dataset 1 and 2 (`.ipynb`: `Step - 1.a.1` - `Step - 1.a.4`)
* [KITTI vision benchmark suite](http://www.cvlibs.net/datasets/kitti/): (`.ipynb`: `Step - 1.a.5`)
* [GTI vehicle image database](http://www.gti.ssr.upm.es/data/Vehicle_database.html): (`.ipynb`: `Step - 1.a.5`)

Here is an example of one of each of the `vehicle` and `non-vehicle` classes, after resized to (64, 64):

![SampleImage](https://github.com/LuLi0077/SDC/blob/master/Vehicle_Detection/output_images/sampleimage.png)


#### b. Extract color features (`.ipynb`: `Step - 1.b`)

* Color histogram: use histograms of pixel intensity as features (`.ipynb`: `Step - 1.b.1`)

Use the sample vehicle image above as an example and assume RGB color space - 

![histogramfeature](https://github.com/LuLi0077/SDC/blob/master/Vehicle_Detection/output_images/histogramfeature.png)

These, collectively, will be our feature vector for this image.

* Explore color spaces: study the distribution of color values in an image by plotting each pixel in some color space (`.ipynb`: `Step - 1.b.2`)

Use the sample vehicle image above as an example, take a look at RGB, HSV and YUV color spaces - 

RGB                        |  HSV                      |  YUV
:-------------------------:|:-------------------------:|:-------------------------:
<img src="https://github.com/LuLi0077/SDC/blob/master/Vehicle_Detection/output_images/RGBcolorspace.png" width="250" height="250">  |  <img src="https://github.com/LuLi0077/SDC/blob/master/Vehicle_Detection/output_images/HSVcolorspace.png" width="250" height="250">  |  <img src="https://github.com/LuLi0077/SDC/blob/master/Vehicle_Detection/output_images/YUVcolorspace.png" width="250" height="250"> 

* Spatial binning of color: create the feature vector using cv2.resize().ravel() (`.ipynb`: `Step - 1.b.3`)


#### c. Extracted HOG features (`.ipynb`: `Step - 1.c`)

* scikit-image HOG: below is the sample vehicle using HOG parameters of `orientations=8`, `pixels_per_cell=(8, 8)` and `cells_per_block=(2, 2)` (`.ipynb`: `Step - 1.c.1`)

![hogsample](https://github.com/LuLi0077/SDC/blob/master/Vehicle_Detection/output_images/hogsample.png)

The HOG visualization is not actually the feature vector, but rather, a representation that shows the dominant gradient direction within each cell with brightness corresponding to the strength of gradients in that cell.

* Combine and normalize features: combine color histograms, spatial binning of color and HOG feature then normalize them before training a classifier (`.ipynb`: `Step - 1.c.2`)

Below shows sample vehicle and non-vehicle images with its raw and normalized features:

![samplefeatures](https://github.com/LuLi0077/SDC/blob/master/Vehicle_Detection/output_images/samplefeatures.png)


### Build a Classifier

Multiple classifiers from sklearn are tested (`.ipynb`: `Step - 2.b`), as well as LeNet-5 from project [Traffic_Sign_Classification](https://github.com/LuLi0077/SDC/tree/master/Traffic_Sign_Classification) (`.ipynb`: `Step - 2.c`) using only 8000 images (4000/class). Various color space and HOG parameter combinations are also tested here (`.ipynb`: `Step - 2.a`), for example: 

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

A different sampling approach is used shown in (`.ipynb`: `Step - 1.a.4.2` and `Step - 1.a.5.2`) -
* When dealing with image data that was extracted from video, there may be sequences of images where the target object (vehicles in this case) appear almost identical in a whole series of images. Sample train and test data from different sources to avoid same images showing up in both sets.
* Increased train set to 20k and test set to 4k images.

After this new sampling strategy - Test Accuracy of LeNet = 0.84450:
![LeNet](https://github.com/LuLi0077/SDC/blob/master/Vehicle_Detection/output_images/LeNet.png)

Neural Net in sklearn seems to work the best and it is used for further testing and tuning. The final classifier selected has test accuracy = .91 (`.ipynb`: `Step - 2.b.1`).


### Sliding Window Search

#### 1. Search and classify

Slide (64, 64) window with 50% overlap over the bottom half of the image. Within each window, extra features, predict its class use the classifier above and draw boxes on where it's classified as a car. (`.ipynb`: `Step - 3.a`)

Image 1                    |  Image 2                    
:-------------------------:|:-------------------------:
<img src="https://github.com/LuLi0077/SDC/blob/master/Vehicle_Detection/output_images/slidewindow1.png" width="400" height="250">  |  <img src="https://github.com/LuLi0077/SDC/blob/master/Vehicle_Detection/output_images/slidewindow2.png" width="400" height="250">  

A more efficient approach is using sub-sampling window search. It extract hog features once and then can be sub-sampled to get all of its overlaying windows. Each window is defined by a scaling factor where a scale of 1 would result in a window that's 8 x 8 cells then the overlap of each window is in terms of the cell distance. This means that a cells_per_step = 2 would result in a search window overlap of 75%.  (`.ipynb`: `Step - 3.b`)

#### 2. Multiple detections & false positives (`.ipynb`: `Step - 3.c`)

Use a heat-map from detections in order to combine overlapping detections and remove false positives. Heat-map is simply adding "heat" (+=1) for all pixels within windows where a positive detection is reported by your classifier. The individual heat-maps for the above images look like this:

Image 1                    |  Image 2                    
:-------------------------:|:-------------------------:
<img src="https://github.com/LuLi0077/SDC/blob/master/Vehicle_Detection/output_images/heatmap1.png" width="400" height="250">  |  <img src="https://github.com/LuLi0077/SDC/blob/master/Vehicle_Detection/output_images/heatmap2.png" width="400" height="250">  

The "hot" parts of the map are where the cars are, and by imposing a threshold, we can reject areas affected by false positives. The outputs are here:

Image 1                    |  Image 2                    
:-------------------------:|:-------------------------:
<img src="https://github.com/LuLi0077/SDC/blob/master/Vehicle_Detection/output_images/box1.png" width="400" height="250">  |  <img src="https://github.com/LuLi0077/SDC/blob/master/Vehicle_Detection/output_images/box2.png" width="400" height="250">  

I also put further restriction on the box size - if it's too small, it can't be a car. 


### Video Implementation (`.ipynb`: `Step - 4`)

Here's a [link to my video result](./project_video.mp4)


### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?


