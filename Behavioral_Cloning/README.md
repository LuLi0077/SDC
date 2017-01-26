# Behavioral Cloning

## Overview

Train a deep neural network to mimic human driving in a video game like simulator. The network will:

1. Take in an image from the center, left or right camera of the car, and
2. Output a steering angle for the car.

Similar approach had been tested on real cars by Nvidia according to [End-to-End Deep Learning for Self-Driving Cars](https://arxiv.org/pdf/1604.07316v1.pdf): ([Blog](https://devblogs.nvidia.com/parallelforall/deep-learning-self-driving-cars/))

>In a new automotive application, we have used convolutional neural networks (CNNs) to map the raw pixels from a front-facing camera to the steering commands for a self-driving car. This powerful end-to-end approach means that with minimum training data from humans, the system learns to steer, with or without lane markings, on both local roads and highways. The system can also operate in areas with unclear visual guidance such as parking lots or unpaved roads.


## Collecting Training Data

![Simulator](https://github.com/LuLi0077/SDC/blob/master/Behavioral_Cloning/ForREADME/Simulator.png)

I started collecting training data by driving a car in the simulator in training mode:

- Two laps of nice, centerline driving.
- One lap weaving out to the right and recover and another lap to the left. Since I don't want the car to learn bad behaviours, I turned off data recording when weave out to the side then turn it back on when I steer back to the middle. 

After much struggle, I settled to use what Udacity had shared.


**driving_log.csv - each row in this sheet correlates the image with the steering angle, throttle, brake, and speed of the car.**

![DrivingLog](https://github.com/LuLi0077/SDC/blob/master/Behavioral_Cloning/ForREADME/drivinglog.png)

**Look at few examples for left, center and right camera images**

![SampleData1](https://github.com/LuLi0077/SDC/blob/master/Behavioral_Cloning/ForREADME/SampleData1.png)  
![SampleData2](https://github.com/LuLi0077/SDC/blob/master/Behavioral_Cloning/ForREADME/SampleData2.png)  
![SampleData3](https://github.com/LuLi0077/SDC/blob/master/Behavioral_Cloning/ForREADME/SampleData3.png) 

**Look at 18 center camera images for steering angles [-0.5, 0.5]**

![SampleData4](https://github.com/LuLi0077/SDC/blob/master/Behavioral_Cloning/ForREADME/SampleData4.png)

**Look at the distribution of steering angles**  

![SteeringDistribution](https://github.com/LuLi0077/SDC/blob/master/Behavioral_Cloning/ForREADME/SteeringDist.png)


## Data pre-processing and augmentation

Data collected contains images sampled from the video, paired with the corresponding steering angle. Training with only collected data is not sufficient since most steering angles are 0. The car will drift off the road if it doesn't learn how to recover from the edges. Also, 8000 images will not likely be enough to build a network. Instead of collecting more data, I choose to augment them in the follwing ways: (many ideas are coming from this Keras Blog on [Building powerful image classification models using very little data](https://blog.keras.io/building-powerful-image-classification-models-using-very-little-data.html).)


* Cropping and resizing to match Nvidia network input plane

![Pre-processing](https://github.com/LuLi0077/SDC/blob/master/Behavioral_Cloning/ForREADME/preprocessing.png)

* Random horizontal flips - for both image and steer angle

![RandomFlips](https://github.com/LuLi0077/SDC/blob/master/Behavioral_Cloning/ForREADME/flip.png)

* Changing brightness to simulate day and night conditions

![Brightness](https://github.com/LuLi0077/SDC/blob/master/Behavioral_Cloning/ForREADME/bright.png)

* Shift the images and adjust steering angles

![Shift](https://github.com/LuLi0077/SDC/blob/master/Behavioral_Cloning/ForREADME/shift.png)
![Shift2](https://github.com/LuLi0077/SDC/blob/master/Behavioral_Cloning/ForREADME/shift2.png)

* Using both left and right camera images to simulate the effect of car drifting off to the side and recovering


## Training

Data is split 90/10 into train and validation sets. Then using image and batch generators to read data from each file, augment on the fly and use it to train/validate the model.

Figure below shows the network architecture, which consists of 9 layers, including a normalization layer, 5 convolutional layers, and 3 fully connected layers.


![Nvidia Network Architecture](https://github.com/LuLi0077/SDC/blob/master/Behavioral_Cloning/ForREADME/Nvidia-cnn-architecture.png)


More explaination from [Nvidia Blog](https://devblogs.nvidia.com/parallelforall/deep-learning-self-driving-cars/)):

> The first layer of the network performs image normalization. The normalizer is hard-coded and is not adjusted in the learning process. Performing normalization in the network allows the normalization scheme to be altered with the network architecture, and to be accelerated via GPU processing.

> The convolutional layers are designed to perform feature extraction, and are chosen empirically through a series of experiments that vary layer configurations. We then use strided convolutions in the first three convolutional layers with a 2×2 stride and a 5×5 kernel, and a non-strided convolution with a 3×3 kernel size in the final two convolutional layers.

> We follow the five convolutional layers with three fully connected layers, leading to a final output control value which is the inverse-turning-radius. The fully connected layers are designed to function as a controller for steering, but we noted that by training the system end-to-end, it is not possible to make a clean break between which parts of the network function primarily as feature extractor, and which serve as controller.


**Output files:**
* model.ipynb - The script used to create and train the model.
* drive.py - The script used to drive the car. 
* model.json - The model architecture.
* model.h5 - The model weights.


## Validating

Validate the model by launching the simulator and entering autonomous mode.


## Testing

[Use the second unseen track for testing] (https://youtu.be/wpWCro5v1i0) 

![Test](https://github.com/LuLi0077/SDC/blob/master/Behavioral_Cloning/ForREADME/Test.png)

## For the Future

1. Test other network architectures such as VGG16/ResNet50 and compare performance

2. Transfer learning: fine-tuning the top layers of a pre-trained network

3. [John Chen's Agile Trainer](https://github.com/diyjac/AgileTrainer) - The Agile Trainer 

4. [GTAV Universe](https://openai.com/blog/GTA-V-plus-Universe/) & [DeepGTAV](https://github.com/ai-tor/DeepGTAV) - 
> The Universe integration with Grand Theft Auto V, built and maintained by Craig Quiter's DeepDrive project, is now open-source. To use it, you'll just need a purchased copy of GTA V, and then your Universe agent will be able to start driving a car around the streets of a high-fidelity virtual world.
