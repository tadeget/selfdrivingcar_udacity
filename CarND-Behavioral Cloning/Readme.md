# **Behavioral Cloning** 

## Writeup Template

### You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---

**Behavioral Cloning Project**

The goals / steps of this project are the following:
* Use the simulator to collect data of good driving behavior
* Build, a convolution neural network in Keras that predicts steering angles from images
* Train and validate the model with a training and validation set
* Test that the model successfully drives around track one without leaving the road
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./examples/center.jpg "center"
[image2]: ./examples/filped_center.jpg "filped_center"
[image3]: ./examples/placeholder_small.png "Recovery Image"
[image4]: ./examples/placeholder_small.png "Recovery Image"
[image5]: ./examples/placeholder_small.png "Recovery Image"
[image6]: ./examples/placeholder_small.png "Normal Image"
[image7]: ./examples/placeholder_small.png "Flipped Image"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/432/view) individually and describe how I addressed each point in my implementation.  

---
### Files Submitted & Code Quality

#### 1. Submission includes all required files and can be used to run the simulator in autonomous mode

My project includes the following files:
* model.py containing the script to create and train the model
* drive.py for driving the car in autonomous mode
* model.h5 containing a trained convolution neural network 
* writeup_report.md or writeup_report.pdf summarizing the results

#### 2. Submission includes functional code
Using the Udacity provided simulator and my drive.py file, the car can be driven autonomously around the track by executing 
```sh
python drive.py model.h5
```

#### 3. Submission code is usable and readable

The model.py file contains the code for training and saving the convolution neural network. The file shows the pipeline I used for training and validating the model, and it contains comments to explain how the code works.

### Model Architecture and Training Strategy

#### 1. An appropriate model architecture has been employed

My iniital approach was to use LeNet, however, after performing some reserach I decided to use the End-to-End CNN  developed by the Nvidia Autonomus car group. The detials of the netwok is given at the "Final Model Architecture" later.

#### 2. Attempts to reduce overfitting in the model

I added a dropout layer at the intersection between the CNN and Dense layer with a probablity of 0.5. Further, to avoid overfit I used a large data set by utilizing the images from lef, center and righ cameras. In addtion, I perfromed  data agumentaion by flipping the images from all the cameras.

The model was trained and validated on different data sets to ensure that the model was not overfitting (code line 75-81). The model was tested by running it through the simulator and ensuring that the vehicle could stay on the track.

#### 3. Model parameter tuning

The model used an adam optimizer, so the learning rate was not tuned manually (model.py line 71).

#### 4. Appropriate training data

Training data was chosen to keep the vehicle driving on the road. I used a combination of center lane driving, recovering from the left and right sides of the road on the first track provided by Udacity. 

For details about how I created the training data, see the next section. 

### Model Architecture and Training Strategy


#### 1. Solution Design Approach

The overall strategy for deriving a model architecture was to conduct research on architectures sutiable for End-to-End autonoms vehicle driving and modify the acttecture if needed.

My first step was to use a convolution neural network model  with one layer to test the overall pipeling for the simulation. But then I selected the architecture provided by the Nvidia autonoms group for the End-to-End driving. I thought this model might be appropriate because it has been tested and has shown good results.

In order to gauge how well the model was working, I split my image and steering angle data into a training and validation set. First, I have used a very high epoch which resulted on overfitting. By adding a Dropout layer after at the first Dense layer and  by redusing the ephoc to 5, the overfitting problem has been handeled. Futher I increased my data set by data agumaentation and utilizing the images from the left and right cameras. 

The final step was to run the simulator to see how well the car was driving around track one. There were a few spots where the vehicle fell off the track especilly at the step curves. To improve the driving behavior in these cases, I modfied the way I drive the vehicle around those step curves and perfrom the training.

At the end of the process, the vehicle is able to drive autonomously around the track without leaving the road.

#### 2. Final Model Architecture

The final model architecture (model.py lines 59-72) consisted of a convolution neural network with the following layers and layer sizes. The model summuary is as follow:

```
Layer (type)                     Output Shape          Param #                 
===================================================================
lambda_1 (Lambda)                (None, 160, 320, 3)   0                     
___________________________________________________________________
cropping2d_1 (Cropping2D)        (None, 66, 320, 3)    0           
___________________________________________________________________
conv2d_1 (Conv2D)                (None, 31, 158, 24)   1824      
___________________________________________________________________
convolution2d_2 (Convolution2D)  (None, 14, 77, 36)    21636                
____________________________________________________________________
convolution2d_3 (Convolution2D)  (None, 5, 37, 48)     43248                
____________________________________________________________________
convolution2d_4 (Convolution2D)  (None, 3, 35, 64)     27712                  
____________________________________________________________________
convolution2d_5 (Convolution2D)  (None, 1, 33, 64)     36928                  
____________________________________________________________________
flatten_1 (Flatten)              (None, 2112)          0                     
____________________________________________________________________
dense_1 (Dense)                  (None, 100)           211300                   
____________________________________________________________________
dense_2 (Dense)                  (None, 50)            5050                           
____________________________________________________________________
dense_3 (Dense)                  (None, 10)            510                           
____________________________________________________________________
dense_4 (Dense)                  (None, 1)             11                             
====================================================================
Total params: 348,219
Trainable params: 348,219
Non-trainable params: 0
```


#### 3. Creation of the Training Set & Training Process

To capture good driving behavior, I recorded two and half laps on track mostly using center lane driving. Sometime I drove  from the left side and right sides of the road back to center so that the vehicle would learn how to auto correct itself at the center lane when there is a drift. 

 Here is an example image of center lane driving:

![alt text][image1]

To augment the dataset, I also flipped images and angles thinking that this would imorove the car with the left turn bias. For example, here is an image that has then been flipped for the image shown above:

![alt text][image2]


After the collection process, I had 11085 number of data points. I then preprocessed this data by normalizing the data using a Lambad function in the range [0,1] and cropped the top half and the region below the hood of the vehicle as they don't provide any infromation other than confusing the model. Futher, by fillping the data 


I finally randomly shuffled the data set and put 20% of the data into a validation set. 

I used this training data for training the model. The validation set helped determine if the model was over or under fitting. The ideal number of epochs was 5 as evidenced from the tranning loss vs validation loos values. I used an adam optimizer so that manually training the learning rate wasn't necessary.