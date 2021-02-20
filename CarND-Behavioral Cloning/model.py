import os
import csv
import matplotlib.pyplot as plt

samples = []
data_loc = '../data_dri_4/IMG/'
img_log =  '../data_dri_4/driving_log.csv'
with open(img_log) as csvfile:
#with open('./data/driving_log.csv') as csvfile:
    reader = csv.reader(csvfile)
    for line in reader:
        samples.append(line)
        
from sklearn.model_selection import train_test_split
train_samples, validation_samples = train_test_split(samples,test_size=0.2)

import cv2
import numpy as np
import sklearn

agument_flip = 1
def generator(samples, batch_size = 32):
    num_samples = len(samples)
    offset_val = np.array([0.0,0.2,-0.2],dtype=np.float32)
    while 1:
        sklearn.utils.shuffle(samples)
        for offset in range(0,num_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]
            images = []
            angles = []
            
            for batch_sample in batch_samples:
                for i in range(0,3):
                    #print('%%%%%%%%%%%%%%%%%',i)
                    image_name = data_loc + batch_sample[i].split('\\')[-1]
                    RGB_image = cv2.cvtColor(cv2.imread(image_name),cv2.COLOR_BGR2RGB)
                    center_angle = float(batch_sample[3])+offset_val[i]
                    images.append(RGB_image)
                    angles.append(center_angle)
                    if agument_flip == 1:
                        images.append(cv2.flip(RGB_image,1))
                        angles.append((center_angle)*-1)
                   
            X_train = np.array(images)
            #print(X_train.shape)
            y_train = np.array(angles)
            yield sklearn.utils.shuffle(X_train,y_train)

batch_size = 32

train_generator = generator(train_samples, batch_size = batch_size)
validation_generator = generator(validation_samples, batch_size = batch_size)

ch,row, col = 3, 160, 320 # trimed image

from keras.models import Sequential
from keras.layers import Flatten, Dense, Lambda, Cropping2D, Convolution2D, Dropout

model = Sequential()
model.add(Lambda(lambda x:x/255.5 - 0.5, input_shape = (row,col,ch),output_shape=(row,col,ch)))
model.add(Cropping2D(cropping=((70,24),(0,0))))
model.add(Convolution2D(24,5,5,subsample=(2,2),activation='relu'))
model.add(Convolution2D(36,5,5,subsample=(2,2),activation='relu'))
model.add(Convolution2D(48,5,5,subsample=(2,2),activation='relu'))
model.add(Convolution2D(64,3,3,activation='relu'))
model.add(Convolution2D(64,3,3,activation='relu'))
model.add(Flatten())
model.add(Dropout(0.5))
model.add(Dense(100))
model.add(Dense(50))
model.add(Dense(10))
model.add(Dense(1))
print(model.summary())
print('****************',train_generator)
model.compile(loss='mse',optimizer='adam')
history_object= model.fit_generator(train_generator,
                    steps_per_epoch= np.ceil(len(train_samples)/batch_size),
                    validation_data = validation_generator,
                    validation_steps= np.ceil(len(validation_samples)/batch_size),
                    epochs=5,verbose=1)
model.save('model.h5')


print(history_object.history.keys())
print('Loss')
print(history_object.history['loss'])
print('Validation Loss')
print(history_object.history['val_loss'])

#plt.plot(history_object.history['loss'])
#plt.plot(history_object.history['val_loss'])
#plt.title('model mean squared error loss')
#plt.ylabel('mean squared error loss')
#plt.xlabel('epoch')
#plt.legend(['training set', 'validation set'], loc='upper right')
#plt.show()
