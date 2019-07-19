# define a class for training data preparation
import csv
import cv2
import numpy as np
from sklearn.utils import shuffle
from sklearn.model_selection import train_test_split
import pdb

class Data_processing:
    def __init__(self, base_path='./simulator_data',batch_size = 128):
        self.data = []
        self.train = []
        self.valid = []
        self.base_path = base_path # have a default path to be used without enabling GPU
        self.image_path = self.base_path + '/IMG/'
        self.log_path = self.base_path + '/driving_log.csv'
        self.correction_angle = 0.2
        self.batch_size = batch_size
        
    # import data
    def import_data(self):
        with open(self.log_path) as csvfile:
            csvreader = csv.reader(csvfile)
            next(csvreader)
            
            for row in csvreader:
                self.data.append(row)
            
        return None
        
    # splitting data into training and validation sets
    def data_sets(self,split_ratio=0.2):
        self.train,self.valid = train_test_split(self.data,test_size=split_ratio)
        
        return None
    
    # using generator to generate batch data
    def data_generator(self, samples):
        num_sample = len(samples)
        
        while True:
            shuffle(samples)
            #for each batch of samples, augment the data, and generate final batch
            for offset in range(0, num_sample, self.batch_size):
                batch_samples = samples[offset:offset+self.batch_size]
                images, steering_angles = [], []
                
                for batch_sample in batch_samples:
                    aug_images, aug_angles = self.augment_batch(batch_sample)
                    images.extend(aug_images)
                    steering_angles.extend(aug_angles)
                    
                X_train, y_train = np.array(images), np.array(steering_angles)
                yield shuffle(X_train,y_train)
    
    # the actual training data generator object
    def train_generator(self):
        return self.data_generator(samples = self.train)
    
    # the actual validation data generator object
    def valid_generator(self):
        return self.data_generator(samples = self.valid)
    
    # augment the image and steering angle for each batch
    def augment_batch(self,batch_sample):
        steering_angle = np.float32(batch_sample[3])
        images, steering_angles=[],[]
        
        for image_index in range(3):
            image_name = batch_sample[image_index].split('/')[-1]
            
            # for each image of left, center, right, do image processing
            image = cv2.imread(self.image_path + image_name)
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image_crop = image_rgb[60:130,:]
            image_resize = cv2.resize(image_crop,(160,70))
            images.append(image_resize) # store the first image
            
            # store the first angle
            if image_index == 1:
                steering_angles.append(steering_angle + self.correction_angle)
            elif image_index == 2:
                steering_angles.append(steering_angle - self.correction_angle)
            else :
                steering_angles.append(steering_angle)
               
            # for the center image, do more augmentation
            if image_index == 0:
                image_flip = cv2.flip(image_resize,1)
                images.append(image_flip)
                steering_angles.append(-steering_angle) # store the second image, totally 4 images with their angles stored
            
            #pdb.set_trace()
        return images, steering_angles