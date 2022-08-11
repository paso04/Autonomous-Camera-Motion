import csv
import os
import pandas as pd
import numpy as np
from sklearn.preprocessing import LabelEncoder
import sklearn.metrics
from keras.utils import to_categorical

folderPath = '/home/npasini1/Desktop/dVRK_UserStudy/pypredictor_results/labeled'
fileNames = os.listdir(folderPath)
fileList = []
predicted_sx_integer_encoded_tot = np.array([])
predicted_dx_integer_encoded_tot = np.array([])
groundTruth_dx_integer_encoded_tot = np.array([])
groundTruth_sx_integer_encoded_tot = np.array([])
label_encoder = LabelEncoder()

for fileName in fileNames:
        fileList.append(fileName)
        path = os.path.join(folderPath, fileName)
        print('Now reading file: ', path)
        dataframe = pd.read_csv(path)

        #extracting ground truth predictions and model predictions for categorical accuracy evaluation
        predicted_sx_gesture = dataframe['SX Gesture']
        predicted_dx_gesture = dataframe['DX Gesture']
        groundTruth_sx_gesture = dataframe['Real SX Gesture']
        groundTruth_dx_gesture = dataframe['Real DX Gesture']
        array_predicted_sx_gesture = np.array(predicted_sx_gesture)
        array_predicted_dx_gesture = np.array(predicted_dx_gesture)
        array_groundTruth_sx_gesture = np.array(groundTruth_sx_gesture)
        array_groundTruth_dx_gesture = np.array(groundTruth_dx_gesture)

        # print(array_groundTruth_dx_gesture[0])
        # print(array_predicted_dx_gesture[0])

        predicted_sx_integer_encoded = label_encoder.fit_transform(array_predicted_sx_gesture)
        predicted_dx_integer_encoded = label_encoder.fit_transform(array_predicted_dx_gesture)
        groundTruth_sx_integer_encoded = label_encoder.fit_transform(array_groundTruth_sx_gesture)
        groundTruth_dx_integer_encoded = label_encoder.fit_transform(array_groundTruth_dx_gesture)
        print(type(predicted_dx_integer_encoded))
        # print(groundTruth_dx_integer_encoded[0])
        # print(predicted_dx_integer_encoded[0])

        predicted_sx_encoded = to_categorical(predicted_sx_integer_encoded)
        predicted_dx_encoded = to_categorical(predicted_dx_integer_encoded)
        groundTruth_sx_encoded = to_categorical(groundTruth_sx_integer_encoded)
        groundTruth_dx_encoded = to_categorical(groundTruth_dx_integer_encoded)
        # print(groundTruth_dx_encoded[0])
        # print(predicted_dx_encoded[0])

        subset_accuracy_dx = sklearn.metrics.accuracy_score(groundTruth_dx_integer_encoded,predicted_dx_integer_encoded)
        print('Subset accuracy for ' + fileName + ' right hand: ', subset_accuracy_dx)
        subset_accuracy_sx = sklearn.metrics.accuracy_score(groundTruth_sx_integer_encoded,predicted_sx_integer_encoded)
        print('Subset accuracy ' + fileName + ' left hand: ', subset_accuracy_sx)

        predicted_sx_integer_encoded_tot = np.append(predicted_sx_integer_encoded_tot, predicted_sx_integer_encoded)
        predicted_dx_integer_encoded_tot = np.append(predicted_dx_integer_encoded_tot, predicted_dx_integer_encoded)
        groundTruth_sx_integer_encoded_tot = np.append(groundTruth_sx_integer_encoded_tot, groundTruth_sx_integer_encoded)
        groundTruth_dx_integer_encoded_tot = np.append(groundTruth_dx_integer_encoded_tot, groundTruth_dx_integer_encoded)

print('\n #################################################################################################################### \n')

final_subset_accuracy_dx = sklearn.metrics.accuracy_score(groundTruth_sx_integer_encoded_tot,predicted_sx_integer_encoded_tot)
print('FINAL Subset accuracy for right hand: ', final_subset_accuracy_dx)
final_subset_accuracy_sx = sklearn.metrics.accuracy_score(groundTruth_dx_integer_encoded_tot,predicted_dx_integer_encoded_tot)
print('FINAL Subset accuracy ' + fileName + ' left hand: ', final_subset_accuracy_sx)