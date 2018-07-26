import numpy as np
from sklearn import linear_model
import sklearn.metrics as sm
from sklearn.preprocessing import PolynomialFeatures
from sklearn.naive_bayes import MultinomialNB
import pickle
import cv2

# import pyximport; pyximport.install()
# import create_mask

# Input file containing data
input_file = '../config/dataLabel.csv'
# Load the data from the input file
data = np.loadtxt(input_file, delimiter=',')
X, y = data[:, :-1], data[:, -1]

# print (X[5:20], y[5:20])

# Split data into training and testing
num_training = int(0.8 * len(X))
num_test = len(X) - num_training
# Training data
X_train, y_train = X[:num_training], y[:num_training]
# Test data
X_test, y_test = X[num_training:], y[num_training:]


def computerMask(image, model , width=420,height = 640):
    outputImage = np.zeros((width,height))
    # print("pixels", image[width,height])
    for i in range(height-1):
        for j in range(width-1):
            B, G, R = image[j,i]
            new_data = [[B, G, R]]
            prob = model.predict(new_data)
            if prob > 0.6:
                outputImage[j,i] = 1
            else:
                outputImage[j,i] = 0
    return outputImage

def linearRegression(X_train, y_train, X_test, y_test):
    # Create the linear regressor model
    linear_regressor = linear_model.LinearRegression()
    # Train the model using the training sets
    linear_regressor.fit(X_train, y_train)
    # Predict the output
    y_test_pred = linear_regressor.predict(X_test)
    # print ('predict= ', y_test_pred)
    # Measure performance
    print("Linear Regressor performance:")
    print("Mean absolute error =", round(sm.mean_absolute_error(y_test,
    y_test_pred), 2))
    print("Mean squared error =", round(sm.mean_squared_error(y_test,
    y_test_pred), 2))
    print("Median absolute error =", round(sm.median_absolute_error(y_test,
    y_test_pred), 2))
    print("Explained variance score =",
    round(sm.explained_variance_score(y_test, y_test_pred), 2))
    print("R2 score =", round(sm.r2_score(y_test, y_test_pred), 2))

    # save the model to disk
    filename = '../config/linear_regressor.pkl'
    pickle.dump(linear_regressor, open(filename, 'wb'), protocol=2)

    return linear_regressor



def GaussianNaiveBayes(X_train, y_train, X_test, y_test):
    model = MultinomialNB()
    model.fit(X_train, y_train)

    # Predict the output
    y_test_pred = model.predict(X_test)

    print("\nNaive Bayes classifier performance:")
    print("Mean absolute error =", round(sm.mean_absolute_error(y_test,
    y_test_pred), 2))
    print("Mean squared error =", round(sm.mean_squared_error(y_test,
    y_test_pred), 2))
    print("Median absolute error =", round(sm.median_absolute_error(y_test,
    y_test_pred), 2))
    print("Explained variance score =",
    round(sm.explained_variance_score(y_test, y_test_pred), 2))
    print("R2 score =", round(sm.r2_score(y_test, y_test_pred), 2))

    # save the model to disk
    filename = '../config/GaussianNaiveBayes.pkl'
    with open(filename, 'wb') as file:
        pickle.dump(model, file, protocol=2)
    print(' Model Saved !!')
    print (model)
    return model




height = 640
width = 420
image = cv2.imread('../data/73_Image.png')
image = cv2.resize(image, (height,width))

x = 254
y = 115
a = 10

# rect = (x-a, y-a ,x+a,y+a)
# mask = np.zeros(image.shape[:2],np.uint8)
# bgdModel = np.zeros((1,65),np.float64)
# fgdModel = np.zeros((1,65),np.float64)
#
# cv2.grabCut(image,mask,rect,bgdModel,fgdModel,5,cv2.GC_INIT_WITH_RECT)
# mask2 = np.where((mask==2)|(mask==0),0,1).astype('uint8')
# image = image*mask2[:,:,np.newaxis]
# cv2.imshow('image', image)
# cv2.waitKey(0)

# GaussianNaiveBayes(X_train, y_train, X_test, y_test)
# linearRegression(X_train, y_train, X_test, y_test)
Model = pickle.load(open('../config/linear_regressor.pkl', 'rb'))
# Model = pickle.load(open('../config/GaussianNaiveBayes.pkl', 'rb'))
# print (bayesModel)
# new_data = [[30, 25, 80]]
# print(Model.predict(new_data))


# import multiprocessing
# pool = multiprocessing.Pool(processes=4)
# r = pool.map(computerMask,image)
import time
start = time.time()
mask = computerMask(image, Model, width, height)
duration = time.time() - start
print(duration)
mask = np.uint8(mask)
outputMask = cv2.bitwise_and(image, image,mask=mask)
cv2.imshow('mask',outputMask )
cv2.waitKey(0)
