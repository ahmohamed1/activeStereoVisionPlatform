import rospy
import sys
import numpy as np
import math
import time

class SaveData:
    def __init__(self):
        self.TrackingData = [['time', 'speed', 'diff']]
        self.oldTime = 0.0
        self.timeDiff = 0.0
        self.oldNumber = 0
        self.sameNumber = 0
        self.saveDataAble = False

    def storeData(self,speed, diff):
        if self.oldTime == 0:
            self.timeDiff = 0
            time = rospy.get_time()
            self.oldTime = time
        else:
            time = rospy.get_time()
            self.timeDiff = (time - self.oldTime)+ self.timeDiff
            self.oldTime = time
        self.TrackingData.append([self.timeDiff, speed, diff])

    def checkSpeedRepetedlly(self, speed):
        if speed == self.oldNumber:
            self.sameNumber += 1
        else:
            self.oldNumber = speed
            self.sameNumber = 0
            
    def saveData(self):
        import csv
        import os
        for i in range(10):
            name = '/home/abdulla/TrackingSpeed/'+ str("%0.4f" % self.exponatialGain) + '/'+ str("%0.2f" % self.mapExponatialValue)+'/00' + str(i) +'.csv'
            fileBool = os.path.isfile(name)    # False
            if fileBool == False:
                with open(name, "wb") as f:
                    writer = csv.writer(f)
                    writer.writerows(self.TrackingData)
                break
        print "Data Saved"

    def saveDateAfterFinish(self):
        import csv
        import os
        name = '/home/abdulla/TrackingSpeed/speedTest.csv'
        fileBool = os.path.isfile(name)    # False
        if fileBool == False:
            with open(name, "wb") as f:
                writer = csv.writer(f)
                writer.writerows(self.TrackingData)
