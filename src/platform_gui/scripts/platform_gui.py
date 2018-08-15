#!/usr/bin/env python
import sys
import os.path
import cv2


import rospy
from platform_vision.helpFunctions import GetImageClass, CreateAnaglyphImage
from std_msgs.msg import Float64

from PyQt5.QtCore import QTimer, QThread, pyqtSignal, Qt
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtWidgets import QApplication, QDialog, QMainWindow
from PyQt5.uic import loadUi


class Platform_GUI(QMainWindow):
    def __init__(self):
        super(Platform_GUI,self).__init__()
        loadUi('/home/abdulla/dev/activeStereoVisionPlatform/src/platform_gui/scripts/platformGUI.ui', self)
        self.getLeftImage = GetImageClass('left')
        self.getRightImage = GetImageClass('right')
        self.createAnaglyphImage = CreateAnaglyphImage()
        self.leftPanMotorPublisher = rospy.Publisher("/left/pan/move", Float64, queue_size=10, latch=True)
        self.leftTiltMotorPublisher = rospy.Publisher("/left/tilt/move", Float64, queue_size=10, latch=True)

        self.rightPanMotorPublisher = rospy.Publisher("/right/pan/move", Float64, queue_size=10, latch=True)
        self.rightTiltMotorPublisher = rospy.Publisher("/right/tilt/move", Float64, queue_size=10, latch=True)
        self.baselineMotorPublisher = rospy.Publisher("/baseline/move", Float64,queue_size=10,latch=True)
        self.leftImage = None
        self.rightImage = None
        self.numbarSavedImages = 0
        # Bottoms Changes
        self.StartStreaming.clicked.connect(self.startCamera)
        self.StopStreaming.clicked.connect(self.stopCamera)
        self.ExitButton.clicked.connect(self.exitFunction)
        self.HomeButton.clicked.connect(self.homeButton)
        self.SaveButtom.clicked.connect(self.saveImages)
        # Dail change
        self.LeftPanDial.sliderReleased.connect(self.updateLeftPanBox)
        self.RightPanDial.sliderReleased.connect(self.updateRightPanBox)
        self.LeftTiltDial.sliderReleased.connect(self.updateLeftTiltBox)
        self.RightTiltDial.sliderReleased.connect(self.updateRightTiltBox)
        self.BaselineHorizontalSlider.sliderReleased.connect(self.updateBaselineBox)
        # QDoubleSpinBox change
        self.LeftPanDoubleSpinBox.editingFinished.connect(self.updateLeftPanDial)
        self.RightPanDoubleSpinBox.editingFinished.connect(self.updateRightPanDial)
        self.LeftTiltDoubleSpinBox.editingFinished.connect(self.updateLeftTiltDial)
        self.RightTiltDoubleSpinBox.editingFinished.connect(self.updateRightTiltDial)
        self.BaselineDoubleSpinBox.editingFinished.connect(self.updateBaselineDial)

        # Cekckbox
        # self.DisparityMapCheckBox.




    def exitFunction(self):
        exit()

    def homeButton(self):
        val = 0.00
        self.LeftPanDial.setValue(val)
        self.RightPanDial.setValue(val)
        self.LeftTiltDial.setValue(val)
        self.RightTiltDial.setValue(val)
        self.BaselineHorizontalSlider.setValue(200)
        self.LeftPanDoubleSpinBox.setValue(val)
        self.RightPanDoubleSpinBox.setValue(val)
        self.LeftTiltDoubleSpinBox.setValue(val)
        self.RightTiltDoubleSpinBox.setValue(val)
        self.BaselineDoubleSpinBox.setValue(200)

        rosVal = Float64()
        rosVal.data = val
        self.leftPanMotorPublisher.publish(rosVal)
        self.rightTiltMotorPublisher.publish(rosVal)
        self.leftTiltMotorPublisher.publish(rosVal)
        self.rightTiltMotorPublisher.publish(rosVal)
        rosVal.data = 200
        self.baselineMotorPublisher.publish(rosVal)
    def saveImages(self):
        fileNameLeft = '/home/abdulla/dev/StereoPlatformImages/' + str(self.numbarSavedImages) + '_LeftImage.png'
        fileNameRight = '/home/abdulla/dev/StereoPlatformImages/' + str(self.numbarSavedImages) + '_RightImage.png'
        fileExist = os.path.isfile(fileNameLeft)
        while fileExist:
            self.numbarSavedImages += 1
            fileNameLeft = '/home/abdulla/dev/StereoPlatformImages/' + str(self.numbarSavedImages) + '_LeftImage.png'
            fileNameRight = '/home/abdulla/dev/StereoPlatformImages/' + str(self.numbarSavedImages) + '_RightImage.png'
            fileExist = os.path.isfile(fileNameLeft)
        cv2.imwrite(fileNameLeft, self.leftImage)
        cv2.imwrite(fileNameRight, self.rightImage)
        # cv2.imwrite('/home/abdulla/dev/data/' + str(i) + '_Depth.png', self.depth)
        print ("Image and Depth image "+ str(self.numbarSavedImages) + " saved seccessfully !!" )
        self.numbarSavedImages += 1

    ################################# Dail change Start ##################################
    def updateLeftPanBox(self):
        val = self.LeftPanDial.value()
        # self.LeftPanTextEdit.setText(str(val))
        self.LeftPanDoubleSpinBox.setValue(val)
        rosVal = Float64()
        rosVal.data = val
        self.leftPanMotorPublisher.publish(rosVal)

    def updateRightPanBox(self):
        val = self.RightPanDial.value()
        # self.LeftPanTextEdit.setText(str(val))
        self.RightPanDoubleSpinBox.setValue(val)
        rosVal = Float64()
        rosVal.data = val
        self.rightPanMotorPublisher.publish(rosVal)


    def updateLeftTiltBox(self):
        val = self.LeftTiltDial.value()
        # self.LeftPanTextEdit.setText(str(val))
        self.LeftTiltDoubleSpinBox.setValue(val)
        if self.LinkTiltCheckBox.checkState():
            self.moveBothTiltTogather(val)
        else:
            rosVal = Float64()
            rosVal.data = val
            self.leftTiltMotorPublisher.publish(rosVal)

    def updateRightTiltBox(self):
        val = self.RightTiltDial.value()
        # self.LeftPanTextEdit.setText(str(val))
        self.RightTiltDoubleSpinBox.setValue(val)
        if self.LinkTiltCheckBox.checkState():
            self.moveBothTiltTogather(val)
        else:
            rosVal = Float64()
            rosVal.data = val
            self.rightTiltMotorPublisher.publish(rosVal)

    def updateBaselineBox(self):
        val = self.BaselineHorizontalSlider.value()
        # self.LeftPanTextEdit.setText(str(val))
        self.BaselineDoubleSpinBox.setValue(val)
        rosVal = Float64()
        rosVal.data = val
        self.baselineMotorPublisher.publish(rosVal)

    ################################# Dail change Finish ##################################
    ############################# DoubleSpinBox change Start ##############################
    def updateLeftPanDial(self):
        val = self.LeftPanDoubleSpinBox.value()
        self.LeftPanDial.setValue(val)
        rosVal = Float64()
        rosVal.data = val
        self.leftPanMotorPublisher.publish(rosVal)


    def updateRightPanDial(self):
        val = self.RightPanDoubleSpinBox.value()
        self.RightPanDial.setValue(val)
        rosVal = Float64()
        rosVal.data = val
        self.rightPanMotorPublisher.publish(rosVal)

    def updateLeftTiltDial(self):
        val = self.LeftTiltDoubleSpinBox.value()
        self.LeftTiltDial.setValue(val)
        if self.LinkTiltCheckBox.checkState():
            self.moveBothTiltTogather(val)
        else:
            rosVal = Float64()
            rosVal.data = -val
            self.leftTiltMotorPublisher.publish(rosVal)

    def updateRightTiltDial(self):
        val = self.RightTiltDoubleSpinBox.value()
        self.RightTiltDial.setValue(val)
        if self.LinkTiltCheckBox.checkState():
            self.moveBothTiltTogather(val)
        else:
            rosVal = Float64()
            rosVal.data = val
            self.rightTiltMotorPublisher.publish(rosVal)

    def moveBothTiltTogather(self,val):
        """Move both motors in the same times"""
        rosVal = Float64()
        rosVal.data = val
        self.rightTiltMotorPublisher.publish(rosVal)
        rosVal.data = -val
        self.leftTiltMotorPublisher.publish(rosVal)
        self.RightTiltDoubleSpinBox.setValue(val)
        self.LeftTiltDoubleSpinBox.setValue(val)
        self.LeftTiltDial.setValue(val)
        self.RightTiltDial.setValue(val)

    def updateBaselineDial(self):
        val = self.BaselineDoubleSpinBox.value()
        self.BaselineHorizontalSlider.setValue(val)
        rosVal = Float64()
        rosVal.data = val
        self.baselineMotorPublisher.publish(rosVal)
    ############################ DoubleSpinBox change Finish ##############################


    ############################ Image Processing ##############################
    def showImageBaseLabel(self, labelName, image):
        labelName.setPixmap(QPixmap.fromImage(image))
        labelName.setScaledContents(True)

    def startCamera(self):
        # self.capture = cv2.VideoCapture(0)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(1)

    def stopCamera(self):
        self.timer.stop()

    def update_frame(self):
        # ret, self.leftImage = self.capture.read()
        self.leftImage = self.getLeftImage.getImage()
        self.rightImage = self.getRightImage.getImage()

        leftImage=cv2.flip(self.leftImage,1)
        rightImage=cv2.flip(self.rightImage,1)

        left_image = self.displayImage(leftImage,1)
        right_Image = self.displayImage(rightImage,1)
        self.showImageBaseLabel(self.left_display, left_image)
        self.showImageBaseLabel(self.right_display, right_Image)

        if self.DisparityMapCheckBox.checkState():
            """compute Disparity"""
            pass
        else:
            self.anaglyphImage = self.createAnaglyphImage.anaglyph(self.leftImage,self.rightImage)
            anaglyphImage = cv2.flip(self.anaglyphImage,1)
            anaglyph_image = self.displayImage(anaglyphImage,1)
            self.showImageBaseLabel(self.disparity_display, anaglyph_image)

    def displayImage(self, img, window=1):
        qformat = QImage.Format_Indexed8
        if len(img.shape)==3: #[0]=row, [1]= col,  [2] = channels
            if img.shape[2] == 4:
                qformat=QImage.Format_RGBA8888
            else:
                qformat = QImage.Format_RGB888
        outImage = QImage(img, img.shape[1], img.shape[0], img.strides[0],qformat)
        #BGR >> RGB
        outImage= outImage.rgbSwapped()

        return outImage
        # if window == 1:
        #     self.left_display.setPixmap(QPixmap.fromImage(outImage))
        #     self.left_display.setScaledContents(True)

if __name__== '__main__' :
    rospy.init_node('Platform_GUI', anonymous = False)

    app=QApplication(sys.argv)
    window= Platform_GUI()
    window.setWindowTitle('Platform Status')
    window.show()
    sys.exit(app.exec_())
