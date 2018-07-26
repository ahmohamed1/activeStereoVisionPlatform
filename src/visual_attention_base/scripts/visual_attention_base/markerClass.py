#!/usr/bin/env python

import rospy
import numpy as np

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA


dummyList = []

for i in range(0,5):
    pose2D = np.random.randint(0,2000,(2)).tolist()
    pose3D = np.random.randint(-1.1,1.0,(3)).tolist()
    probability2D = np.random.randint(0.0,100.0,(1)).tolist()
    probability3D = np.random.randint(0.0,100.0,(1)).tolist()
    dummyList.append([probability2D, probability3D, pose2D, pose3D])

class CreateVisualizationMarker:

    def __init__(self, frame):
        self.marker_publisher = rospy.Publisher('visualization_cognitive_map', MarkerArray, queue_size=5)
        self.frame = frame
        self.dummyCount

        
    def createMarker(self, idea, text, position):
        marker_text = Marker()
        marker_text.header.frame_id = self.frame
        marker_text.type = Marker.TEXT_VIEW_FACING
        marker_text.action = Marker.ADD
        marker_text.scale.x = 0.2
        marker_text.scale.y = 0.2
        marker_text.scale.z = 0.2
        marker_text.color.a = 1.0
        marker_text.color.r = 1.0
        marker_text.color.g = 1.0
        marker_text.color.b = 0.0
        marker_text.pose.orientation.w = 1.0
        marker_text.pose.position.x = position[0]
        marker_text.pose.position.y = position[1]
        marker_text.pose.position.z = position[2]
        marker_text.id = idea
        marker_text.text = text
        return marker_text

    def publishMarkerArray(self, List):
        markerArray = MarkerArray()
        for i, list in enumerate(List):
            text = self.convertInformationToString(i, list[0][0], list[1][0],list[2], list[3])
            marker = self.createMarker(i, text, list[3])
            markerArray.markers.append(marker)
            # self.dummyCount += 1
        self.marker_publisher.publish(markerArray)


    def convertInformationToString(self, idea, probability2D, probability3D, pose2D, pose3D):
        string = 'Tomato ID: ' + str(idea)
        string = string+ '\n2D propability: ' + str(probability2D) + '%'
        string = string + '\n2D pose: ' + str(pose2D)
        string = string + '\n3D propability: ' + str(probability3D) + '%'
        string = string + '\n3D pose: ' + str(pose3D)
        return string


def main():
  rospy.init_node('visualAttention_information')
  createVisualizationMarker = CreateVisualizationMarker('map')
  while not rospy.is_shutdown():
      createVisualizationMarker.publishMarkerArray(dummyList)


if __name__ == '__main__':
  main()
