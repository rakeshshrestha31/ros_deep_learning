#!/usr/bin/env python
from __future__ import print_function

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from vision_msgs.msg import Detection2DArray

class BoundingBoxAnnotator:
  def __init__(self):
    self.image_pub = rospy.Publisher("~image_with_bbox", Image, queue_size=2)
    self.bridge = CvBridge()

    self.image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
    self.detection_sub = message_filters.Subscriber('/detectnet/detections', Detection2DArray)
		
    self.time_sync = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.detection_sub], 15, 0.3) # , allow_headerless=True)
    self.time_sync.registerCallback(self.callback)

  def callback(self, image, detections):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(image)
    except CvBridgeError as e:
      rospy.logerror(e)
    
    for detection in detections.detections:
      center_x = int(detection.bbox.center.x)
      center_y = int(detection.bbox.center.y)
      size_x = int(detection.bbox.size_x)
      size_y = int(detection.bbox.size_y)
      top_x = center_x - size_x / 2
      top_y = center_y - size_y / 2
      bottom_x = center_x + size_x / 2
      bottom_y = center_y + size_y / 2

      cv_image = cv2.rectangle(
        cv_image, 
        (top_x, top_y),
        (bottom_x, bottom_y),
        (255, 0, 0),
        5
      )
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "rgb8"))
    except CvBridgeError as e:
      rospy.logerror(e)

if __name__ == "__main__":
  rospy.init_node("bbox_annotator")
  bbox_annotator = BoundingBoxAnnotator()
  rospy.spin()
