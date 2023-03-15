#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw',Image, self.image_callback) #'camera/rgb/image_raw'
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    self.twist = Twist()

  def image_callback(self, msg):
    # fetch image data
    image = self.bridge.imgmsg_to_cv2(msg)
    
    # color filter (yellow)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = numpy.array([ 40, 0, 0])
    upper_yellow = numpy.array([ 120, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # clear all the data but 20 pixel under 3/4 of the image
    h, w, d = image.shape
    search_top = int(3*h/4)
    search_bot = int(3*h/4 + 20)
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0

    # get the centroid and draw it 
    M = cv2.moments(mask)
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
      err = cx - w/2

      # let the movement keep centroid on the mid of lane
      self.twist.linear.x = 0.4
      self.twist.angular.z = -float(err) / 1000
      self.cmd_vel_pub.publish(self.twist)
    else:
      # if no yellow lane forward, turn around.
      self.twist.linear.x = 0
      self.twist.angular.z = 0.3
      self.cmd_vel_pub.publish(self.twist)

    cv2.imshow("window", image)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()