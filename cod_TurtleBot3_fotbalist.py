import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from geometry_msgs.msg import Twist

class BallFollower:
    def __init__(self):
        rospy.init_node('ball_follower', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()
        self.detecting_ball = True  
        self.threshold_area = 1000  
    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
            return
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_color = np.array([0, 50, 50])
        upper_color = np.array([179, 255, 255])
        mask = cv2.inRange(hsv, lower_color, upper_color)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if self.detecting_ball:
            if not contours:
                self.twist.angular.z = 0.5  
                self.cmd_vel_pub.publish(self.twist)
            else:
                valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > self.threshold_area]
                if valid_contours:
                    largest_contour = max(valid_contours, key=cv2.contourArea)
                    M = cv2.moments(largest_contour)
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    self.twist.angular.z = 0
                    self.twist.linear.x = 0.2
                    self.cmd_vel_pub.publish(self.twist)
                    self.detecting_ball = False  
        else:
            self.twist.linear.x = 0.5  
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(1)  
            self.stop_robot()  
            self.detecting_ball = True  
    def stop_robot(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(0.1)  
if __name__ == '__main__':
    try:
        ball_follower = BallFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass