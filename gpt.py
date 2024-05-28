#!/usr/bin/env python3
import rclpy
import numpy as np
import cv2
from rclpy.node import Node
import signal
import sys

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header

def detect(image):
    try:
        img = image
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, th = cv2.threshold(gray, 10, 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(th, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        if len(contours) == 1:
            mi, ma, _, _ = cv2.minMaxLoc(gray)
            str = cv2.convertScaleAbs(gray, alpha=255 / (ma - mi), beta=-255 * mi / (ma - mi))
            ret, th = cv2.threshold(str, 28, 255, cv2.THRESH_BINARY)
            contours, hierarchy = cv2.findContours(th, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        for i in range(len(contours)):
            if cv2.contourArea(contours[i]) > 2000:
                break
        contour = contours[i]
        rect = cv2.minAreaRect(contour)
        w, h = int(rect[1][0]), int(rect[1][1])
        box = cv2.boxPoints(rect)
        src = np.array(box, dtype='float32')
        dst = np.array([[0, h - 1], [0, 0], [w - 1, 0], [w - 1, h - 1]], dtype="float32")
        m = cv2.getPerspectiveTransform(src, dst)
        res = cv2.warpPerspective(img, m, (w, h))
        hsv = cv2.cvtColor(res, cv2.COLOR_BGR2HSV)
        R = cv2.inRange(hsv, (0, 51, 51), (10, 255, 255))
        G = cv2.inRange(hsv, (51, 51, 51), (70, 255, 255))
        B = cv2.inRange(hsv, (111, 51, 51), (130, 255, 255))
        Rc = np.count_nonzero(R)
        Gc = np.count_nonzero(G)
        Bc = np.count_nonzero(B)
        M = max(Rc, Gc, Bc)
        if Rc == M:
            return 'R'
        elif Gc == M:
            return 'G'
        else:
            return 'B'
    except Exception as e:
        print(f"Error in detect function: {e}")
        return '0'

class DetermineColor(Node):
    def __init__(self):
        super().__init__('color_detector')
        self.image_sub = self.create_subscription(Image, '/color', self.callback, 10)
        self.color_pub = self.create_publisher(Header, '/rotate_cmd', 10)
        self.bridge = CvBridge()

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            msg = Header()
            msg.stamp = self.get_clock().now().to_msg()
            msg.frame_id = '0'  # default: STOP
            c = detect(image)
            if c == 'B':
                msg.frame_id = '+1'
            elif c == 'R':
                msg.frame_id = '-1'
            else:
                msg.frame_id = '0'
            self.color_pub.publish(msg)
        except CvBridgeError as e:
            self.get_logger().error('Failed to convert image: %s' % e)
        except Exception as e:
            self.get_logger().error('Exception in callback: %s' % e)

def main(args=None):
    rclpy.init(args=args)
    detector = DetermineColor()

    def signal_handler(sig, frame):
        detector.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
