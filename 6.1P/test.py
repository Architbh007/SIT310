#!/usr/bin/env python3

# SIT310 Task 6.1P - Lane Detection Node
# Subscribes to camera images from a pre-recorded bag file and
# detects lane markings using OpenCV image processing techniques.

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError


class LaneDetector:
    def __init__(self):
        rospy.init_node('lane_detector', anonymous=True)

        # Topic name matching the bag file recorded from the Duckiebot
        image_topic = "/akandb/camera_node/image/compressed"

        # Subscribe to receive a new frame each time the bag file publishes one
        self.sub = rospy.Subscriber(
            image_topic,
            CompressedImage,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )
        rospy.loginfo("Lane Detector started. Subscribed to: %s", image_topic)

    def output_lines(self, original_image, lines, line_color=(255, 0, 0)):
        # Draws Hough Transform lines onto the image
        # Green circles = start points, Red circles = end points
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0], l[1]), (l[2], l[3]), line_color, 2, cv2.LINE_AA)
                cv2.circle(output, (l[0], l[1]), 2, (0, 255, 0))
                cv2.circle(output, (l[2], l[3]), 2, (0, 0, 255))
        return output

    def image_callback(self, msg):
        # Decode the incoming compressed image into an OpenCV format
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            rospy.logerr("CvBridge error: %s", e)
            return

        if img_bgr is None:
            return

        height, width = img_bgr.shape[:2]

        # Crop to bottom 50% - removes sky and background, keeps road only
        crop_top = int(height * 0.5)
        cropped = img_bgr[crop_top:height, 0:width]

        # Convert to HSV - makes colour filtering much more reliable
        hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)

        # White filter - targets low saturation, high brightness pixels
        lower_white = np.array([0,   0,  180], dtype=np.uint8)
        upper_white = np.array([180, 60, 255], dtype=np.uint8)
        mask_white = cv2.inRange(hsv, lower_white, upper_white)
        white_filtered_hsv = cv2.bitwise_and(hsv, hsv, mask=mask_white)
        white_filtered_bgr = cv2.cvtColor(white_filtered_hsv, cv2.COLOR_HSV2BGR)

        # Yellow filter - targets the hue range for yellow (20-35 in HSV)
        lower_yellow = np.array([20,  80,  80], dtype=np.uint8)
        upper_yellow = np.array([35, 255, 255], dtype=np.uint8)
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        yellow_filtered_hsv = cv2.bitwise_and(hsv, hsv, mask=mask_yellow)
        yellow_filtered_bgr = cv2.cvtColor(yellow_filtered_hsv, cv2.COLOR_HSV2BGR)

        # Canny edge detection - finds sharp edges across the road region
        gray_cropped = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray_cropped, threshold1=50, threshold2=150)
        edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

        # Hough Transform on white filtered image - detects straight lane lines
        _, _, v_white = cv2.split(white_filtered_hsv)
        lines_white = cv2.HoughLinesP(
            v_white,
            rho=1,
            theta=np.pi / 180,
            threshold=30,
            minLineLength=20,
            maxLineGap=10
        )

        # Hough Transform on yellow filtered image - detects centre dashes
        _, _, v_yellow = cv2.split(yellow_filtered_hsv)
        lines_yellow = cv2.HoughLinesP(
            v_yellow,
            rho=1,
            theta=np.pi / 180,
            threshold=30,
            minLineLength=20,
            maxLineGap=10
        )

        # Draw both sets of lines on the cropped image
        # White lane lines in blue, yellow centre lines in cyan
        combined = np.copy(cropped)
        combined = self.output_lines(combined, lines_white,  line_color=(255, 0,   0))
        combined = self.output_lines(combined, lines_yellow, line_color=(0,   255, 255))

        # Display all five processing stages in separate windows
        cv2.imshow("1 - Cropped Image",         cropped)
        cv2.imshow("2 - White Filtered",         white_filtered_bgr)
        cv2.imshow("3 - Yellow Filtered",        yellow_filtered_bgr)
        cv2.imshow("4 - Canny Edges",            edges_bgr)
        cv2.imshow("5 - Hough Lines (Combined)", combined)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            rospy.signal_shutdown("User quit")


if __name__ == '__main__':
    try:
        detector = LaneDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()