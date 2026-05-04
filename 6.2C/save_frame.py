#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage

class FrameSaver:
    def __init__(self):
        rospy.init_node('frame_saver', anonymous=True)
        self.saved = False
        self.sub = rospy.Subscriber(
            "/mybot002437/camera_node/image/compressed",
            CompressedImage,
            self.callback,
            queue_size=1
        )
        rospy.loginfo("Waiting for a frame...")

    def callback(self, msg):
        if self.saved:
            return
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv2.imwrite("/data/saved_frame.jpg", img)
        rospy.loginfo("Frame saved to /data/saved_frame.jpg")
        self.saved = True
        rospy.signal_shutdown("Frame saved")

if __name__ == '__main__':
    try:
        FrameSaver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass