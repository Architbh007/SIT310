#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import AprilTagDetectionArray

class Target_Follower:
    def __init__(self):
        
        rospy.init_node('target_follower_node', anonymous=True)
        rospy.on_shutdown(self.clean_shutdown)

       
        self.cmd_vel_pub = rospy.Publisher('/mybota002437/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/mybota002437/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)

        #States
        self.STATE_SEEKING  = "SEEKING"
        self.STATE_TRACKING = "TRACKING"
        self.state          = self.STATE_SEEKING
        self.last_seen_x    = None  

        # --- Tunable parameters ---
        self.SEEK_OMEGA    = 1.4   # Slower seeking so it doesn't overshoot
        self.OMEGA_GAIN    = 2.0   # P-gain for tracking rotation
        self.OMEGA_MIN     = 0.3   # Min omega to overcome friction
        self.OMEGA_MAX     = 4.0   # Max omega cap
        self.CENTER_THRESH = 0.05  # Dead-zone: don't rotate if x error within this
        # --------------------------

        rospy.spin()

    def tag_callback(self, msg):
        self.move_robot(msg.detections)

    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v     = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def publish_velocity(self, v, omega):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v     = v
        cmd_msg.omega = omega
        self.cmd_vel_pub.publish(cmd_msg)

    def seek(self):
        # Spin toward the side where tag was last seen
        if self.last_seen_x is not None:
            if self.last_seen_x > 0:
                seek_direction = -1  # tag was on right, spin right
            else:
                seek_direction = 1   # tag was on left, spin left
        else:
            seek_direction = 1       # no memory yet, default spin left

        rospy.loginfo("[SEEKING] No tag detected - rotating to find object...")
        self.publish_velocity(v=0.0, omega=self.SEEK_OMEGA * seek_direction)

    def track(self, x):
        if abs(x) < self.CENTER_THRESH:
            omega = 0.0
            rospy.loginfo("[TRACKING] Tag centred - holding position.")
        else:
            omega = -self.OMEGA_GAIN * x

            if 0 < omega < self.OMEGA_MIN:
                omega = self.OMEGA_MIN
            elif -self.OMEGA_MIN < omega < 0:
                omega = -self.OMEGA_MIN

            omega = max(-self.OMEGA_MAX, min(self.OMEGA_MAX, omega))

            direction = "right" if omega < 0 else "left"
            rospy.loginfo("[TRACKING] x=%.3f → turning %s (omega=%.3f)", x, direction, omega)

        self.publish_velocity(v=0.0, omega=omega)

    def move_robot(self, detections):
        if len(detections) == 0:
            if self.state != self.STATE_SEEKING:
                rospy.loginfo("Tag lost - switching to SEEKING state.")
                self.state = self.STATE_SEEKING
            self.seek()
        else:
            if self.state != self.STATE_TRACKING:
                rospy.loginfo("Tag found - switching to TRACKING state.")
                self.state = self.STATE_TRACKING
            x = detections[0].transform.translation.x
            self.last_seen_x = x  # remember where we last saw the tag
            self.track(x)

if __name__ == '__main__':
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass