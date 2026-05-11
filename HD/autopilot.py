#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import AprilTagDetectionArray

class Autopilot:
    def __init__(self):
        rospy.init_node('autopilot_node', anonymous=True)

        self.robot_state = "LANE_FOLLOWING"
        self.ignore_tags = False  # cooldown flag to prevent double triggering

        rospy.on_shutdown(self.clean_shutdown)

        ###### REPLACE "mybota002437" IF NEEDED ######
        self.cmd_vel_pub = rospy.Publisher('/mybota002437/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.state_pub = rospy.Publisher('/mybota002437/fsm_node/mode', FSMState, queue_size=1)
        rospy.Subscriber('/mybota002437/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        #############################################

        rospy.spin()

    def tag_callback(self, msg):
        if self.robot_state != "LANE_FOLLOWING":
            return
        if self.ignore_tags:
            return
        self.move_robot(msg.detections)

    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def set_state(self, state):
        self.robot_state = state
        state_msg = FSMState()
        state_msg.header.stamp = rospy.Time.now()
        state_msg.state = self.robot_state
        self.state_pub.publish(state_msg)

    def publish_cmd(self, v, omega, duration):
        """Send a velocity command for a fixed duration."""
        cmd_msg = Twist2DStamped()
        rate = rospy.Rate(10)
        start = rospy.Time.now()
        while (rospy.Time.now() - start).to_sec() < duration:
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.v = v
            cmd_msg.omega = omega
            self.cmd_vel_pub.publish(cmd_msg)
            rate.sleep()

    def start_cooldown(self, seconds):
        """Ignore tags for a few seconds after handling one."""
        self.ignore_tags = True
        rospy.Timer(rospy.Duration(seconds), self.end_cooldown, oneshot=True)

    def end_cooldown(self, event):
        self.ignore_tags = False
        rospy.loginfo("Cooldown done, watching for tags again.")

    def do_stop(self):
        rospy.loginfo("STOP SIGN detected — stopping.")
        self.set_state("NORMAL_JOYSTICK_CONTROL")

        self.stop_robot()
        rospy.sleep(2.0)  # wait at stop sign

        # Move forward a bit so sign leaves camera view
        self.publish_cmd(0.2, 0.0, 1.5)
        self.stop_robot()

        self.set_state("LANE_FOLLOWING")
        self.start_cooldown(5.0)
        rospy.loginfo("Resuming lane following after stop.")

    def do_left_turn(self):
        rospy.loginfo("LEFT TURN sign detected.")
        self.set_state("NORMAL_JOYSTICK_CONTROL")

        # Move forward to reach the intersection line
        self.publish_cmd(0.2, 0.0, 1.0)

        # Turn left (positive omega)
        self.publish_cmd(0.15, 2.5, 1.2)

        # Straighten out
        self.publish_cmd(0.2, 0.0, 0.5)
        self.stop_robot()

        self.set_state("LANE_FOLLOWING")
        self.start_cooldown(5.0)
        rospy.loginfo("LEFT TURN complete, resuming lane following.")

    def do_right_turn(self):
        rospy.loginfo("RIGHT TURN sign detected.")
        self.set_state("NORMAL_JOYSTICK_CONTROL")

        # Move forward to reach the intersection line
        self.publish_cmd(0.2, 0.0, 1.0)

        # Turn right (negative omega)
        self.publish_cmd(0.15, -2.5, 1.2)

        # Straighten out
        self.publish_cmd(0.2, 0.0, 0.5)
        self.stop_robot()

        self.set_state("LANE_FOLLOWING")
        self.start_cooldown(5.0)
        rospy.loginfo("RIGHT TURN complete, resuming lane following.")

    def move_robot(self, detections):
        if len(detections) == 0:
            return

        for detection in detections:
            tag_id = detection.tag_id
            rospy.loginfo(f"Tag detected: ID={tag_id}")

            if tag_id == 1:       # Stop sign
                self.do_stop()
                break
            elif tag_id == 10:    # Left turn
                self.do_left_turn()
                break
            elif tag_id == 9:     # Right turn
                self.do_right_turn()
                break

if __name__ == '__main__':
    try:
        autopilot_instance = Autopilot()
    except rospy.ROSInterruptException:
        pass