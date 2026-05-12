#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, AprilTagDetectionArray


class Autopilot:
    def __init__(self):
        rospy.init_node("autopilot_node", anonymous=True)

        self.veh = "mybota002437"

        self.STOP_ID = 26
        self.RIGHT_ID = 9
        self.LEFT_ID = 11

        self.busy = False
        self.ignore_tags = False

        rospy.on_shutdown(self.clean_shutdown)

        self.cmd_vel_pub = rospy.Publisher(
            f"/{self.veh}/car_cmd_switch_node/cmd",
            Twist2DStamped,
            queue_size=1
        )

        self.state_pub = rospy.Publisher(
            f"/{self.veh}/fsm_node/mode",
            FSMState,
            queue_size=1
        )

        rospy.Subscriber(
            f"/{self.veh}/apriltag_detector_node/detections",
            AprilTagDetectionArray,
            self.tag_callback,
            queue_size=1
        )

        rospy.loginfo("Autopilot started. Waiting for lane/signs...")

        rospy.sleep(3.0)
        self.set_state("LANE_FOLLOWING")
        rospy.loginfo("Lane following auto-started.")

        rospy.spin()

    def clean_shutdown(self):
        rospy.loginfo("Shutting down. Stopping robot.")
        self.stop_robot(1.0)

    def set_state(self, state):
        rospy.loginfo(f"Switching FSM to {state}")

        msg = FSMState()
        msg.state = state
        rate = rospy.Rate(10)

        for _ in range(10):
            msg.header.stamp = rospy.Time.now()
            self.state_pub.publish(msg)
            rate.sleep()

    def publish_cmd(self, v, omega, duration):
        msg = Twist2DStamped()
        rate = rospy.Rate(10)
        start = rospy.Time.now()

        rospy.loginfo(f"Command: v={v}, omega={omega}, duration={duration}")

        while (rospy.Time.now() - start).to_sec() < duration and not rospy.is_shutdown():
            msg.header.stamp = rospy.Time.now()
            msg.v = v
            msg.omega = omega
            self.cmd_vel_pub.publish(msg)
            rate.sleep()

    def stop_robot(self, duration=0.5):
        self.publish_cmd(0.0, 0.0, duration)

    def start_cooldown(self, seconds):
        self.ignore_tags = True
        rospy.Timer(rospy.Duration(seconds), self.end_cooldown, oneshot=True)

    def end_cooldown(self, event):
        self.ignore_tags = False
        rospy.loginfo("Cooldown finished. Watching for tags again.")

    def tag_callback(self, msg):
        if self.busy or self.ignore_tags:
            return

        if len(msg.detections) == 0:
            return

        detection = msg.detections[0]
        tag_id = detection.tag_id

        if isinstance(tag_id, list):
            tag_id = tag_id[0]

        rospy.loginfo(f"TAG SEEN: ID={tag_id}")

        if tag_id == self.STOP_ID:
            self.do_stop()

        elif tag_id == self.RIGHT_ID:
            self.do_right_turn()

        elif tag_id == self.LEFT_ID:
            self.do_left_turn()

    def do_stop(self):
        self.busy = True
        rospy.loginfo("STOP SIGN detected. Stopping.")

        self.set_state("NORMAL_JOYSTICK_CONTROL")
        self.stop_robot(2.5)

        self.set_state("LANE_FOLLOWING")
        self.start_cooldown(4.0)

        self.busy = False
        rospy.loginfo("STOP complete. Lane following resumed.")

    def do_left_turn(self):
        self.busy = True
        rospy.loginfo("LEFT SIGN detected. Stopping then turning left.")

        self.set_state("NORMAL_JOYSTICK_CONTROL")
        self.stop_robot(0.7)

        self.publish_cmd(0.0, 3.0, 1.4)

        self.stop_robot(0.4)
        self.set_state("LANE_FOLLOWING")
        self.start_cooldown(4.0)

        self.busy = False
        rospy.loginfo("LEFT TURN complete. Lane following resumed.")

    def do_right_turn(self):
        self.busy = True
        rospy.loginfo("RIGHT SIGN detected. Stopping then turning right.")

        self.set_state("NORMAL_JOYSTICK_CONTROL")
        self.stop_robot(0.7)

        self.publish_cmd(0.0, -3.0, 1.4)

        self.stop_robot(0.4)
        self.set_state("LANE_FOLLOWING")
        self.start_cooldown(4.0)

        self.busy = False
        rospy.loginfo("RIGHT TURN complete. Lane following resumed.")


if __name__ == "__main__":
    try:
        Autopilot()
    except rospy.ROSInterruptException:
        pass