#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, AprilTagDetectionArray
from sensor_msgs.msg import Range


class Autopilot:
    def __init__(self):
        rospy.init_node("autopilot_node", anonymous=True)

        self.veh = "mybota002437"

        self.busy = False
        self.ignore_tags = False

        self.current_distance = 999.0
        self.obstacle_threshold = 0.20

        self.valid_tag_visible = False
        self.valid_tag_last_seen = rospy.Time.now()

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

        rospy.Subscriber(
            f"/{self.veh}/front_center_tof_driver_node/range",
            Range,
            self.distance_callback,
            queue_size=1
        )

        rospy.loginfo("Autopilot started.")

        # Wait for FSM + lane following nodes to fully initialize
        rospy.sleep(3.0)

        # Automatically enable lane following
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

    def distance_callback(self, msg):
        self.current_distance = msg.range

        if self.busy:
            return

        # If a valid tag was recently detected,
        # do not treat the sign as obstacle
        tag_recent = (
            rospy.Time.now() - self.valid_tag_last_seen
        ).to_sec() < 1.0

        if (
            self.current_distance < self.obstacle_threshold
            and not tag_recent
        ):
            self.do_obstacle_wait()

    def tag_callback(self, msg):
        if self.busy or self.ignore_tags:
            return

        if len(msg.detections) == 0:
            self.valid_tag_visible = False
            return

        for detection in msg.detections:

            tag_id = detection.tag_id

            if isinstance(tag_id, list):
                tag_id = tag_id[0]

            rospy.loginfo(f"Tag detected: ID={tag_id}")

            if tag_id in [9, 10, 26]:
                self.valid_tag_visible = True
                self.valid_tag_last_seen = rospy.Time.now()

            # LEFT TURN
            if tag_id == 10:
                self.do_left_turn()
                break

            # RIGHT TURN
            elif tag_id == 9:
                self.do_right_turn()
                break

            # STOP SIGN
            elif tag_id == 26:
                self.do_stop_until_removed()
                break

    def start_cooldown(self, seconds):
        self.ignore_tags = True

        rospy.Timer(
            rospy.Duration(seconds),
            self.end_cooldown,
            oneshot=True
        )

    def end_cooldown(self, event):
        self.ignore_tags = False
        rospy.loginfo("Cooldown finished.")

    def do_left_turn(self):
        self.busy = True

        rospy.loginfo("LEFT TURN detected.")

        self.set_state("NORMAL_JOYSTICK_CONTROL")

        rospy.sleep(0.3)

        self.stop_robot(0.3)

        # Turn left immediately
        self.publish_cmd(0.0, 4.0, 1.3)

        self.stop_robot(0.3)

        self.set_state("LANE_FOLLOWING")

        self.start_cooldown(4.0)

        self.busy = False

        rospy.loginfo("LEFT TURN complete.")

    def do_right_turn(self):
        self.busy = True

        rospy.loginfo("RIGHT TURN detected.")

        self.set_state("NORMAL_JOYSTICK_CONTROL")

        rospy.sleep(0.3)

        self.stop_robot(0.3)

        # Turn right immediately
        self.publish_cmd(0.0, -4.0, 1.3)

        self.stop_robot(0.3)

        self.set_state("LANE_FOLLOWING")

        self.start_cooldown(4.0)

        self.busy = False

        rospy.loginfo("RIGHT TURN complete.")

    def do_stop_until_removed(self):
        self.busy = True

        rospy.loginfo("STOP SIGN detected.")

        self.set_state("NORMAL_JOYSTICK_CONTROL")

        rate = rospy.Rate(10)

        while (
            self.current_distance < self.obstacle_threshold
            and not rospy.is_shutdown()
        ):
            self.stop_robot(0.1)
            rate.sleep()

        rospy.loginfo("Stop sign removed.")

        self.set_state("LANE_FOLLOWING")

        self.start_cooldown(3.0)

        self.busy = False

    def do_obstacle_wait(self):
        self.busy = True

        rospy.loginfo("UNKNOWN OBSTACLE detected.")

        self.set_state("NORMAL_JOYSTICK_CONTROL")

        rate = rospy.Rate(10)

        while (
            self.current_distance < self.obstacle_threshold
            and not rospy.is_shutdown()
        ):
            self.stop_robot(0.1)
            rate.sleep()

        rospy.loginfo("Obstacle removed.")

        self.set_state("LANE_FOLLOWING")

        self.start_cooldown(2.0)

        self.busy = False


if __name__ == "__main__":
    try:
        Autopilot()
    except rospy.ROSInterruptException:
        pass