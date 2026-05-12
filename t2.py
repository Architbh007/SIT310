#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Range
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped
from duckietown_msgs.msg import AprilTagDetectionArray


class AutopilotNode:
    def __init__(self):
        rospy.init_node("autopilot_node", anonymous=True)

        self.veh = rospy.get_param("~veh", "mybota002437")

        self.STOP_ID = 26
        self.RIGHT_ID = 9
        self.LEFT_ID = 10

        self.TRIGGER_DISTANCE = 0.25
        self.TURN_90_TICKS = 48

        self.pending_action = None
        self.tof_distance = 999.0
        self.stop_tag_visible = False
        self.action_running = False

        self.left_ticks = 0
        self.right_ticks = 0

        self.cmd_pub = rospy.Publisher(
            f"/{self.veh}/car_cmd_switch_node/cmd",
            Twist2DStamped,
            queue_size=1
        )

        self.fsm_pub = rospy.Publisher(
            f"/{self.veh}/fsm_node/mode",
            String,
            queue_size=1
        )

        rospy.Subscriber(
            f"/{self.veh}/apriltag_detector_node/detections",
            AprilTagDetectionArray,
            self.tag_callback
        )

        rospy.Subscriber(
            f"/{self.veh}/front_center_tof_driver_node/range",
            Range,
            self.tof_callback
        )

        rospy.Subscriber(
            f"/{self.veh}/left_wheel_encoder_node/tick",
            WheelEncoderStamped,
            self.left_encoder_callback
        )

        rospy.Subscriber(
            f"/{self.veh}/right_wheel_encoder_node/tick",
            WheelEncoderStamped,
            self.right_encoder_callback
        )

        rospy.loginfo("Autopilot ready. Start lane following externally using CLI.")

    def set_state(self, state):
        msg = String()
        msg.data = state
        self.fsm_pub.publish(msg)
        rospy.loginfo(f"FSM set to: {state}")

    def publish_cmd(self, v, omega):
        msg = Twist2DStamped()
        msg.header.stamp = rospy.Time.now()
        msg.v = v
        msg.omega = omega
        self.cmd_pub.publish(msg)

    def force_stop_for(self, seconds):
        rate = rospy.Rate(30)
        end_time = rospy.Time.now() + rospy.Duration(seconds)

        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            self.publish_cmd(0.0, 0.0)
            rate.sleep()

    def stop_robot(self):
        self.force_stop_for(1.0)

    def left_encoder_callback(self, msg):
        self.left_ticks = msg.data

    def right_encoder_callback(self, msg):
        self.right_ticks = msg.data

    def tag_callback(self, msg):
        detected_ids = []

        for detection in msg.detections:
            tag_id = detection.tag_id
            detected_ids.append(tag_id)

        self.stop_tag_visible = self.STOP_ID in detected_ids

        if self.action_running:
            return

        # Only STORE action here. Do NOT execute here.
        if self.pending_action is None:
            if self.STOP_ID in detected_ids:
                self.pending_action = "STOP"
                rospy.loginfo("STOP detected. Stored pending action. Waiting for ToF <= 0.25m.")

            elif self.LEFT_ID in detected_ids:
                self.pending_action = "LEFT"
                rospy.loginfo("LEFT detected. Stored pending action. Waiting for ToF <= 0.25m.")

            elif self.RIGHT_ID in detected_ids:
                self.pending_action = "RIGHT"
                rospy.loginfo("RIGHT detected. Stored pending action. Waiting for ToF <= 0.25m.")

    def tof_callback(self, msg):
        self.tof_distance = msg.range

        if self.action_running:
            return

        if self.pending_action is None:
            return

        # Only execute when close enough
        if self.tof_distance <= self.TRIGGER_DISTANCE:
            rospy.loginfo(
                f"ToF distance {self.tof_distance:.2f}m <= {self.TRIGGER_DISTANCE:.2f}m. Executing action."
            )
            self.execute_pending_action()

    def execute_pending_action(self):
        self.action_running = True

        action = self.pending_action
        self.pending_action = None

        rospy.loginfo(f"Executing action: {action}")

        if action == "STOP":
            self.handle_stop()

        elif action == "LEFT":
            self.turn_left_90_ticks()

        elif action == "RIGHT":
            self.turn_right_90_ticks()

        self.action_running = False

    def handle_stop(self):
        rospy.loginfo("STOP triggered at 0.25m. Taking manual control and stopping.")

        self.set_state("NORMAL_JOYSTICK_CONTROL")
        rospy.sleep(0.5)

        # Strong stop to cancel lane-follow movement
        self.force_stop_for(2.0)

        rospy.loginfo("Robot stopped. Holding while STOP tag is visible.")

        rate = rospy.Rate(20)

        while not rospy.is_shutdown() and self.stop_tag_visible:
            self.publish_cmd(0.0, 0.0)
            rate.sleep()

        # Extra stop after tag disappears
        self.force_stop_for(1.0)

        rospy.loginfo("STOP tag disappeared. Action complete. Resume lane following externally.")

    def turn_left_90_ticks(self):
        self.set_state("NORMAL_JOYSTICK_CONTROL")
        rospy.sleep(0.5)

        self.stop_robot()

        start_left = self.left_ticks
        start_right = self.right_ticks

        rate = rospy.Rate(20)

        rospy.loginfo("Starting LEFT 90-degree tick turn")

        while not rospy.is_shutdown():
            left_delta = abs(self.left_ticks - start_left)
            right_delta = abs(self.right_ticks - start_right)
            avg_ticks = (left_delta + right_delta) / 2.0

            rospy.loginfo_throttle(
                0.5,
                f"LEFT turning ticks: {avg_ticks}/{self.TURN_90_TICKS}"
            )

            if avg_ticks >= self.TURN_90_TICKS:
                break

            self.publish_cmd(0.0, 3.0)
            rate.sleep()

        self.stop_robot()
        rospy.loginfo("LEFT turn complete. Resume lane following externally.")

    def turn_right_90_ticks(self):
        self.set_state("NORMAL_JOYSTICK_CONTROL")
        rospy.sleep(0.5)

        self.stop_robot()

        start_left = self.left_ticks
        start_right = self.right_ticks

        rate = rospy.Rate(20)

        rospy.loginfo("Starting RIGHT 90-degree tick turn")

        while not rospy.is_shutdown():
            left_delta = abs(self.left_ticks - start_left)
            right_delta = abs(self.right_ticks - start_right)
            avg_ticks = (left_delta + right_delta) / 2.0

            rospy.loginfo_throttle(
                0.5,
                f"RIGHT turning ticks: {avg_ticks}/{self.TURN_90_TICKS}"
            )

            if avg_ticks >= self.TURN_90_TICKS:
                break

            self.publish_cmd(0.0, -3.0)
            rate.sleep()

        self.stop_robot()
        rospy.loginfo("RIGHT turn complete. Resume lane following externally.")

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = AutopilotNode()
    node.run()