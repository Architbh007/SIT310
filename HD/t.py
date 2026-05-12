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

        # Sign IDs
        self.STOP_ID = 24
        self.RIGHT_ID = 9
        self.LEFT_ID = 11

        # Distance trigger
        self.TRIGGER_DISTANCE = 0.20  # 20 cm

        # Tick calibration
        self.TURN_90_TICKS = 48

        # State variables
        self.pending_action = None
        self.tof_distance = None
        self.stop_tag_visible = False
        self.action_running = False

        # Encoder values
        self.left_ticks = 0
        self.right_ticks = 0

        # Publishers
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

        # Subscribers
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

        rospy.sleep(1.0)

        # Start in lane following
        self.set_state("LANE_FOLLOWING")
        rospy.loginfo("Autopilot started in LANE_FOLLOWING mode")

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

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)
        rospy.sleep(0.3)

    def left_encoder_callback(self, msg):
        self.left_ticks = msg.data

    def right_encoder_callback(self, msg):
        self.right_ticks = msg.data

    def tof_callback(self, msg):
        self.tof_distance = msg.range

        if self.action_running:
            return

        if self.pending_action is None:
            return

        if self.tof_distance <= self.TRIGGER_DISTANCE:
            rospy.loginfo(f"ToF triggered at {self.tof_distance:.2f} m")
            self.execute_pending_action()

    def tag_callback(self, msg):
        detected_ids = []

        for detection in msg.detections:
            tag_id = detection.tag_id
            detected_ids.append(tag_id)

        self.stop_tag_visible = self.STOP_ID in detected_ids

        if self.action_running:
            return

        if self.pending_action is not None:
            return

        if self.STOP_ID in detected_ids:
            self.pending_action = "STOP"
            rospy.loginfo("STOP sign detected. Pending action stored.")

        elif self.LEFT_ID in detected_ids:
            self.pending_action = "LEFT"
            rospy.loginfo("LEFT sign detected. Pending action stored.")

        elif self.RIGHT_ID in detected_ids:
            self.pending_action = "RIGHT"
            rospy.loginfo("RIGHT sign detected. Pending action stored.")

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
        self.set_state("NORMAL_JOYSTICK_CONTROL")
        self.stop_robot()

        rospy.loginfo("STOP action complete. Waiting for STOP tag to disappear...")

        rate = rospy.Rate(10)

        while not rospy.is_shutdown() and self.stop_tag_visible:
            self.stop_robot()
            rate.sleep()

        rospy.loginfo("STOP tag disappeared. Resuming lane following.")
        self.set_state("LANE_FOLLOWING")

    def turn_left_90_ticks(self):
        self.set_state("NORMAL_JOYSTICK_CONTROL")
        self.stop_robot()

        start_left = self.left_ticks
        start_right = self.right_ticks

        rate = rospy.Rate(20)

        rospy.loginfo("Starting LEFT 90-degree tick turn")

        while not rospy.is_shutdown():
            left_delta = abs(self.left_ticks - start_left)
            right_delta = abs(self.right_ticks - start_right)

            avg_ticks = (left_delta + right_delta) / 2.0

            if avg_ticks >= self.TURN_90_TICKS:
                break

            self.publish_cmd(0.0, 3.0)
            rate.sleep()

        self.stop_robot()
        rospy.loginfo("LEFT turn complete")
        self.set_state("LANE_FOLLOWING")

    def turn_right_90_ticks(self):
        self.set_state("NORMAL_JOYSTICK_CONTROL")
        self.stop_robot()

        start_left = self.left_ticks
        start_right = self.right_ticks

        rate = rospy.Rate(20)

        rospy.loginfo("Starting RIGHT 90-degree tick turn")

        while not rospy.is_shutdown():
            left_delta = abs(self.left_ticks - start_left)
            right_delta = abs(self.right_ticks - start_right)

            avg_ticks = (left_delta + right_delta) / 2.0

            if avg_ticks >= self.TURN_90_TICKS:
                break

            self.publish_cmd(0.0, -3.0)
            rate.sleep()

        self.stop_robot()
        rospy.loginfo("RIGHT turn complete")
        self.set_state("LANE_FOLLOWING")

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = AutopilotNode()
    node.run()