#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import AprilTagDetectionArray


class Autopilot:
    def __init__(self):
        rospy.init_node('autopilot_node', anonymous=True)

        rospy.loginfo("Autopilot node started!")

        self.robot_state = "LANE_FOLLOWING"
        self.ignore_tags = False

        self.veh = "mybota002437"

        rospy.on_shutdown(self.clean_shutdown)

        self.cmd_vel_pub = rospy.Publisher(
            f'/{self.veh}/car_cmd_switch_node/cmd',
            Twist2DStamped,
            queue_size=1
        )

        self.state_pub = rospy.Publisher(
            f'/{self.veh}/fsm_node/mode',
            FSMState,
            queue_size=1
        )

        rospy.Subscriber(
            f'/{self.veh}/apriltag_detector_node/detections',
            AprilTagDetectionArray,
            self.tag_callback,
            queue_size=1
        )

        rospy.loginfo("Autopilot ready. Listening for tags...")
        rospy.spin()

    def tag_callback(self, msg):
        if self.robot_state != "LANE_FOLLOWING":
            return

        if self.ignore_tags:
            return

        self.move_robot(msg.detections)

    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot(duration=1.0)

    def stop_robot(self, duration=0.5):
        cmd_msg = Twist2DStamped()
        rate = rospy.Rate(10)
        start = rospy.Time.now()

        while (rospy.Time.now() - start).to_sec() < duration and not rospy.is_shutdown():
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.v = 0.0
            cmd_msg.omega = 0.0
            self.cmd_vel_pub.publish(cmd_msg)
            rate.sleep()

    def set_state(self, state):
        rospy.loginfo(f"Switching FSM to {state}")
        self.robot_state = state

        state_msg = FSMState()
        state_msg.state = state

        rate = rospy.Rate(10)

        for _ in range(10):
            state_msg.header.stamp = rospy.Time.now()
            self.state_pub.publish(state_msg)
            rate.sleep()

    def publish_cmd(self, v, omega, duration):
        cmd_msg = Twist2DStamped()
        rate = rospy.Rate(10)
        start = rospy.Time.now()

        rospy.loginfo(f"Publishing command: v={v}, omega={omega}, duration={duration}")

        while (rospy.Time.now() - start).to_sec() < duration and not rospy.is_shutdown():
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.v = v
            cmd_msg.omega = omega
            self.cmd_vel_pub.publish(cmd_msg)
            rate.sleep()

    def start_cooldown(self, seconds):
        self.ignore_tags = True
        rospy.Timer(rospy.Duration(seconds), self.end_cooldown, oneshot=True)

    def end_cooldown(self, event):
        self.ignore_tags = False
        rospy.loginfo("Cooldown done. Watching for tags again.")

    def do_stop(self):
        rospy.loginfo("STOP SIGN detected — stopping now.")

        self.set_state("NORMAL_JOYSTICK_CONTROL")
        rospy.sleep(0.5)

        self.stop_robot(duration=3.0)

        rospy.loginfo("Stop complete. Moving forward slightly.")
        self.publish_cmd(0.12, 0.0, 1.0)

        self.stop_robot(duration=0.5)

        self.set_state("LANE_FOLLOWING")
        self.start_cooldown(5.0)

        rospy.loginfo("Resuming lane following after stop.")

    def do_left_turn(self):
        rospy.loginfo("LEFT TURN sign detected.")

        self.set_state("NORMAL_JOYSTICK_CONTROL")
        rospy.sleep(0.5)

        self.stop_robot(duration=0.5)

        self.publish_cmd(0.12, 0.0, 1.0)

        self.stop_robot(duration=0.3)

        self.publish_cmd(0.0, 4.0, 1.5)

        self.stop_robot(duration=0.3)

        self.publish_cmd(0.12, 0.0, 0.6)

        self.stop_robot(duration=0.5)

        self.set_state("LANE_FOLLOWING")
        self.start_cooldown(5.0)

        rospy.loginfo("LEFT TURN complete. Resuming lane following.")

    def do_right_turn(self):
        rospy.loginfo("RIGHT TURN sign detected.")

        self.set_state("NORMAL_JOYSTICK_CONTROL")
        rospy.sleep(0.5)

        self.stop_robot(duration=0.5)

        self.publish_cmd(0.12, 0.0, 1.0)

        self.stop_robot(duration=0.3)

        self.publish_cmd(0.0, -4.0, 1.5)

        self.stop_robot(duration=0.3)

        self.publish_cmd(0.12, 0.0, 0.6)

        self.stop_robot(duration=0.5)

        self.set_state("LANE_FOLLOWING")
        self.start_cooldown(5.0)

        rospy.loginfo("RIGHT TURN complete. Resuming lane following.")

    def move_robot(self, detections):
        if len(detections) == 0:
            return

        for detection in detections:
            tag_id = detection.tag_id

            if isinstance(tag_id, list):
                tag_id = tag_id[0]

            rospy.loginfo(f"Tag detected: ID={tag_id}")

            if tag_id == 26:
                self.do_stop()
                break

            elif tag_id == 10:
                self.do_left_turn()
                break

            elif tag_id == 9:
                self.do_right_turn()
                break


if __name__ == '__main__':
    try:
        autopilot_instance = Autopilot()
    except rospy.ROSInterruptException:
        pass
