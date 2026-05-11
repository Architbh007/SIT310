#!/usr/bin/env python3
import rospy
import time
from duckietown_msgs.msg import AprilTagDetectionArray, Twist2DStamped, FSMState, BoolStamped
from std_msgs.msg import String

class IntersectionHandlerNode:
    def __init__(self):
        self.node_name = rospy.get_name()
        self.robot_name = rospy.get_param("~robot_name", "mybot002437")

        # State
        self.state = "LANE_FOLLOWING"
        self.last_tag_time = 0
        self.cooldown = 5.0  # seconds between intersection responses

        # AprilTag IDs (Duckietown standard)
        self.TAG_STOP       = [163, 169]
        self.TAG_LEFT       = [57, 62]
        self.TAG_RIGHT      = [56, 61]
        self.TAG_OBSTACLE   = []  # handled separately via object detection

        # Publishers
        self.pub_car_cmd = rospy.Publisher(
            f"/{self.robot_name}/car_cmd_switch_node/cmd",
            Twist2DStamped, queue_size=1
        )
        self.pub_fsm = rospy.Publisher(
            f"/{self.robot_name}/fsm_node/mode",
            FSMState, queue_size=1
        )

        # Subscribers
        rospy.Subscriber(
            f"/{self.robot_name}/apriltag_detector_node/detections",
            AprilTagDetectionArray,
            self.cb_apriltag
        )

        rospy.loginfo(f"[{self.node_name}] Intersection handler ready.")

    def cb_apriltag(self, msg):
        now = time.time()
        if now - self.last_tag_time < self.cooldown:
            return  # still in cooldown, ignore

        for detection in msg.detections:
            tag_id = detection.tag_id

            if tag_id in self.TAG_STOP:
                rospy.loginfo(f"STOP SIGN detected (tag {tag_id})")
                self.do_stop()
                self.last_tag_time = now
                break

            elif tag_id in self.TAG_LEFT:
                rospy.loginfo(f"LEFT TURN detected (tag {tag_id})")
                self.do_turn(direction="left")
                self.last_tag_time = now
                break

            elif tag_id in self.TAG_RIGHT:
                rospy.loginfo(f"RIGHT TURN detected (tag {tag_id})")
                self.do_turn(direction="right")
                self.last_tag_time = now
                break

    def publish_cmd(self, v, omega, duration):
        """Send a velocity command for a fixed duration."""
        cmd = Twist2DStamped()
        cmd.v = v
        cmd.omega = omega
        rate = rospy.Rate(10)
        start = time.time()
        while time.time() - start < duration:
            cmd.header.stamp = rospy.Time.now()
            self.pub_car_cmd.publish(cmd)
            rate.sleep()

    def do_stop(self):
        rospy.loginfo("Executing: STOP")
        self.publish_cmd(0.0, 0.0, 2.0)   # stop for 2 seconds
        rospy.loginfo("Resuming lane following after stop")

    def do_turn(self, direction="left"):
        rospy.loginfo(f"Executing: {direction.upper()} TURN")
        # Approach intersection slowly
        self.publish_cmd(0.2, 0.0, 0.8)

        # Turn: left = positive omega, right = negative omega
        omega = 2.5 if direction == "left" else -2.5
        self.publish_cmd(0.15, omega, 1.2)

        # Straighten out
        self.publish_cmd(0.2, 0.0, 0.5)
        rospy.loginfo(f"{direction.upper()} TURN complete, resuming lane following")

    def on_shutdown(self):
        self.publish_cmd(0.0, 0.0, 0.1)

if __name__ == "__main__":
    rospy.init_node("intersection_handler_node", anonymous=False)
    node = IntersectionHandlerNode()
    rospy.on_shutdown(node.on_shutdown)
    rospy.spin()