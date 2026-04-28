#!/usr/bin/env python3
import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import WheelEncoderStamped
from sensor_msgs.msg import Range

class ClosedLoopSquare:
    def __init__(self):
        rospy.init_node('closed_loop_square', anonymous=True)

        self.pub = rospy.Publisher(
            '/mybota002437/car_cmd_switch_node/cmd',
            Twist2DStamped,
            queue_size=1
        )

        # Encoder values
        self.left_ticks = 0
        self.right_ticks = 0
        self.left_start = 0
        self.right_start = 0

        # ToF sensor
        self.tof_distance = float('inf')
        self.STOP_THRESHOLD = 0.30  # metres

        # Tuned parameters
        self.forward_speed = 0.22
        self.turn_speed = 2.2
        self.ticks_per_metre = 368
        self.ticks_per_90deg = 48

        self.rate = rospy.Rate(10)
        self.msg = Twist2DStamped()

        # Subscribers
        rospy.Subscriber(
            '/mybota002437/left_wheel_encoder_node/tick',
            WheelEncoderStamped,
            self.left_encoder_callback,
            queue_size=1
        )
        rospy.Subscriber(
            '/mybota002437/right_wheel_encoder_node/tick',
            WheelEncoderStamped,
            self.right_encoder_callback,
            queue_size=1
        )
        rospy.Subscriber(
            '/mybota002437/front_center_tof_driver_node/range',
            Range,
            self.tof_callback,
            queue_size=1
        )

    def tof_callback(self, msg):
        self.tof_distance = msg.range

    def obstacle_detected(self):
        return self.tof_distance < self.STOP_THRESHOLD

    def left_encoder_callback(self, msg):
        self.left_ticks = msg.data

    def right_encoder_callback(self, msg):
        self.right_ticks = msg.data

    def avg_ticks(self):
        left_diff = abs(self.left_ticks - self.left_start)
        right_diff = abs(self.right_ticks - self.right_start)
        return (left_diff + right_diff) / 2.0

    def reset_ticks(self):
        self.left_start = self.left_ticks
        self.right_start = self.right_ticks

    def stop_robot(self):
        for _ in range(10):
            self.msg.header.stamp = rospy.Time.now()
            self.msg.v = 0.0
            self.msg.omega = 0.0
            self.pub.publish(self.msg)
            self.rate.sleep()

    def wait_for_clear(self):
        rospy.loginfo("Obstacle detected at %.2fm! Waiting...", self.tof_distance)
        while not rospy.is_shutdown() and self.obstacle_detected():
            self.msg.header.stamp = rospy.Time.now()
            self.msg.v = 0.0
            self.msg.omega = 0.0
            self.pub.publish(self.msg)
            self.rate.sleep()
        rospy.loginfo("Obstacle cleared. Resuming in 0.5s...")
        rospy.sleep(0.5)

    def move_straight(self, distance, speed):
        tick_target = int((distance / 1.0) * self.ticks_per_metre)
        self.reset_ticks()
        rospy.loginfo("Moving straight %sm (%d ticks)...", distance, tick_target)

        while not rospy.is_shutdown() and self.avg_ticks() < tick_target:
            if self.obstacle_detected():
                ticks_done = self.avg_ticks()
                self.stop_robot()
                self.wait_for_clear()
                self.reset_ticks()
                tick_target = tick_target - int(ticks_done)

            self.msg.header.stamp = rospy.Time.now()
            self.msg.v = speed if distance > 0 else -speed
            self.msg.omega = -0.010
            self.pub.publish(self.msg)
            self.rate.sleep()

        self.stop_robot()
        rospy.sleep(1)

    def rotate(self, angle_deg, speed):
        # No collision prevention during rotation
        tick_target = int((angle_deg / 90.0) * self.ticks_per_90deg)
        self.reset_ticks()
        rospy.loginfo("Rotating %d degrees (%d ticks)...", angle_deg, tick_target)
        self.msg.header.stamp = rospy.Time.now()
        self.msg.v = 0.0
        self.msg.omega = speed if angle_deg > 0 else -speed
        self.pub.publish(self.msg)
        while not rospy.is_shutdown() and self.avg_ticks() < tick_target:
            self.msg.header.stamp = rospy.Time.now()
            self.pub.publish(self.msg)
            self.rate.sleep()
        self.stop_robot()
        rospy.sleep(1)

    def drive_square(self):
        rospy.sleep(2)
        for i in range(4):
            rospy.loginfo("Starting side %d of square", i + 1)
            self.move_straight(1.0, self.forward_speed)
            self.rotate(90, self.turn_speed)
        self.stop_robot()
        rospy.loginfo("Square complete!")

if __name__ == '__main__':
    try:
        driver = ClosedLoopSquare()
        driver.drive_square()
    except rospy.ROSInterruptException:
        pass