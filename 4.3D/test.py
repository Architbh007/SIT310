#!/usr/bin/env python3
import rospy
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped
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

        # Distance per tick (1m / 368 ticks)
        self.distance_per_tick = 1.0 / 368.0

        # Robot geometry for rotation
        self.wheel_base = 0.1

        # Obstacle detection
        self.obstacle_detected = False
        self.stop_distance = 0.30  # metres

        # Tuned parameters
        self.forward_speed = 0.22
        self.turn_speed = 2.2
        self.ticks_per_90deg = 48

        self.rate = rospy.Rate(10)
        self.cmd = Twist2DStamped()

        # Subscribers
        rospy.Subscriber(
            '/mybota002437/left_wheel_encoder_node/tick',
            WheelEncoderStamped,
            self.left_cb,
            queue_size=1
        )
        rospy.Subscriber(
            '/mybota002437/right_wheel_encoder_node/tick',
            WheelEncoderStamped,
            self.right_cb,
            queue_size=1
        )
        rospy.Subscriber(
            '/mybota002437/front_center_tof_driver_node/range',
            Range,
            self.tof_cb,
            queue_size=1
        )

    def left_cb(self, msg):
        self.left_ticks = msg.data

    def right_cb(self, msg):
        self.right_ticks = msg.data

    def tof_cb(self, msg):
        if msg.range < self.stop_distance:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def stop(self):
        self.cmd.header.stamp = rospy.Time.now()
        self.cmd.v = 0.0
        self.cmd.omega = 0.0
        self.pub.publish(self.cmd)

    def move_straight(self, distance, speed):
        start = self.left_ticks
        rate = rospy.Rate(10)

        direction = 1 if distance >= 0 else -1
        distance = abs(distance)

        while not rospy.is_shutdown():
            delta = abs(self.left_ticks - start)
            travelled = delta * self.distance_per_tick

            if travelled >= distance:
                break

            if self.obstacle_detected:
                rospy.loginfo("Obstacle detected! Stopping...")
                self.stop()

                # Wait until obstacle is gone
                while self.obstacle_detected and not rospy.is_shutdown():
                    rospy.sleep(0.1)

                rospy.loginfo("Path clear. Resuming...")
                # Reset start so remaining distance is tracked correctly
                start = self.left_ticks
                distance = distance - travelled
                continue

            self.cmd.header.stamp = rospy.Time.now()
            self.cmd.v = direction * speed
            self.cmd.omega = -0.010
            self.pub.publish(self.cmd)
            rate.sleep()

        self.stop()
        rospy.loginfo("Straight done. Travelled: %.3fm", travelled)

    def rotate(self, angle_deg, speed):
        # No obstacle detection during rotation
        tick_target = int((angle_deg / 90.0) * self.ticks_per_90deg)
        start = self.left_ticks
        rate = rospy.Rate(10)

        direction = 1 if angle_deg >= 0 else -1

        while not rospy.is_shutdown():
            delta = abs(self.left_ticks - start)
            if delta >= tick_target:
                break
            self.cmd.header.stamp = rospy.Time.now()
            self.cmd.v = 0.0
            self.cmd.omega = direction * speed
            self.pub.publish(self.cmd)
            rate.sleep()

        self.stop()
        rospy.loginfo("Rotation done.")

    def drive_square(self):
        rospy.sleep(2)
        for i in range(4):
            rospy.loginfo("Starting side %d of square", i + 1)
            self.move_straight(1.0, self.forward_speed)
            rospy.sleep(1)
            self.rotate(90, self.turn_speed)
            rospy.sleep(1)
        self.stop()
        rospy.loginfo("Square complete!")

if __name__ == '__main__':
    try:
        driver = ClosedLoopSquare()
        driver.drive_square()
    except rospy.ROSInterruptException:
        pass