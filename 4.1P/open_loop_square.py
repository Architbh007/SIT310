#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState

class Drive_Square:
    def __init__(self):
        #Initialize global class variables
        self.cmd_msg = Twist2DStamped()

        #Initialize ROS node
        rospy.init_node('drive_square_node', anonymous=True)

        #Initialize Pub/Subs
        self.pub = rospy.Publisher('/mybot002437/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/mybot002437/fsm_node/mode', FSMState, self.handle_fsm_state, queue_size=1)

        #Movement parameters 
        self.forward_speed = 0.22   # linear velocity for straight segments 
        self.forward_omega = 0.0    # angular correction for straight driving (0 = no drift correction)
        self.forward_time  = 4.8    # duration to drive forward 

        self.turn_speed = 2.2       # angular velocity for 90-degree turns (rad/s)
        self.turn_time  = 1.0       # duration to complete 90-degree turn (seconds)

    def fsm_callback(self, msg):
        rospy.loginfo("State: %s", msg.state)
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.halt_robot()
        elif msg.state == "LANE_FOLLOWING":
            rospy.sleep(1) # Wait for a sec for the node to be ready
            self.execute_square()

    def send_velocity(self, v, omega, duration):
        # Continuously publish a velocity command for a set duration
        start_time = rospy.Time.now().to_sec()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if rospy.Time.now().to_sec() - start_time >= duration:
                break
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = v
            self.cmd_msg.omega = omega
            self.pub.publish(self.cmd_msg)
            rate.sleep()

    def halt_robot(self):
        # Send several zero-velocity commands to ensure the robot fully stops
        for _ in range(10):
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.0
            self.cmd_msg.omega = 0.0
            self.pub.publish(self.cmd_msg)
            rospy.Rate(10).sleep()

    def drive_straight(self):
        # Drive forward for the configured duration 
        rospy.loginfo("Driving straight...")
        self.send_velocity(
            v=self.forward_speed,
            omega=self.forward_omega,
            duration=self.forward_time
        )
        self.halt_robot()
        rospy.sleep(1)  # brief pause before the next action

    def execute_turn(self):
        # Rotate in place for the configured duration
        rospy.loginfo("Executing 90-degree turn...")
        self.send_velocity(
            v=0.0,
            omega=self.turn_speed,
            duration=self.turn_time
        )
        self.halt_robot()
        rospy.sleep(1)  # brief pause to let the robot settle before moving again

    def execute_square(self):
        # Drive four sides of a square - one straight segment and one turn per side
        rospy.sleep(1)  # wait for the publisher connection to establish
        for i in range(4):
            rospy.loginfo("Side %d of 4", i + 1)
            self.drive_straight()
            self.execute_turn()
        self.halt_robot()
        rospy.loginfo("Square complete. Robot stopped.")

    def run(self):
        # Keep the node alive and listening for state callbacks
        rospy.spin()

if __name__ == '__main__':
    try:
        duckiebot_movement = Drive_Square()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass