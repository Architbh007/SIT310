#!/usr/bin/env python3

# Import Dependencies
import rospy 
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import time 
import math

class TurtlesimStraightsAndTurns:
    def __init__(self):
        
        # Initialize class variables
        self.last_distance = 0
        self.goal_distance = 0
        self.dist_goal_active = False
        self.forward_movement = True

        # We need to remember where the turtle was when a goal started
        # so we can measure "how far have we gone since then"
        self.dist_start = 0.0

        # Same idea for angle goals store the heading we want to reach
        self.angle_goal_active = False
        self.target_angle = 0.0

        # For position goals we need the target x,y and which phase we're in
        # (first rotate to face the target, then drive straight to it)
        self.pos_goal_active = False
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.pos_phase = 'rotate'

        # We'll store the latest pose here so every callback can use it
        self.current_pose = None

        # Initialize the node
        rospy.init_node('turtlesim_straights_and_turns_node', anonymous=True)

        # Initialize subscribers  
        rospy.Subscriber("/turtle_dist", Float64, self.distance_callback)
        rospy.Subscriber("/goal_angle", Float64, self.goal_angle_callback)
        rospy.Subscriber("/goal_distance", Float64, self.goal_distance_callback)
        rospy.Subscriber("/turtle1/pose", Pose, self.pose_callback)
        # Position goal uses Point (x, y) instead of Float64 so we can send both coordinates
        rospy.Subscriber("/goal_position", Point, self.goal_position_callback)

        # Initialize publishers
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Initialize a timer. The timer callback will act as our main function
        timer_period = 0.01
        rospy.Timer(rospy.Duration(timer_period), self.timer_callback)

        # Printing to the terminal, ROS style
        rospy.loginfo("Initalized node!")
        
        # This blocking function call keeps python from exiting until node is stopped
        rospy.spin()

    def pose_callback(self, msg):
        # Just keep saving the latest pose so we always know where the turtle is
        self.current_pose = msg

    def distance_callback(self, msg):
        self.last_distance = msg.data

    def goal_angle_callback(self, msg):
        goal_angle = msg.data

        # If the goal is 0 there's nothing to do
        if goal_angle == 0.0:
            self.angle_goal_active = False
            return

        # We need the current heading to figure out where we want to end up
        if self.current_pose is None:
            return

        # Add the requested rotation to the current heading to get the target heading.
        # We then wrap it to [-pi, pi] because turtlesim's theta lives in that range
        # and flips abruptly at the ±pi boundary — normalize_angle handles that safely.
        self.target_angle = self._normalize_angle(self.current_pose.theta + goal_angle)
        self.angle_goal_active = True

    def goal_distance_callback(self, msg):
        ########## YOUR CODE GOES HERE ##########
        # Set goal_distance, dist_goal_active and forward_movement variables here

        goal = msg.data

        # If the goal is 0 the turtle shouldn't move at all
        if goal == 0.0:
            self.dist_goal_active = False
            return

        # Store how far we need to travel (always positive)
        self.goal_distance = abs(goal)

        # Positive goal means go forward, negative means go backward
        self.forward_movement = (goal > 0)

        # Take a snapshot of the turtle right now so the timer can measure
        # how far we've moved since this goal was received
        self.dist_start = self.last_distance

        # Tell the timer there's an active goal to work on
        self.dist_goal_active = True

        ###########################################

    def goal_position_callback(self, msg):
        # Can't do anything useful without knowing where we currently are
        if self.current_pose is None:
            return

        # Save the target coordinates
        self.goal_x = msg.x
        self.goal_y = msg.y

        # Always start by rotating to face the target before driving
        self.pos_phase = 'rotate'
        self.pos_goal_active = True

    def timer_callback(self, msg):
        ########## YOUR CODE GOES HERE ##########
        # If a goal is active, first check if the goal is reached 
        # Then publish a cmd_vel message

        # Don't do anything until we've received at least one pose 
        if self.current_pose is None:
            return

        # Every tick we build a fresh Twist message
        twist = Twist()

        #Distance goal
        if self.dist_goal_active:

            # How far have we travelled since this goal started?
            travelled = abs(self.last_distance - self.dist_start)

            #small tolerance because we can't hit it perfectly
            if travelled >= self.goal_distance - 0.05:
                # We're done — stop and clear the flag
                self.dist_goal_active = False
                rospy.loginfo("Distance goal reached.")
            else:
                # Still going - drive forward or backward at 1 unit/s
                twist.linear.x = 1.0 if self.forward_movement else -1.0

        #Angle goal
        elif self.angle_goal_active:

            # _angle_diff always gives us the shortest way around the circle,
            # so we never accidentally spin 350° when we only needed 10°.
            # This also handles the ±pi flip in turtlesim's theta correctly.
            remaining = self._angle_diff(self.target_angle, self.current_pose.theta)

            if abs(remaining) < 0.01:
                # Close enough to the target heading - stop rotating
                self.angle_goal_active = False
                rospy.loginfo("Angle goal reached.")
            else:
                # Spin proportionally - fast when far away, slow when nearly there
                # copysign makes sure we spin in the right direction
                speed = max(0.3, min(2.0, abs(remaining)))
                twist.angular.z = math.copysign(speed, remaining)

        #Position goal
        elif self.pos_goal_active:

            dx = self.goal_x - self.current_pose.x
            dy = self.goal_y - self.current_pose.y
            dist_to_goal = math.sqrt(dx**2 + dy**2)

            if dist_to_goal < 0.05:
                # We've arrived — stop
                self.pos_goal_active = False
                rospy.loginfo("Position goal reached.")

            else:
                # Figure out which direction we need to be facing
                target_heading = math.atan2(dy, dx)
                heading_error = self._angle_diff(target_heading, self.current_pose.theta)

                if self.pos_phase == 'rotate':
                    if abs(heading_error) < 0.01:
                        # Facing the right way — now we can start driving
                        self.pos_phase = 'drive'
                    else:
                        # Still turning — apply proportional angular speed
                        speed = max(0.3, min(2.0, abs(heading_error)))
                        twist.angular.z = math.copysign(speed, heading_error)

                elif self.pos_phase == 'drive':
                    # If we've drifted off course, go back to rotating to fix it
                    if abs(heading_error) > 0.05:
                        self.pos_phase = 'rotate'
                    else:
                        # Drive forward — faster when far, slower when close
                        twist.linear.x = max(0.3, min(2.0, dist_to_goal))

        # Send whatever velocity we decided on (or zeros to stop)
        self.velocity_publisher.publish(twist)

        ###########################################

    # Wraps any angle into [-pi, pi] so we stay consistent with turtlesim's range
    @staticmethod
    def _normalize_angle(angle):
        while angle >  math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

    # Returns the shortest signed arc from current to target - handles the ±pi jump
    @staticmethod
    def _angle_diff(target, current):
        diff = target - current
        while diff >  math.pi: diff -= 2 * math.pi
        while diff < -math.pi: diff += 2 * math.pi
        return diff


if __name__ == '__main__': 

    try: 
        turtlesim_straights_and_turns_class_instance = TurtlesimStraightsAndTurns()
    except rospy.ROSInterruptException: 
        pass
