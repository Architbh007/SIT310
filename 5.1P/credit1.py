#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import AprilTagDetectionArray

class Target_Follower:
    def __init__(self):
        
        rospy.init_node('target_follower_node', anonymous=True)
        rospy.on_shutdown(self.clean_shutdown)

        self.cmd_vel_pub = rospy.Publisher('/mybot002437/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/mybot002437/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)

        #Rotation PID gains
        self.KP_omega = 2.0
        self.KI_omega = 0.0
        self.KD_omega = 0.1

        #Linear PID gains
        self.KP_v     = 0.5
        self.KI_v     = 0.0
        self.KD_v     = 0.1

        # --- Tunable parameters ---
        self.GOAL_DISTANCE  = 0.4
        self.OMEGA_MIN      = 0.3
        self.OMEGA_MAX      = 4.0
        self.V_MAX          = 0.3
        self.CENTER_THRESH  = 0.05
        self.DIST_THRESH    = 0.05

        # PID state variables
        self.prev_error_omega = 0.0
        self.integral_omega   = 0.0
        self.prev_error_v     = 0.0
        self.integral_v       = 0.0
        self.prev_time        = rospy.Time.now()

        rospy.spin()

    def tag_callback(self, msg):
        self.move_robot(msg.detections)

    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v     = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def publish_velocity(self, v, omega):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v     = v
        cmd_msg.omega = omega
        self.cmd_vel_pub.publish(cmd_msg)

    def compute_pid(self, error, prev_error, integral, kp, ki, kd, dt):
        P = kp * error
        integral += error * dt
        I = ki * integral
        D = kd * (error - prev_error) / dt if dt > 0 else 0.0
        output = P + I + D
        return output, error, integral

    def move_robot(self, detections):

        # No tag: stay still and reset PID
        if len(detections) == 0:
            rospy.loginfo("[WAITING] No tag detected - staying still.")
            self.stop_robot()
            self.prev_error_omega = 0.0
            self.integral_omega   = 0.0
            self.prev_error_v     = 0.0
            self.integral_v       = 0.0
            self.prev_time        = rospy.Time.now()
            return

        # Time delta for PID calculations
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()
        self.prev_time = current_time

        if dt <= 0:
            return

        x = detections[0].transform.translation.x
        z = detections[0].transform.translation.z

        rospy.loginfo("x=%.3f  z=%.3f", x, z)

        #Rotation PID to align with tag
        if abs(x) < self.CENTER_THRESH:
            omega = 0.0
            self.prev_error_omega = 0.0
            self.integral_omega   = 0.0
            rospy.loginfo("[ROTATION] Tag centred.")
        else:
            omega_error = -x
            omega, self.prev_error_omega, self.integral_omega = self.compute_pid(
                omega_error,
                self.prev_error_omega,
                self.integral_omega,
                self.KP_omega, self.KI_omega, self.KD_omega,
                dt
            )
            if 0 < omega < self.OMEGA_MIN:
                omega = self.OMEGA_MIN
            elif -self.OMEGA_MIN < omega < 0:
                omega = -self.OMEGA_MIN
            omega = max(-self.OMEGA_MAX, min(self.OMEGA_MAX, omega))
            direction = "right" if omega < 0 else "left"
            rospy.loginfo("[ROTATION] x=%.3f → turning %s (omega=%.3f)", x, direction, omega)

        #Linear PID to maintain distance from tag
        if abs(z - self.GOAL_DISTANCE) < self.DIST_THRESH:
            v = 0.0
            self.prev_error_v = 0.0
            self.integral_v   = 0.0
            rospy.loginfo("[LINEAR] At goal distance.")
        else:
            dist_error = z - self.GOAL_DISTANCE
            v, self.prev_error_v, self.integral_v = self.compute_pid(
                dist_error,
                self.prev_error_v,
                self.integral_v,
                self.KP_v, self.KI_v, self.KD_v,
                dt
            )
            v = max(-self.V_MAX, min(self.V_MAX, v))
            direction = "forward" if v > 0 else "backward"
            rospy.loginfo("[LINEAR] z=%.3f → moving %s (v=%.3f)", z, direction, v)

        self.publish_velocity(v=v, omega=omega)


if __name__ == '__main__':
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass