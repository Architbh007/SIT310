docker -H mybot002437.local exec demo_lane_following rostopic pub /mybot002437/wheels_driver_node/wheels_cmd duckietown_msgs/WheelsCmdStamped "vel_left: 0.3
vel_right: 0.3"