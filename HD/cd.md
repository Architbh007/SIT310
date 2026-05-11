Terminal 1 — Launch Autopilot Stack
bashdts duckiebot demo --demo_name indefinite_navigation --duckiebot_name mybot002437 --package_name duckietown_demos
Just leave this running, don't close it.

Terminal 2 — Run your node
bashsource /code/catkin_ws/devel/setup.bash
rosrun autopilot_task intersection_handler_node.py _robot_name:=mybot002437
Leave this running too, watch it for logs.

Terminal 3 — Trigger lane following
bashrostopic pub /mybot002437/fsm_node/mode duckietown_msgs/FSMState "state: LANE_FOLLOWING"
This one is a one-off command, just run it once and your bot should start moving.

TL;DR
TerminalWhat it doesKeep open?1Runs the full autopilot stack (lane follow + apriltag)✅ Yes2Your intersection handler, watches for sign detections✅ Yes3Kicks off lane following modeJust run once