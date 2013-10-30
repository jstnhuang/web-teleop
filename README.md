# web-teleop

Web teleop interface for a PR2 robot, based on [Robot Web Tools](robotwebtools.org).

To bring up the server in a simulator:
1. `roscore`
2. `roslaunch pr2_gazebo pr2_table_object.launch`
3. `roslaunch rosbridge_server rosbridge_websocket.launch`
4. `rosrun mjpeg_server mjpeg_server`
