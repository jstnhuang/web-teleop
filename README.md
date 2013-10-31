# web-teleop

Web teleop interface for a PR2 robot, based on [Robot Web Tools](robotwebtools.org).

To bring up the server in a simulator:

1. `roscore`
2. `roslaunch pr2_gazebo pr2_table_object.launch`
3. `roslaunch rosbridge_server rosbridge_websocket.launch`
4. `rosrun mjpeg_server mjpeg_server`

To bring up the depth cloud:

1. `rosrun tf2_web_republisher tf2_web_republisher`
2. `roslaunch openni_launch openni.launch depth_registration:=true`
3. `rosrun ros_web_video ros_web_video _port:=9999 _framerate:=15 _bitrate:=250000 _profile:=best _www_file_server:=true _wwwroot:=/path/to/wwwroot/`
4. `rosrun depthcloud_encoder depthcloud_encoder_node _depth:=/head_mount_kinect_rgb/depth/image_raw _rgb:=/head_mount_kinect/rgb/image_raw`

To bring up the PR2 URDF:
 
1. `roslaunch pr2_description upload_pr2.launch`
2. `rosrun robot_state_publisher robot_state_publisher`
3. `rosparam set use_gui true`
4. `rosrun joint_state_publisher joint_state_publisher`

(Unfinished) To bring up the interactive markers:

1. `roslaunch ik_sample ik_servers.launch`
2. `rosrun ik_sample gripper_markers.py`
1. `rosrun interactive_marker_proxy proxy pic_ns:=/ik_request_markers_l target_frame:=/base_link`
