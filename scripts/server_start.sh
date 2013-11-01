tmux start-server
tmux new-session -d -s webteleop -n roscore
tmux new-window -t webteleop:1 -n pr2_gazebo
tmux new-window -t webteleop:2 -n rosbridge
tmux new-window -t webteleop:3 -n mjpeg_server
tmux new-window -t webteleop:4 -n tf2_web_republisher
tmux new-window -t webteleop:5 -n openni
tmux new-window -t webteleop:6 -n ros_web_video
tmux new-window -t webteleop:7 -n depthcloud_encoder

tmux send-keys -t webteleop:0 "roscore" C-m
sleep 1
tmux send-keys -t webteleop:1 "roslaunch pr2_gazebo pr2_table_object.launch" C-m
tmux send-keys -t webteleop:2 "roslaunch rosbridge_server rosbridge_websocket.launch" C-m
tmux send-keys -t webteleop:3 "rosrun mjpeg_server mjpeg_server" C-m
tmux send-keys -t webteleop:4 "rosrun tf2_web_republisher tf2_web_republisher" C-m
tmux send-keys -t webteleop:5 "roslaunch openni_launch openni.launch depth_registration:=true" C-m
tmux send-keys -t webteleop:6 "rosrun ros_web_video ros_web_video _port:=9999 _framerate:=15 _bitrate:=250000 _profile:=best _www_file_server:=true _wwwroot:=./" C-m
tmux send-keys -t webteleop:7 "rosrun depthcloud_encoder depthcloud_encoder_node _depth:=/head_mount_kinect_rgb/depth/image_raw _rgb:=/head_mount_kinect/rgb/image_raw" C-m
