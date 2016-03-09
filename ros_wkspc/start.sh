#!/bin/sh
tmux new-session -d -s "dev"
tmux send-keys -t "dev" 'cd ~/pandubot_wkspc/ros_wkspc/src' Enter
tmux new-session -d -s "ROS"
tmux send-keys -t "ROS"  'cd ~/pandubot_wkspc/ros_wkspc' Enter
tmux new-session -d -s "codeRef"
tmux new-session -d -s "blog"
tmux send-keys -t "blog" 'cd ~/career/website/blog' Enter
tmux attach -t "dev"

