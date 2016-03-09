#!/bin/sh
SESSION="pandubot"
tmux new-session -d -s $SESSION
tmux rename-window -t $SESSION:1 ROS 
tmux split-window -h
tmux select-pane -t 1
tmux resize-pane -R 30
tmux send-keys -t 1 C-z 'source devel/setup.bash' Enter
tmux attach -t $SESSION
