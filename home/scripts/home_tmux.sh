# 创建名为my-session的会话
tmux new-window -c
tmux split-window -h
tmux split-window -v
tmux select-pane -L
tmux split-window -v

tmux send-keys -t 0.1 'cd ~/home_ws && source devel/setup.zsh && roslaunch home obj_detect.launch' Enter
sleep 3
tmux send-keys -t 0.2 'cd ~/home_ws && source devel/setup.zsh && roslaunch home navigation.launch' Enter
sleep 2
tmux send-keys -t 0.3 'cd ~/home_ws && source devel/setup.zsh && roslaunch home xf.launch' Enter
sleep 2
tmux send-keys -t 0.4 'cd ~/home_ws && source devel/setup.zsh && roslaunch home home_sm.launch' Enter
