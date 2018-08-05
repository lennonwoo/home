# 创建名为my-session的会话
tmux new-window -c ~/home_ws/ 'source devel/setup.zsh && roslaunch home obj_detect.launch'
sleep 3
tmux split-window -h -c ~/home_ws/ 'source devel/setup.zsh && roslaunch home navigation.launch'
sleep 2
tmux split-window -v -c ~/home_ws/ 'source devel/setup.zsh && roslaunch home xf.launch'
sleep 2
tmux select-pane -L
tmux split-window -v -c ~/home_ws/ 'source devel/setup.zsh && roslaunch home home_sm.launch'
