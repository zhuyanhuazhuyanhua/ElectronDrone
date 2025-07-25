#!/bin/bash

# 设置 session 名字
SESSION_NAME="t24"

# 创建新的 tmux session，后台运行，不附着
tmux new-session -d -s $SESSION_NAME

# livox驱动
tmux send-keys -t $SESSION_NAME:0 'roslaunch mavros px4.launch ' C-m

echo "1"

sleep 1.0

bash start360.sh

echo "2
"

# 附着到这个session
# tmux attach-session -t $SESSION_NAME
