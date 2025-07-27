#!/bin/bash
# 开机自启动脚本,不要随便修改!
bash /home/bupt/fly_ws/start360.sh

# mavros
tmux new-session -d -s mavros_session
tmux send-keys -t mavros_session:0 'roslaunch mavros px4.launch' C-m

