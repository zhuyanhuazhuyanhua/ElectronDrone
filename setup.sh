#!/bin/bash
# 开机自启动脚本,不要随便修改!

# mavros
tmux new-session -d -s mavros_session
tmux send-keys -t mavros_session:0 'roslaunch mavros px4.launch' C-m

sleep 1
bash /home/bupt/fly_ws/start360.sh

sleep 1

tmux new-session -d -s offboard
tmux send-keys -t offboard:0 'source /home/bupt/fly_ws/devel/setup.bash && \
 roslaunch offboard_run run_offboard.launch' C-m