#!/bin/zsh
set -e

# 创建会话和第一个窗口。
# 防重复：先删同名旧会话再建。2>/dev/null 只藏报错，|| true 才是关键——
# set -e 下没有 || true，session 不存在时 kill 返回非 0 会让脚本直接退出。
tmux kill-session -t ros_session 2>/dev/null || true
tmux new-session -d -s ros_session -n main_nodes zsh
tmux set-window-option -t ros_session:main_nodes pane-base-index 0

# Pane 0：ROS 主服务。
tmux send-keys -t ros_session:main_nodes.0 'roscore' C-m

# Pane 1：飞控连接、雷达和定位。
# 【实机按需改】fly_demo/utils.launch 是模板占位；换成你们飞机实际的
# 飞控驱动 + 定位启动命令（要求发布 /mavros/state 和 /mavros/local_position/odom）。
tmux split-window -h -t ros_session:main_nodes.0 zsh
tmux send-keys -t ros_session:main_nodes.1 'sleep 3; roslaunch fly_demo utils.launch' C-m

# Pane 2：heart 任务，程序提示后由操作者输入 1，遥控器切 OFFBOARD 后起飞。
# 环境按本仓库 clone 后的目录 ~/airhust 写；若 clone 到别处，改这个路径。
tmux split-window -h -t ros_session:main_nodes.1 zsh
tmux send-keys -t ros_session:main_nodes.2 'sleep 8; source ~/airhust/devel/setup.zsh --extend; roslaunch mission heart.launch' C-m

# 整理分屏并打开会话。
tmux select-layout -t ros_session:main_nodes tiled
tmux attach-session -t ros_session
