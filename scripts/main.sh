#!/bin/bash

TERMINAL="gnome-terminal"

mid360="./mid360.sh"

lio="./lio.sh"

division="./division.sh"

pcd2scan="./pcd2scan.sh"


source_cmd="source install/setup.bash"


eval $source_cmd
echo "✅ ROS2环境已加载"

$TERMINAL --title "mid360" -e "bash -c '$source_cmd && $mid360; read -p \"按回车关闭窗口...\"'" &
pid_mid360=$!
echo "✅ 建图终端已启动,PID: $pid_mid360"

sleep 3

$TERMINAL --title "lio" -e "bash -c '$source_cmd && $lio; read -p \"按回车关闭窗口...\"'" &
pid_lio=$!
echo "✅ 建图终端已启动,PID: $pid_lio"

sleep 3

$TERMINAL --title "点云分割" -e "bash -c '$source_cmd && $division; read -p \"按回车关闭窗口...\"'" &
pid_division=$!
echo "✅ 建图终端已启动,PID: $pid_division"

sleep 3


$TERMINAL --title "pcd2scan" -e "bash -c '$source_cmd && $pcd2scan; read -p \"按回车关闭窗口...\"'" &
pid_pcd2scan=$!
echo "✅ 建图终端已启动,PID: $pid_pcd2scan"


# 等待所有后台进程（可选：若需主脚本不退出，保留此句）
wait $pid_mid360 $pid_lio  $pid_division $pid_pcd2scan
echo "所有ROS2节点已退出,脚本结束"