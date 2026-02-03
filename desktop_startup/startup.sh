#!/usr/bin/bash

start() {
  /home/ubuntu/robot_ws/src/elsabot_robot/desktop_startup/run_robot.sh
}

stop() {
  echo "stopping"
}

case $1 in
  start|stop) "$1" ;;
esac

