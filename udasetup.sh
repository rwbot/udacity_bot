#!/usr/bin/env bash

# source /home/workspace/catkin_ws/src/udacity_bot/udasetup.sh

git config --global credential.helper 'cache --timeout=999999'
git config --global user.name 'rwbot'
git config --global user.email 'rwbotx@gmail.com'
alias gitcon="git config --global credential.helper 'cache --timeout=999999'; git config --global user.name 'rwbot'; git config --global user.email 'rwbotx@gmail.com'"

alias cdc="cd /home/workspace/catkin_ws" #custom cd command that goes to catkin_ws
alias cdcs="cd /home/workspace/catkin_ws/src" #custom cd command that goes to catkin_ws/SRC
alias cdcu="cdcs; cd udacity_bot"
alias cdcmk="cdc; catkin_make; wait; source devel/setup.bash" #goes to catkin_ws, runs catkin_make, waits until it's made, then sources devel/setup.bash
alias cddcmk="cdc; rm -rf devel build; catkin_make; wait; source devel/setup.bash" #goes to catkin_ws, deletes build and devel directory runs catkin_make, waits until it's made, then sources devel/setup.bash

alias kz="killall gazebo & killall gzserver & killall gzclient" #kills gazebo if frozen
alias rlu="roslaunch udacity_bot udacity_world.launch"
alias rla="roslaunch udacity_bot amcl.launch"
alias rlz="roslaunch udacity_bot rviz.launch"
alias rlr="roslaunch udacity_bot reconfig.launch"
alias rlg="roslaunch udacity_bot gui.launch"
alias rlk="roslaunch udacity_bot keyboard_teleop.launch"
alias rln="rosrun udacity_bot navigation_goal"


