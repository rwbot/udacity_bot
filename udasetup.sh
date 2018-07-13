#!/usr/bin/env bash

# source /home/workspace/catkin_ws/src/udacity_bot/udasetup.sh

git config --global credential.helper 'cache --timeout=999999' 
git config --global user.name 'rwbot'
git config --global user.email 'rwbotx@gmail.com'

alias cdc="cd /home/workspace/catkin_ws" #custom cd command that goes to catkin_ws
alias cdcs="cd /home/workspace/catkin_ws/src" #custom cd command that goes to catkin_ws/SRC
alias cdcmk="cdc; catkin_make; wait; source devel/setup.bash" #goes to catkin_ws, runs catkin_make, waits until it's made, then sources devel/setup.bash
alias cddcmk="cdc; rm -rf devel build; catkin_make; wait; source devel/setup.bash" #goes to catkin_ws, deletes build and devel directory runs catkin_make, waits until it's made, then sources devel/setup.bash
alias cdu="cdcs; cd /home/workspace/catkin_ws/src/udacity_bot"
alias gitcon="git config --global credential.helper 'cache --timeout=999999'; git config --global user.name 'rwbot'; git config --global user.email 'rwbotx@gmail.com'"

alias rlu="roslaunch udacity_bot udacity_world.launch"
alias rla="roslaunch udacity_bot amcl.launch"


source ~/.bashrc








# alias delg="rosservice call /gazebo/delete_model \"model_name: 'gurdy'\""
# alias glg="roslaunch my_gurdy_description spawn_gurdy.launch"
# alias dglg="delg; wait; glg"

# alias delm="rosservice call /gazebo/delete_model \"model_name: 'mira'\""
# alias glm="roslaunch my_mira_description spawn_mira.launch"
# alias dglm="del; wait; glm"