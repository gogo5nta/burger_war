#!/bin/bash

# 2020.03.01 作成 T.Ishigami  from https://github.com/OneNightROBOCON/burger_war/blob/master/README.md

# 環境設定
echo "export GAZEBO_MODEL_PATH=$HOME/catkin_ws/src/burger_war/burger_war/models/" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc

# pip のインストール 
sudo apt-get install python-pip
#　requests flask のインストール
sudo pip install requests flask
# turtlebot3 ロボットモデルのインストール
sudo apt-get install ros-kinetic-turtlebot3 ros-kinetic-turtlebot3-msgs ros-kinetic-turtlebot3-simulations
# aruco (ARマーカー読み取りライブラリ）
sudo apt-get install ros-kinetic-aruco-ros

cd ~/catkin_ws
catkin_make
