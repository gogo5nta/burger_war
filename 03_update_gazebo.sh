#!/bin/bash

#ubuntu 16.04でインストールしたROS(kinetic)はgazeboに不具合があるため、ver7にアップデートすることをお勧め(7.15までアップデート可能)
#https://roboticshubchallenge.slack.com/archives/CL9TFG2RZ/p1564533784053800?thread_ts=1564533261.050300&cid=CL9TFG2RZ

sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable lsb_release -cs main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
