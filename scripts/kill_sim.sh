#!/bin/bash

pkill -f sim_bag 
pkill -9 -f gz
pkill -9 -f Autopilot
tmux kill-server