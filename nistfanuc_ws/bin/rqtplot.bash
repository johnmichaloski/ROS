#!/bin/bash


# rostopic echo fanuc_cartesian_status
rqt_plot /fanuc_cartesian_status/velocity:acceleration:jerk

read -n1 -r -p "Press any key to continue..." key

