
# Use:
# > gnuplot
# gnuplot> load "test.plt"
# To see on screen:
# gnuplot> set term x11
# gnuplot> replot


#SET TERMINAL
set term jpeg
# reset to see: set term x11
set output 'gomtion.jpg'
set title "GO Motion"

#Axes label
set xlabel 'x-axis'
set ylabel 'y-axis'
set zlabel 'z-axis' 

splot   "test1.log"
