Beware!

> python test.py 

does not work in this folder. It can't find the item GoTraj from the import gotrajcommander.

You need to copy the test.py to the folder: ws/devel/lib/python2.7/dist-packages/gotrajcommander after you have built the ROS workspace.

It has something to do with the init.py but I haven't figured it out yet.