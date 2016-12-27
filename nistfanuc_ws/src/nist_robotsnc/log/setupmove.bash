#!/bin/bash

cat ./Moveto.log | awk '/Next  Pose/{ print $0 }' \
| sed -n -e 's/^.*Next  Pose.*=\([^|]*\).*/\1/p' \
| sed -n -e 's/:/ /gp'

