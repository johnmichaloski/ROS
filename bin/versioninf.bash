!/bin/bash

# Sorted Package Version Information in simple HTML Table Format
# Usage  ./versioninfo.bash pkgname
# Assume ROS installed and pkg compiles

pkgs=`rospack depends $1 |sort `

echo "<HTML><BODY>"
echo "<h1> " $1 " Package Version Dependencies </h1>"
echo "<TABLE>"
echo "<TR><TH>Package</TH><TH>Version</TH></TR>"
for pkg in $pkgs
do
   ver=`rosversion $pkg `
   echo  "<TR><TD> $pkg </TD><TD>  $ver </TD></TR>"
done
echo "</TABLE></BODY></HTML>"
