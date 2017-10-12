#!/bin/bash
#
# Script to sync data into mainpage.dox
# after other files, such as package.xml
# present of the same dir.
# Examples are authors or descriptions, etc.
#
#      V. Santos, 25-Mar-2014
#
#######################################################

read_dom () {
   local IFS=\>
   read -d \< ENTITY CONTENT
	}

###########################

if [ "$ROS_ROOT" = "" ]; then
	echo "Your ROS_ROOT is not set or ROS not installed in your computer. Aborting" 
	exit
fi

ROS_BASE=`echo $ROS_ROOT | sed 's/\/share.*$//'`

if [ -e ${ROS_ROOT}/tools/rosbash/rosbash ]; then
	source ${ROS_ROOT}/tools/rosbash/rosbash
elif [ -e ${ROS_BASE}/share/rosbash/rosbash ]; then
	source ${ROS_BASE}/share/rosbash/rosbash
else
	echo "Can't find your ROS shell stuff. Aborting."
	exit
fi

roscd atlascar
cd ../../../
cd src

fList=`find . -name mainpage.dox -print`
fList=`find . -name package.xml -print`

cDir=`pwd`
for f in $fList
do
	cd $cDir
	d=`dirname $f`
	cd $d
	if [ ! -r package.xml ]; then
		echo "no package.xml in $d"
	else
		authors=`\
		while read_dom; do
	    	    if [[ $ENTITY = "author" ]]; then
	            	echo $CONTENT
            	    fi
        	done < package.xml`
	fi

	echo "$authors in $d"

	if [ ! -r mainpage.dox ]; then
		echo "no mainpage.dox in $d"
	else
		grep "\\author" mainpage.dox
	fi

	echo "======================"
done


