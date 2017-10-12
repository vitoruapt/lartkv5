#/bin/bash
#
#  Script to create automatically metapackges
#  using README info from directories...
#  These are the following potential metapackages
#       applications bases demos hmi navigation perception planning sensors utils
#
###############################################
list="applications bases demos hmi navigation perception planning sensors utils"

cDir=`pwd`

for f in $list 
do
	cd "$cDir"
	mkdir -p $f
	cd $f
	if [ -r "../../$f/README" ]; then
		description=`cat ../../$f/README | sed -n '/@brief/,/@examples/p' | tail -n+2 | head -n-2`
	else
		description="$f metapackage"
	fi
	rundepends=`ls -l ../../$f | grep '^d' | awk '{print "  <run_depend>" $NF "</run_depend>"}' `


#CMakeLists.txt
cat << EOT > CMakeLists.txt
cmake_minimum_required(VERSION 2.8.3)
project($f)
find_package(catkin REQUIRED)
catkin_metapackage()
EOT


#package.xml
cat << EOT > package.xml
<?xml version="1.0"?>
<package>
  <name>$f</name>
  <version>5.0.0</version>
  <description>
     $description
  </description>
  <maintainer email="dem-atlas-admin@ua.pt">vsantos</maintainer>
  <license>BSD</license>
  <export>
    <metapackage/>
  </export>
  <buildtool_depend>catkin</buildtool_depend>

  $rundepends

</package>
EOT

done
