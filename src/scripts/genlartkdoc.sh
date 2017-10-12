#!/bin/bash
#
# Script to create LARtk5 documentation
# including the HTML files for easier
# browsing of documents.
#  requires ros and tree
#
#   V. Santos, 25-Mar-2014
#   V. Santos, 20-Mar-2015
#
###############################################

skipDocRefresh=0   #make it 1 to skip documentation full refresh

if [ "$1" = 1 ]; then skipDocRefresh=1 ; fi

if [ "$ROS_ROOT" = "" ]; then
	echo "Your ROS_ROOT is not set, or ROS not installed in your computer." 
	if [ "`hostname`" = "LARS" ]; then
	        echo "Since this is LARS, some defaults are assumed."
		source /opt/ros/fuerte/setup.bash
		export ROS_PACKAGE_PATH=/home/laradmin/lar5:$ROS_PACKAGE_PATH
	else
		echo "Aborted!"
		exit
	fi
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

#also check if tree is installed
command -v tree >/dev/null 2>&1 || { echo "tree command is not installed or not in path. Install with sudo apt-get install tree" >&2; exit 1; }

list=`rospack list | grep lar |awk '{print $1}'`
listMeta=`rosstack list | grep lar |awk '{print $1}'`

#some macros for colored output
textreset=$(tput sgr0) # reset the foreground colour
red=$(tput setaf 1)
yellow=$(tput setaf 2) 
orange=$(tput setaf 3) 
lightblue=$(tput setaf 4) 
magenta=$(tput setaf 5) 
cyan=$(tput setaf 6) 
lightgray=$(tput setaf 7) 
bold=$(tput smso)
offbold=$(tput rmso)

#place in starting position
roscd atlascar   #Assume atlascar packahage exists ;-)
cd ../../../
larDir=`pwd`

if [ ! ${skipDocRefresh} == 1 ]; then
	rm -r doc  #Perhaps you may want to give a warn before this operation :-)
	mkdir -p doc
	
	#step 1 - create the tags.yaml file
	tagsyamlFile="doc/lartk5_tags.yaml"
	rm -f "$tagsyamlFile"
	for pkg in $list 
	do
		echo "- docs_url: doc/$pkg/html" >> "$tagsyamlFile"
		echo "  location: file://$larDir/doc/tags/$pkg.tag" >> "$tagsyamlFile" 
	done
	
	
	#step 2 - generate doc [and create individual tag files][suppressed]
	#and replicate output into a file for later error correction 
	for pkg in $list 
	do
		pkgPath=`rospack find $pkg`
		echo "${red}==> Processing ${bold} $pkg ${offbold} Documentation ===${textreset}"
		docDir="doc/$pkg"   #relative to current path
		mkdir -p "$docDir"
		#rosdoc_lite -o $docDir -g doc/tags/$pkg.tag "$pkgPath" 2>&1  | tee "$docDir"/doxygen.warnings
		rosdoc_lite -o $docDir "$pkgPath" 2>&1  | tee "$docDir"/doxygen.warnings
	done
	
	#skip step 3 since its behaviour is still under analysis
	#step 3 - 2nd pass of rosdoc_lite using target file
	#for pkg in $list 
	#do
	#	pkgPath=`rospack find $pkg`
	#	docDir="doc/$pkg"   #relative to current path
	#	rosdoc_lite -o $docDir  -t "$tagsyamlFile" "$pkgPath"
	#done
	
	#...cover also metapackages, although this may not be very useful
	#...so it is better to skip it right now.
	#for metapkg in $listMeta 
	#do
	#	pkgPath=`rosstack find $metapkg`
	#	echo "${orange}==> Processing ${bold} $metapkg ${offbold} metapackage Documentation ===${textreset}"
	#	docDir="doc/$metapkg"   #relative to current path
	#	mkdir -p "$docDir"
	#	rosdoc_lite -o $docDir -g doc/tags/$metapkg.tag "$pkgPath" 2>&1  | tee "$docDir"/doxygen.warnings
	#done
	
fi


##############################################################
#Now refresh the HTML files

roscd atlascar
cd ../../../src

patt=`rospack list | grep lar | awk '{printf("%s|",$1)}' | sed 's/|$//'`
#pkgs=`rosstack list | grep lar | awk '{printf("%s ",$1)}'`
rejPatt='data|calibration*|image*|diagnostic|action|build|bin|lib|doc*|triclops|include|launch|config|msg|src|samples|[Mm]atlab*|*rviz*|[Rr]ecordings|models|OpenHap*|*cripts|metapackages|urdf|*_fanuc_*'
patt="atlascar*"

htmlFile="../doc/lartkindex.html"
baseHTMLFile="../doc/index.html"

#tree -n -h --noreport -L 2 -d -T "LAR toolki v4 Packages" -H ./doc ${pkgs} > "$htmlFile"
#tree -n --noreport -L 3 -d -T "LAR toolkit v4 Packages" -I "${rejPatt}" -H ./ ./  > "$htmlFile"
#tree -n --noreport -T "LAR toolkit v4 Packages" -P "atlas*" -H ./ ./  > "$htmlFile"
tree -n --noreport --nolinks -L 4 -d -T "LARtk v5 Packages" -I "${rejPatt}" -H ./ ./  > "$htmlFile"
sed -i 's/courier/helvetica/g' "$htmlFile"
sed -i '/P { font-weight/ s|font-weight: normal|font-weight: normal; font-size: small|' "$htmlFile"
sed -i '/<h1>LARtk .*<\/h1>/ s|h1|h3|g' "$htmlFile"

allpkgs=`rospack list | grep lar | awk '{print $1}'`

for pkg in $allpkgs
do
	link="${pkg}/html/index.html"
	sed -i "s|\b${pkg}\b|<a href=\"${link}\" target=\"rFrame\">${pkg}</a>|" "$htmlFile"
done


#${baseHTMLfile}
cat <<EOT > ../doc/index.html
<!DOCTYPE html>
<html>

<frameset cols="20%,*">
  <frame src="lartkindex.html" name="lFrame">
<!--  <frame src="atlascar/html/index.html" name="rFrame"> -->
  <frame src="struct/lar5deps.html" name="rFrame"> -->
</frameset>

</html>
EOT

###############################################################
# Now build the structure and adjust the index file

echo "${green}==> Generating LARtk5 structure for the starting page ===${textreset}"

cd scripts
./roslarstruct.sh
cd ..
structlink="struct/lar5deps.html"
sed -i "s|\./|<a href=\"${structlink}\" target=\"rFrame\">LARtk STRUCTURE</a>|" "$htmlFile"

###############################################################
#And now build the LARtk5 stats
cd scripts
./myrosstats5.sh

#And now create a Makefile for easier rsync to server
cd ../../doc
cat <<EOT > Makefile
FULLFLAGS=-rtpvulzC --progress --exclude=.pass --max-size=100000
LOCALDIR=.
REMOTEDIR=vsantos@lars.mec.ua.pt:/var/www/lartk5

#############################################################

put:
	@echo "==================================================="
	@rsync \$(FULLFLAGS) \$(LOCALDIR) \$(REMOTEDIR)

EOT

