#!/bin/bash
#
# Script to extract and represent the LAR Toolkit
# structure under ROS (v3+) - now in v4 06-Abr-2014,00:07
# now in v5 20-Mar-2015,17:41
#
# Dir structure:
#	PackageSet -> packages
# Package dependency:
#   after the package.xml files
#   Use the "rospack depends1" command  # NOT operational!
#
#   V. Santos, 09-Abr-2013,10:13
#   V. Santos, 11-Abr-2013,00:3
#      Add package details using HTML-like labels in graphviz
#   V. Santos, 29-Abr-2013,22:48
#      Use SVG format for images allowing tooltips and URLs
#      However this seems to misformat in older versions of graphviz
#      For those cases stick to other formats...
#   V. Santos, 27-Mai-2013,15:43
#      Some cosmetic touchings so graphs are easier to read.
#   V. Santos, 30-Mai-2013,00:04
#      More cosmetic and enhanced tooltips with tweaks in SVG
#      since graphviz has some limitations
#   V. Santos, 01-Jun-2013,10:12
#      Added icons indicating quality of documentation
#      Now authors can't hide their laziness in documenting :-)
#   V. Santos, 06-Abr-2014,00:11
#      Adaptation for v4 which uses catkin in hydro+ ROS distro
#   V. Santos, 20-Mar-2015,17:41
#      Adaptation for v5 - indigo ROS distro and git version control system
################################################################

#First look for base lar dir based on atlascar package
lardir=`rospack find atlascar`
if [ -d "$lardir" ]; then
	BASELARDIR=`rospack find atlascar | awk -F/ '{for(n=1;n<NF-1;n++) printf("%s/",$n); printf("\n");}'`
else
	echo "Package atlascar not found in your system."
	echo "Cannot locate your lar dir. Aborting!"
	exit
fi

useDetailedBlocks=1 #1=Use detailed blocks in images
unattended=0    #by default expect interactive run
useSVG=1   #by default use SVG image format; otherwise it will be PNG

#--------------------------------------------------------
command -v dot >/dev/null 2>&1 || { echo "dot command is not installed or not in path. Install with sudo apt-get install graphviz" >&2; exit 1; }
graphVizVer=`dot -V 2>&1 | awk '{print $5}'`
verMain=`echo $graphVizVer | awk -F. '{print $1}'`
verRel=`echo $graphVizVer | awk -F. '{print $2}'`
#If Graphviz is an early version, some output failures in SVG!
if [ $verMain -lt "3" -a  $verRel -lt "26" ] ; then useSVG=0 ; fi
#--------------------------------------------------------

if [ "$useSVG" -eq "1" ] ; then
	imgformat="svg"
else
	imgformat="png"
fi

#If run from crontab it is unattended (not a tty)
if [ "`tty`" = "not a tty" ]; then unattended=1; fi

################################################################

if [ $unattended = 1 ]; then
	SVNDIR=/home/laradmin/lar5/src
	outputResult=/tmp/lar5struct.out
else
	#SVNDIR="$HOME/lar"
	SVNDIR="$BASELARDIR"
	outputResult=/dev/stdout
fi

pushd . > /dev/null
cd ${SVNDIR}
if [ -d .svn ] ; then
	svnRevision=`svn info | grep Revision | awk '{print $2}'`
else
	svnRevision=0
fi

popd > /dev/null

#==========================

if [ "$ROS_ROOT" = "" ]; then
	echo "Your ROS_ROOT is not set or ROS not installed in your computer" > $outputResult
	echo "Aborting" >> $outputResult
	exit
fi

genDate=`date +%d%b%Y`
echo $genDate > $outputResult

#Create the auxiliary shell functions, such as roscd, etc...
#Since this is an autonomous shell, those shell functions are non-existing
#at this moment. You could run the ROS setup.bash script but that would overwrite 
#your own environment variables and loose the path to your packages.
#With this option, all is preseved and the useful functions are created for this shell too
if [ -e ${ROS_ROOT}/tools/rosbash/rosbash ]; then
	source ${ROS_ROOT}/tools/rosbash/rosbash
fi

#These are the names of all the packages in LAR
larPackages=(`rospack list | grep lar| awk '{print $1}'`)

lPackages=`rospack list | grep lar| awk '{print $1}'`
declare -A AllToolTips   #Declare as associative array to store all tooltips
declare -A AllURLs   #Declare as associative array to store all URLs
declare -A AllQstate  #Declare as associative array to store all info quality state (0 -oK, 1-poor 2-bad)
#fill in all the tooltips for future usage
saveCurrentDir=`pwd`
for f in $lPackages ; do
	dir=`rospack find $f` ; if [ -d "$dir" ] ; then cd "$dir" ; fi 
	tooltip=`cat package.xml | sed -n '/<description/,/<\/description/ p' | sed 's/<.*>//; s/^[ \t]*//; s/\"/\\\"/g; /^$/d' | tr '[\n]' '[ ]'`
	tooltipLen=`echo $tooltip | wc -w`
	if [ "$tooltipLen" -lt 2 ]; then
	    tooltip="package.xml - DESCRIPTION ABSENT! :-("
	    qState=1  #Poor info quality
	else
	    tooltip="package.xml - $tooltip"
	    qState=0   #Acceptable info quality
	fi

	if [ -r "mainpage.dox" ]; then
		#tooltip2=`cat mainpage.dox | sed -n '/\\htmlinclude/,// p' | tail -n +2 |  sed '1,/^$/d; /^$/q; /^$/d' | awk 'length($0)'`
		tooltip2=`cat mainpage.dox | sed -n '1,/\\htmlinclude/ !p' |  sed '/./,$!d' | sed '1,/^$/!d ; /^$/ d' \
			| sed 's/"//g' \
			| sed 's/\\\b//g' \
			| sed 's/\\\n//g' \
			| tr '"\n' ' '  \
			`
		tooltip2Len=`echo $tooltip2 | wc -w`
		if [ "$tooltip2Len" -lt 5 ]; then   #less than 5 words is poor
	    		let qState++  #increment the bad quality of info
		fi
		tooltip2="mainpage.dox - $tooltip2"
		tooltip="$tooltip $tooltip2"  
	fi
	AllToolTips[$f]="$tooltip"
	#AllURLs[$f]="http://lars.mec.ua.pt/lartk/doc/$f/html/"   #This is a bit hardcoded for the LAR server :-(
	AllURLs[$f]="../$f/html/index.html"   #This is a hardcoded for local machine
	AllQstate[$f]=$qState
done

cd $saveCurrentDir

#Now start managing all package sets one by one
PackageSet=`rospack list | grep lar5 | awk '{print $2}' | awk -F/ '{print $6}' | sort -u`
echo "Processing package sets..."
toughSets=("sensors" "perception" "bases" "utils")

for ps in $PackageSet ; do
	echo -n "$ps " >> $outputResult
	pList=`rospack list | grep lar | grep $ps`
	packNames=`rospack list | grep lar| grep $ps | awk '{print $1}'`
	dotFile=$saveCurrentDir/$ps.dot
	#echo $dotFile

	#Create the general properties in the appropriate dot file
	echo "digraph { " > $dotFile
	echo "graph [fontname=\"Helvetica-Oblique\"," >> $dotFile

	#in case the packageset is a difficult one use high packing :-)
	if [[ "${toughSets[@]}" =~ "${ps} " || "${toughSets[${#toughSets[@]}-1]}" == "${ps}" ]]; then
		echo "layout=\"osage\"," >> $dotFile    #Force a specific algoritm for packing
		echo "pack=30, " >> $dotFile
	else
		echo "pack=5, " >> $dotFile
	fi

	echo "fontsize=11," >> $dotFile
	echo "label=\"LARtk-v5, Rev. $svnRevision - Package set dependencies: $ps, LAR-DEMUA, `hostname`, $genDate\\n\"," >> $dotFile
#	echo "labelloc=t," >> $dotFile
	echo "overlap=false, " >> $dotFile
	echo "splines=spline, " >> $dotFile
#	echo "voro_margin=1, " >> $dotFile
#	echo "pack=true, " >> $dotFile
	echo "packmode=\"graph\", " >> $dotFile
#	echo "sep=0.4, " >> $dotFile
#	echo "packmode=\"node\", " >> $dotFile
#	echo "rankdir=LR," >> $dotFile
#	echo "K=1.5, " >> $dotFile
   	echo "];" >> $dotFile
#   echo "node [color=lightblue2, style=filled];" >> $dotFile

#end of properties creations

#Define the properties of the central package blocks: this should be speeded up since it is repeated several times (use arrays?)
	for f in  $packNames ; do
	          dir=`rospack find $f` ; if [ -d "$dir" ] ; then cd "$dir" ; fi 
	      tooltip=${AllToolTips[$f]}
	      packURL=${AllURLs[$f]}
	  if [ "$useDetailedBlocks" -eq 1 ]; then
	  	 #auth=`cat package.xml | grep author  | sed 's/[ \t]*<author>//; s/<\/author>//; s/^ *//'` #full authors
	  	 auth=`cat package.xml | grep "<author" | grep -v '<!--' | sed 's|.*>\(.*\)</author>|\1|'` #full authors
		sauth=`echo $auth | sed 's/,.*$/ et al./'`  #brief author
		if [ -d .svn ] ; then
        lastChgAuthor=`svn info | grep Author | awk -F: '{print $2}'`   #last author that changed it
               update=`svn info | grep Date   | awk '{print $4}'`    #last update time
                 sRev=`svn info | grep "Last Changed Rev" | awk -F: '{print $2}' | sed 's/^ *//'`    #SVN revision of last update
	            else
        lastChgAuthor="no svn"
               update="0"
                 sRev="0"
	 	fi

		  str="$sauth, $update, v$sRev"
		   lf=$((${#f} * 13))   #Estimation of "real" size of string with package name
		   lr=$((${#str} * 8))  #Estimation of "real" size of properties string name
		if [ "$lf" -lt "$lr" ]; then 
		   str="$sauth<BR/>$update, v$sRev"
		fi
		imgStr=''
		if [ "${AllQstate[$f]}" -eq 1 ]; then
		    imgStr='<TD><IMG SRC="images/blue_danger.png"/></TD>'
		elif [ "${AllQstate[$f]}" -gt 1 ]; then
		    imgStr='<TD><IMG SRC="images/red_danger.png"/></TD>'
		fi
		echo "$f [label=<<TABLE><TR><TD>$f</TD>$imgStr</TR><TR><TD><FONT POINT-SIZE=\"8.0\">$str</FONT></TD></TR></TABLE>>]" >> $dotFile
	  fi
	  echo "$f [color=yellow, style=filled, shape=box]; " >> $dotFile
	  echo "$f [URL=\"$packURL\", tooltip=\"$tooltip\"]; " >> $dotFile
	done

#Define the dependencies <depend package="colladadom" /> #rospack deps $f did not work well!
	colVal=("red" "green" "blue" "black" "orange" "purple" "magenta" "navy" "salmon3" "deeppink" "brown" "darkturquoise" "gold" "slateblue" "tan4")
	colIdx=0   #init color indexer.
	ignoreDeps="roscpp|std_msgs"  #dependencies to ignore in lists. These are common to most packages so not useful info.
	for f in  $packNames ; do
		dir=`rospack find $f`   #roscd has failed... :-(
		if [ -d "$dir" ] ; then cd "$dir" ; fi 

		#depList=`rosdep -n keys $f`   #method based on rosdep... not a good solution :-(
		#depList=`rospack depends1 $f`   #method based on rospack. Equivalent to parse the package.xml
		#depList=`cat package.xml | egrep "<depend package|<rosdep name" | sed 's/.*"\(.*\)".*/\t\1/'`   #original method. Circumvents possible rospack dep. failure
		depList=`cat package.xml | egrep "<build_depend>" | grep -v '<!--' | egrep -v "${ignoreDeps}" | sed 's|</*build_depend>||g'`

		for d in $depList ; do
			if [[ "${larPackages[@]}" =~ "${d} " || "${larPackages[${#larPackages[@]}-1]}" == "${d}" ]]; then
			#Thanks to http://stackoverflow.com/questions/3685970/bash-check-if-an-array-contains-a-value
			#test if dependency is a LAR package
				echo "\"$d\" [color=cyan, style=filled]; " >> $dotFile
				echo "\"$d\" [shape=folder]; " >> $dotFile
				echo "\"$d\" [fontsize=12, height=0.25 ]; " >> $dotFile
				echo "\"$d\" [tooltip=\"${AllToolTips[${d}]}\" ]; " >> $dotFile
				echo "\"$d\" [URL=\"${AllURLs[${d}]}\" ]; " >> $dotFile
			#	echo "----$d----"
			else  #It is not a LAR package. Most probably it's ROS's
				#dep packages specific properties:
				echo "\"$d\" [shape=note] ;" >> $dotFile
				echo "\"$d\" [fontsize=12, height=0.25 ]; " >> $dotFile
			fi

			#edgeProperties="[color=\"$HueVal 1.0  1.0\"]"   #HSV coding color [0; 1]
			edgeProperties="[color=${colVal[colIdx]}]"   #Comment this line if you don't want colored edges
				    
			#If a package is both central and dependency it will have both properties (also add edge properties here)
			echo " \"$d\" -> \"$f\" $edgeProperties ;" >> $dotFile
		done
		#update color indexer
		let colIdx++ ; if [ $colIdx -ge ${#colVal[*]} ] ; then colIdx=0; fi
	done

	echo "}" >> $dotFile   #Close dot file

	cd "$saveCurrentDir"

	#gAlgorithm="dot" # OK but LAYOUT TOO WIDE.
	gAlgorithm="neato" # BEST except to perception 
	#gAlgorithm="twopi" # BEST in general -  filter for radial layouts of graphs
	#gAlgorithm="circo" # NOT so GOOD- filter for circular layout of graphs
	#gAlgorithm="fdp" # SOME SEGMENTATION FAULTS! filter for drawing undirected graphs
	#gAlgorithm="sfdp" # OK but some SEGMENTATION FAULTS filter for drawing large undirected graphs
	#gAlgorithm="osage" # Compacts a lot :-) but edges flow very squized!


	#NB. This command may be internally overriden by a local algorithm ("layout" property)
	$gAlgorithm -T$imgformat -O $dotFile  #-Gsize='10,8'

	#$gAlgorithm -Timap -o$ps.map -Tjpeg -o$ps.jpeg -O $dotFile 


done

echo ""
cd "$SVNDIR/scripts"
rm -rf out/* struct/*.$imgformat struct/images/*
mkdir -p out
mkdir -p struct
mkdir -p struct/images
if [ "$useSVG" -eq "1" ] ; then
	sed -i 's/mainpage\.dox/\&#13;\&#13;mainpage.dox/' *.$imgformat   #Tweak the SVG file for multiline tooltip since graphviz does not do it!
	#if you want the hyperlinks to go to a new window just do this:
	#sed -i 's/xlink:href=/target=\"_blank\" xlink:href=/' *.$imgformat
	#or this...
	sed -i 's/xlink:href=/target=\"_parent\" xlink:href=/' *.$imgformat
fi
mv *.$imgformat struct
mv *.dot out
cp -p images/*.png struct/images

#====================================
# Now create the HTML page with the list of dependencies

htmlFile="struct/lar5deps.html"
cat > $htmlFile << EOT
<!DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.0 Strict//EN"
"http://www.w3.org/TR/xhtml1/DTD/xhtml1-strict.dtd">
<html xmlns="http://www.w3.org/1999/xhtml">
<head>
<meta http-equiv="Content-Type" Content="text/html; charset=UTF-8">
<title>LAR Toolkit V5 Dependencies of Package sets</title>
</head>
<body>
<h1>LAR Toolkit V5 Dependencies of Package Sets</h1>
The LAR toolkit (v5) is developed in ROS and is therefore
organised in packages. For easier association, these packages
are organised physically in sets of packages that share
some similar funcionality. Many of the packages depend
on ROS own packages, but some also depend on LAR toolkit's
packages spread over the package sets.
This document illustrates the depedencies of packages within
each of the several <b>package sets</b> that make up the LAR toolkit.
Some common dependencies may be ommited such as: $ignoreDeps.
The package sets currently present in the LAR toolkit are: <b>$PackageSet</b>
<br> <br>
<b>Legend for figures: </b>
<ul>
<li>yellow blocks - packages from the set that depend on others but are not dependencies inside this package set
<li>blue blocks - packages that are dependencies for other packages in this package set
<li>white blocks - packages not from LAR toolkit (mostly external or from ROS repository)
</ul>
Inside the yellow blocks is also mentioned the original creator
of the package, the date of the last modification and its correspondent SVN revision.
Blocks may also include an icon stating the quality of documental package description:
<ul>
<li><IMG SRC="images/blue_danger.png"/> - Poor description (in either package.xml or mainpage.dox)
<li><IMG SRC="images/red_danger.png"/> - Absent description (both in mainpage.dox and mainpage.dox)
</ul>
Individual statistics of packages (for v3) can be found <a href="http://lars.mec.ua.pt/lartk/lar3stats.html">here<a/>.
</br>Individual statistics of packages (for v5 on server probably outdated) can be found <a href="http://lars.mec.ua.pt/lartk5/lar5stats.html">here<a/>.
</br>Individual statistics of packages (for v5 in your local computer) can be found <a href="../lar5stats.html">here<a/>.
<hr>
EOT

for ps in $PackageSet ; do
	packNames=`rospack list | grep lar| grep $ps | awk '{printf("%s, ",$1)}'| sed 's/, $/\./'`
	if [ -r "${SVNDIR}/$ps/README" ] ; then
		README_brief="`cat ${SVNDIR}/$ps/README | sed '1,/@brief/ d; /^$/,$ d'`"
	else
		README_brief="Package set description file README absent."
	fi
	cat >> $htmlFile << EOT
<h2>$ps</h2>
$README_brief
<br>
This package set includes the following packages
highlighted in yellow in the figure: <b> ${packNames} </b><br>
<object data="$ps.dot.$imgformat"></object> 
<hr>
EOT
done

#alternative formats tried...
#<img src="$ps.dot.$imgformat">  
#<A HREF="$ps.map"><IMG SRC="$ps.jpeg" ismap="ismap" /></a>
#<object data="$ps.dot.$imgformat" type="image/svg+xml"></object> 

cat >> $htmlFile << EOT
<i>Page generated on `hostname`, `date`</i>
</body>
</html>
EOT

#Now move entire dir into the doc directory
rm -rf ../../doc/struct
rm -rf out
mv struct ../../doc/
