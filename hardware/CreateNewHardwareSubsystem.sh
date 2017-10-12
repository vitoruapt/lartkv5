#!/bin/bash
#
# Script to create the base directory and
# file structure to document hardware/firmware
# developments in the LAR toolkit environment.
#
#  The structure of directories and files is:
#
#  <Platform>
#     |____ <Subsystem>
#             |___ <subsystem>_index.html - HTML file with a general description of the subsystem
#             |___ susbsystem.js          - Javascript File with a generic definitions about system
#             |___ Mechanical             - Folder with mechanical drawings and CAD models
#             |___ Electrical             - Folder with electrical and electronics circuits (schematics, PCBs, etc.)
#             |___ Code                   - Code for microcontrollers and other firmware
#
#
#  Current platforms
#	atlascar_hw
#	atlasmv_hw
#	atlas20XX_hw
#	arenabots_hw
#	humanoid_hw
#	common_hw
#
#############################################################

AllPlatforms=(atlascar_hw atlasmv_hw atlas20XX_hw arenabots_hw humanoid_hw common_hw)
echo "Pick the platform where to create subsystem"
echo "	0-atlascar_hw"
echo "	1-atlasmv_hw"
echo "	2-atlas20XX_hw"
echo "	3-arenabots_hw"
echo "	4-humanoid_hw"
echo "	5-common_hw"
echo "[default is 0-atlascar_hw]"
read _platform
if [ ${#_platform} == 0 ] ; then _platform=0 ; fi
if [[ $_platform != *[[:digit:]]* ]]; then echo "Must select a numbered option"; exit; fi
       
if [ ${_platform} -lt 0 -o ${_platform} -gt 5 ] ; then echo "Choice out of range. Aborted"; exit; fi
_platformToUse=${AllPlatforms[$_platform]}

echo "Enter the name of the subsystem to create:"
read _title
_title=`echo $_title | tr ' ' '_'`

if [ ${#_title} == 0 ] ; then
	echo "You must supply a name for the subsytem to create"
	echo "such as 'PedalActuation', 'SteeringWheel', etc."
	exit
fi

echo "Enter the name of the author (enter to use automatic)"
echo "     default is ${USER}"
read _author
if [ ${#_author} == 0 ] ; then
	_author=$USER
fi
_date=`date +%d-%b-%Y`

echo "The following properties will be assumed."
echo "	Platform to use=$_platformToUse"
echo "	Name of the subsystem=$_title"
echo "	Author=$_author"
echo "	Creation Date=$_date"
echo "Continue [Y/n]?"
read _go
if [ "$_go" == "n" ] ; then
	echo "Nothing created. Aborted by user."
       exit	
fi

if [ -d "${_platformToUse}/${_title}" ] ; then
	echo ${_platformToUse}/${_title} aready exists. Abort.
	exit
fi

cd ${_platformToUse}
mkdir ${_title}
cd ${_title}
mkdir Electrical Mechanical Code


cat << EOT > subsystemId.js
<!-- Code to insert directly on the importing HTML file -->
<!-- V. Santos, 24-Apr-2012,14:42 -->
<!-- Adjust the following lines  -->

var title="${_title}"
var author="${_author}"
var date="${_date}"

<!-- ========================= -->
<!-- No need to edit below -->

ftitle="<title>"+title+"</title>"
ptitle="<h1>"+title+"</h1>"
pauthor="<address>Author: "+author+", "+date+"</address>"

document.write(ftitle)
document.write(ptitle)
document.write("<hr>")
document.write(pauthor)

<!--The next would be metadata ... can skip by now
	fauthor="<meta name=\"date\" content="+author+">"
	fdate="<meta name=\"date\" content="+date+">"
	document.write(fauthor)
	document.write(fdate)
-->
EOT



cat << EOT > ${_title}_index.html
<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.01//EN" "http://www.w3.org/TR/html4/strict.dtd">
<html>
<head>
<!-- Just insert some automatic text -->
<script src="subsystemId.js" type="text/javascript"> </script> 
</head>
<body>
<h2>Brief description</h2>
This subsystem is concerned with ...
<h2>System operation</h2>
[Long description]<br>
This should be a longer description of what the system does.
<h2>Mechanical components</h2>
[conceptual mechanism, Drawings of fixing and supporting systems...]<br>
<h2>Electrical circuits</h2>
[Electrical circuits of the devices - use images to illustrate]<br>
<h2>Firmware and similar code<br>
</h2>
[Describe what programs and environments are used along with links to relevant code]<br>
<h2>Maintenance</h2>
[Here lay specific instructions for technitians and engineers]<br>
</body>
</html>
EOT

echo "	+--------------------------------------------------+"
echo "	| Now you must edit the HTML file just created and |"
echo "	| include the relevant data in local directories:  |"
echo "	|   Mechanical                                     |"
echo "	|   Electrical                                     |"
echo "	|   Code                                           |"
echo "	+--------------------------------------------------+"

