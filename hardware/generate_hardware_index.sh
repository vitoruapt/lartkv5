#!/bin/bash
#
# script to generate a index.html file after
# all the first level html files
#
#  V. Santos, 02-Jul-2013,17:53
#  V. Santos, 03-Jul-2013,19:04
#       Put on-line if run on LARS
################################################

dd=`date`
cat << EOT > index.html
<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.01//EN" "http://www.w3.org/TR/html4/strict.dtd">
<html>
<head>
<script type="text/javascript">
	var title="LARtk Hardware Platform Description"
	ftitle="<title>"+title+"</title>"
	ptitle="<h1>"+title+"</h1>"
	document.write(ftitle)
	document.write(ptitle)
	document.write("<hr>")
</script>
</head>
<body>
This page describes the hardware subsystems in the several robotics platforms (not yet complete!)
EOT

#fList=(`find . -maxdepth 3 -name \*.htm*`)
#./atlascar_hw/Throttle/throttle.html
#./atlascar_hw/PedalActuation/pedalactuation_index.html
#./atlascar_hw/laser3D/laser3D_index.html
#./arenabots_hw/basecar/arenabots.html

find . -mindepth 2 -maxdepth 3 -name \*.htm\* \
	| grep -v '.svn' \
	| awk -F/ '{print $2 " " $3 " " $0}' \
	| sort -r \
	| sed 's/_hw / /' \
	| awk '{ if ( platform[$1] < 1) print "</ul><h2>"$1" platform</h2><ul>" ; print "<li><a href=\""$3"\">"$2"</a></li><br>" ; ++platform[$1]} END { print "</ul>" }' \
	>> index.html

cat << EOT >> index.html
<br>
<hr>
This page is generated automatically.
<address>LARtk v4, $dd</address>
</body>
</html>
EOT

