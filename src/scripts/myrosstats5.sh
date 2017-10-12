#!/bin/bash
#
# Script to produce some stats on the documentation
#
#   V. Santos, 01-Apr-2012,02:08
#   V. Santos, 03-Apr-2012,00:25
#   V. Santos, 15-Apr-2012,18:11
#       More robust to different users and added a command line option to force E-mail sending
#   V. Santos, 22-Apr-2012,15:33
#       More exact mainpage.dox analysis, and also generates an HTML output
#   V. Santos, 17-May-2012,23:21
#       More details on stats including links to the on-line documentation
#   V. Santos, 05-Dez-2012,17:18
#	Includes information whether it has been tested in ROS FUERTE
#	this is checked with the presense of file FUERTE_OK in base package dir.
#   V. Santos, 08-Abr-2013,15:22, 16:56
#       Includes analysis of @file and main @brief directives
#   V. Santos, 16-Abr-2013,00:29
#       Some refinements in output and proceeds even if doxygen.warning
#       is absent. GROOVY_OK added for the same reason as FUERTE_OK
#   V. Santos, 18-Abr-2013,23:18
#       A few more infos on packages
#   V. Santos, 29-Abr-2013,19:12
#   	 in source code were purged
#   V. Santos, 26-Mai-2013,16:07
#        Includes the pack_set field (packages_set)
#   V. Santos, 04-Jun-2013,15:20
#        Minor cosmetic changes and increased robustness on other machines
#   V. Santos, 26-Mai-2014,01:10
#        Migration to lar4 
#   V. Santos, 20-Mar-2015,17:44
#        Migration to lar5 which uses git instead of svn
################################################################

read_dom () {
   local IFS=\>
   read -d \< ENTITY CONTENT
}
		      

echo "NOT YET OPERATIONAL"; exit

unattended=0    #by default expect interactive run
#If run from crontab it is unattended (not a tty)
if [ "`tty`" = "not a tty" ]; then unattended=1; fi

#Variable to impose E-mail stats delivery (always done in unattended mode)
ForceEmail=0

if [ $# -eq 1 ]; then
	case "$1" in
	-h | --h | --help | -help)
			echo "$0 -m -> to force e-mail sending"
			exit
			;;
	-m)
			ForceEmail=1
			;;
esac
fi

################################################################

#Create a temporary txt file and then create an HTML version
#The procedure is the same in attended or unattended
#one will do a simple file output to stdout, and the other will E-mail it
tkVersion=5

tmp_statFile=/tmp/_stats.txt
statFile=/tmp/lar${tkVersion}stats.txt
HTMLout=/tmp/lar${tkVersion}stats.html
compFailures=/tmp/compileFailures.out    #This is a file that may exist in case there was a recent compilation
#Example of its format
# ==> leader_follower [FAIL] [ 0.66 seconds ]                
# ==> phua_haptic [FAIL] [ 2.66 seconds ]                    
# ==> pointcloud_clustering [FAIL] [ 0.20 seconds ]          
# ==> kmlPathCreator [FAIL] [ 0.51 seconds ]                 
# ==> vs_pack1 [FAIL] [ 0.20 seconds ]                       
# ==> atlascar_teleop [FAIL] [ 0.58 seconds ]                


#---- Check you have ROS -  begin-----------------------------

if [ "$ROS_ROOT" = "" ]; then
		echo "Your ROS_ROOT is not set or ROS not installed in your computer" > $outputResult
		echo "Aborting" >> $outputResult
		exit
fi

#Create the auxiliary shell functions, such as roscd, etc...
#Since this is an autonomous shell, those shell functions are non-existing
#at this moment. You could run the ROS setup.bash script but that would overwrite 
#your own environment variables and loose the path to your packages.
#With this option, all is preserved and the useful functions are created for this shell too
if [ -e ${ROS_ROOT}/tools/rosbash/rosbash ]; then
source ${ROS_ROOT}/tools/rosbash/rosbash
#which is the same as the bourne shell original:
#  . ${ROS_ROOT}/tools/rosbash/rosbash
fi

larinrospath=`echo $ROS_PACKAGE_PATH | grep -c /$PREFIXLARNAME`
if [ "$larinrospath" = "0" ]; then
		echo "Can't find the $PREFIXLARNAME* dir in your ROS_PACKAGE_PATH"   > $outputResult
		echo "Make sure your ROS_PACKAGE_PATH variable includes the required paths"   >> $outputResult
		echo "You can edit manually the $0 script or adjust that variable in ~/.bashrc"   >> $outputResult
		echo "Aborting script $0"   >> $outputResult
		exit
fi
#---- Check you have ROS -  end-----------------------------


#for lar3
pkgFile='manifest.xml'
homeLarScripts="$HOME/lar/scripts"

#for lar4 and lar5
pkgFile='package.xml'
homeLarScripts="$HOME/lar${tkVersion}/src/scripts"
LARDOC="$HOME/lar${tkVersion}/doc"


# Check if you have txt2html
command -v txt2html >/dev/null 2>&1 || { echo "txt2html command is not installed or not in path. Install with sudo apt-get install txt2html" >&2; exit 1; }



if [ $unattended = 1 -o $ForceEmail = 1 ]; then
	SVNDIR=/home/laradmin/lar
	#svnRevision=`svn info $SVNDIR | grep Revision | awk '{print $2}'`
	svnRevision='NOT READY'
	outputResult=/tmp/lar${tkVersion}stats.out
	#userList="almeida.j@ua.pt andrejoliveira@ua.pt joelfpereira@ua.pt mriem@ua.pt pedropinheiro@ua.pt pemdp@ua.pt pmbc@ua.pt procopio@ua.pt rpascoal@ua.pt salvado.p@ua.pt talhada@ua.pt vitor@ua.pt"
	userList=dem-atlas-soft@ua.pt   #This is the software development team...
else
	outputResult=/dev/stdout
	echo "Analysing code... please wait some seconds." >> $outputResult
	#First look for base lar dir based on atlascar package
	lardir=`rospack find atlascar`
	if [ -d "$lardir" ]; then
		cd -P "$lardir"; cd ../.. #because of possible symbolic links :-(
		BASELARDIR=`pwd`
		SVNDIR=$BASELARDIR
		#svnRevision=`svn info $SVNDIR | grep Revision | awk '{print $2}'`
		svnRevision='NOT READY'
	else
		echo "Package atlascar not found in your system." >> $outputResult
		echo "Cannot locate your lar dir. Aborting!" >> $outputResult
		exit
	fi
fi

#==========================

dir=`rospack find atlascar`
if [ -d "$dir" ]; then
	cd "$dir"
	cd ../..
	dir=`pwd`  #Now we are localized :-)
	#Well, next line works for me :-) but if you put documentation elsewhere it should be looked for...
	doxygenW="`pwd`/doxygen.warnings"

	#For lar4 and lar5
	doxygenW="$HOME/lar${tkVersion}/doc/alldoxygen.warnings"
	find .. -name doxygen.warnings | xargs cat | grep -v INCLUDE_PATH > ${doxygenW}   #concatenate all doxygen.warnings files

	#-----------------------------


	if [ "`hostname`" = "LARS" ] ; then
		#special location in case of LARS server
		doxygenW="/var/www/lartk/doxygen.warnings"
	fi
else
	echo "Package atlascar not found in your system." >> $outputResult
	echo "Cannot locate your lar dir. Aborting!" >> $outputResult
	exit
fi

if [ ! -r $doxygenW ] ; then
		echo "File $doxygenW not found! Run the command \"myrosdoc lar\""
		echo "Script will proceed without Doxygen warning informations"
		doxygenW='/dev/null'
		#echo "Exiting."
		#exit
	else
		#clean the Doxygen file whenever the cleaning command is available
		if [ -x ${homeLarScripts}/clean_doxygen_warnings_file.sh ] ; then 
			${homeLarScripts}/clean_doxygen_warnings_file.sh $doxygenW
		#echo "$doxygenW was cleaned"
		fi
fi


listOfPackges=`rospack list | grep lar | awk '{print $2}'`

###########/!\##### THIS IS OLD ###################
if [ -r $compFailures ]; then
#this is the expected format
# ==> leader_follower [FAIL] [ 0.66 seconds ]                
	listOfFails=`awk '{print $2}' $compFailures`
else
	listOfFails=''
fi

#                  AUTHOR     PACKAGE PACK_SET PKSUBSET  Creation  ERR     WARN   .dox Head   .dox Desc   BAD@file   BAD@brief   #files   #lines   #eff. lines  FAIL FUERTE
   #strFormat=' :  %-30.30s  : %-30.30s  : %10.10s : %10.10s : %10.10s : %4.4s  : %4.4s  : %-10.10s  : %-9.9s : %-10.10s : %-10.10s : %6.6s  :  %6.6s  :  %11.11s :  %12.12s  :  %10.10s : %4.4s : %6.6s : %6.6s : %4.4s :\n'
#printf "$strFormat" "AUTHOR" "PACKAGE" "PACK_SET" "PKSUBSET" "Creation"  "ERR"   "WARN" ".dox HEAD" ".dox DESC" "BAD@file" "BAD@brief" "#files" "#lines" "#eff. lines" "Last Changed" "Date" "Rev." "FUERTE" "GROOVY" "FAIL" \
   strFormat=' :  %-40.40s  : %-30.30s  : %10.10s : %10.10s : %10.10s : %4.4s  : %4.4s  : %-10.10s  : %-9.9s : %-10.10s : %-10.10s : %6.6s  :  %6.6s  :  %11.11s :  %12.12s  :  %10.10s : %4.4s :\n'
printf "$strFormat" "AUTHOR" "PACKAGE" "PACK_SET" "PKSUBSET" "Creation"  "ERR"   "WARN" ".dox HEAD" ".dox DESC" "BAD@file" "BAD@brief" "#files" "#lines" "#eff. lines" "Last Changed" "Date" "Rev." \
			> $tmp_statFile
for d in ${listOfPackges} ; do
		cd ${d}
		#srcList=`find . -path "*/src/*" -a \( -name \*.cpp -o -name \*.h -o -name \*.c -o -name \*.hpp -o -name \*.cc \) -print`
		srcList=`find . \( -path "./src/*" -o -path "./include/*" \) -a \( -name \*.cpp -o -name \*.h -o -name \*.c -o -name \*.hpp -o -name \*.cc \) -print`

		#echo -----
		#echo $srcList
		#echo -----

		if [ "${srcList}" = "" ] ; then
			totLines=0
			effLines=0
			effLines2=0
			numFiles=0
			badfileRatio="-/-"  #number of files with absent/bad @file directive in total files
			badbriefRatio="-/-"  #number of files with absent/bad @brief directive in total files
		else
			totLines=`cat ${srcList}  | wc -l`

			#Count total effective code lines. Remove all comments occuring at line beginning as well as empty lines
			effLines=`cat ${srcList} | sed 's/^[ \t]*//' | sed '/^\/\//d' | sed '/^\/\*/,/\*\/$/d' | sed '/^*$/d' | wc -l`

			#Perhaps we could format all to a standard with a defined indent with the apropriate definition file
			#and the code would be something like: (errors and warnings should be redirected ....)
			#effLines2=`cat ${srcList} | sed 's/^[ \t]*//' | sed '/^\/\//d' | sed '/^\/\*/,/\*\/$/d' | sed '/^*$/d' | indent | wc -l`

			numFiles=`echo ${srcList} | wc -w`  #all sources
			withFileFiles=`egrep -m 1 -H "[@\\]file" ${srcList} | wc -l`  #files with the @file directive
			withoutFileFiles=$(($numFiles - $withFileFiles))               #files without @files firective

			#Calculate the ratio of bad @file directives in potential source files
			#This version includes a sed filter to erase spurious  characteres that were mileading the editors :-((
			#Please avoid to use editors that append CR (13 ascii) at the end of lines (like windows based stuff ;-)
			badFiles=`egrep -m 1 -H "[@\\]file" ${srcList} \
				    | sed 's/^\.\/src\///;s/^\.\/include\///;s///g' \
					| sed 's/:.*file/ file/' \
					| awk '{print $NF}' \
					| awk 'BEGIN {toBad=0;} {if (NF>2 && $1!=$3 ) toBad++;} END {printf("%d\n",toBad);}'`
			badFiles=$(($badFiles + $withoutFileFiles))
			badfileRatio="$badFiles/$numFiles"  #statistics for user
			bfRatio=$((100 * $badFiles / $numFiles ))   #bad @file ratio in % Above 50% of bad files is Bad!

			if [ "$bfRatio" -eq 0 ] ; then
				badfileRatio=`echo -n $badfileRatio"_fine"`
			elif [ "$bfRatio" -lt 50 ] ; then
				badfileRatio=`echo -n $badfileRatio"_poor"`
			else
				badfileRatio=`echo -n $badfileRatio"_bad"`
			fi

			MINCHARSFORFILEBRIEF=20  #minimum number of chars to have a decent brief
			badBrief=`egrep -m 1 -n -H "[@\\]file" ${srcList} \
				| gawk -F: '{ "tail --lines=+"$2+1 " " $1 "| head -1"|getline d; printf("%s\n",d); }' \
				| sed 's/^.*[@\\]brief *//' \
				| awk -v minChars=$MINCHARSFORFILEBRIEF 'BEGIN {sL=0} {if (length() < minChars ) sL++;} END {printf("%d\n",sL);}'`

			badBrief=$(($badBrief + $withoutFileFiles))
			badbriefRatio="$badBrief/$numFiles"  #statistics for user
			bbRatio=$((100 * $badBrief / $numFiles ))   #bad @brief ratio in % Above 50% of bad briefs is Bad!

			if [ "$bbRatio" -eq 0 ] ; then
				badbriefRatio=`echo -n $badbriefRatio"_fine"`
			elif [ "$bbRatio" -lt 50 ] ; then
				badbriefRatio=`echo -n $badbriefRatio"_poor"`
			else
				badbriefRatio=`echo -n $badbriefRatio"_bad"`
			fi
		fi

		#pwd; echo $badfileRatio

		#Date of creation
		creationDate=`svn log -q | grep \| | tail -1 | awk -F\| '{print $3}' | awk '{print $1}'	`

		#The name of the package as in the path (used as short description here)
		shortd=`echo $d | awk -F/ '{print $NF}'`

		#The name of the packages set where package is included
		#/home/vitor/lar/sensors/laser/RCPRG_laser_drivers/LMS1xx
		fullpack=`echo $d | sed 's/.*lar\?\///'`

		#For lar4 it is:
		#/home/vitor/lar4/src/perception/pedestrians/multimodal_pedestrian_detect
		fullpack=`echo $d | sed 's|.*lar.*/src/||'`

		#echo $fullpack
		pack_set=`echo $fullpack | awk -F/ '{print $1}'`
		pack_subset=`echo $fullpack | awk -F/ '{print $2}'`
		if [ "$pack_subset" == "$shortd" ] ; then pack_subset="0" ; fi   #forcing zero will impose empty cell in html output

		if [ -r ${pkgFile} ] ; then
			#auth=`cat ${pkgFile} | grep author  | sed 's/[ \t]*<author>//; s/<\/author>//; s/^ *//'`
			#auth=`cat ${pkgFile} | grep author  | sed 's/.*<author>//; s/<\/author>//; s/^ *//'`
			auth=`while read_dom; do if [[ $ENTITY = "author" ]]; then echo -n "$CONTENT, " ; fi ; done < ${pkgFile}`; auth=`echo $auth | sed 's/,$//'`
			#desc=`cat ${pkgFile} | grep "<description"  | awk -F\" '{print $2}'`
			desc=`while read_dom; do if [[ $ENTITY = "description" ]]; then echo -n "$CONTENT " ; fi ; done < ${pkgFile}`
			errs=`cat ${doxygenW}  | egrep -i error   | egrep "/${shortd}/|\[${shortd}\]" | wc -l`
			#warns=`cat ${doxygenW} | egrep -i warning | grep -v "does not exist" | grep "/${shortd}/|\[${shortd}\]" | wc -l`
			warns=`cat ${doxygenW} | egrep -i warning | egrep "/${shortd}/|\[${shortd}\]" | wc -l`
			#Previous discarded some irrelevant "does not exist"... check in future versions
		fi

		#Not currently used but kept for future possibilities
		if [ -r ROS_NOBUILD ] ; then
				rosnobuild='X'
		else
				rosnobuild='-'
		fi

		#These are no longer used
		if [ -r FUERTE_OK ] ; then
				fuerteOK='OK'
			elif [ -r FAIL_FUERTE ] ; then
				fuerteOK='FAIL'
			else
				fuerteOK='-'
		fi
		if [ -r GROOVY_OK ] ; then
				groovyOK='OK'
		else
				groovyOK='-'
		fi
		##############################################

		#######/!\################### DOES NOT WORK
		#Analyse whether there was a build fail in last attempt (stored in /tmp... hopefuly)
		buildFail=`echo $listOfFails | awk -v pkg=${shortd} '{ if( NF==0 ) { print " "; next } for (var=1;var<=NF;var++) if( $var==pkg ) print "X";}'`
		# I would expect the following to do it but it does not :-(
		#buildFail=`echo $listOfFails | awk -v pkg=${shortd} '{ split($0,arr," "); if( pkg in arr ) print "X";}'`

		#Analise last intervention in svn (use the svn info)
		lastChangeAuthor=`svn info | grep Author | awk -F: '{print $2}'`
		  lastChangeDate=`svn info | grep Date   | awk '{print $4}'`
		            sRev=`svn info | grep "Last Changed Rev" | awk -F: '{print $2}'`
		#######/!\################### END OF DOES NOT WORK

		## Analyse mainpage.dox
		fff=mainpage.dox
		if [ -r $fff ] ; then   #mainpage present, but how good?
			#\mainpage title (yes/no) - #words in \mainpage header description
			mpadescw=`cat $fff | grep "\\mainpage" | wc -w`
			if [ $mpadescw -lt 2 ] ; then
				mainPageHead='NOT_OK'
			else 
					if [ $mpadescw -lt 4 ] ; then
						mainPageHead="GOOD";
					else
						mainPageHead="FINE";
					fi
			fi

			#number of tokens in file: lines starting by \something
			numtoks=`cat ${fff} | grep '^\\\[a-zA-Z].*' | wc -l`
			#This statistical descriptor not yet used by now, V. Santos, 22-Apr-2012,15:12

			# Code corresponding to package description
			# after \htmlinclude and before next \token  (erase them to avoid their counting)
			# and discarding comments and empty lines. Number of words
			descnumw=`cat $fff | \
						sed '/^\/\*/d' | sed '/^\*\//d' | \
						sed '/^<!--/,/^-->/d' | \
						sed -n '/\\htmlinclude/,/^\\[a-zA-Z][a-zA-Z].*/p' | \
						sed '/^\\\[a-zA-Z][a-zA-Z].*/d' | \
						sed '/^$/d' |\
						wc -w`
			if [ $descnumw -lt 7 ] ; then
					packageLongDesc="POOR"
			else
					if [ $descnumw -lt 20 ] ; then
						packageLongDesc="GOOD"
					else
						packageLongDesc="FINE"
					fi
			fi
		else   #The mainpage.dox does not exist!
			mainPageHead="Missing!"
			packageLongDesc="Missing!"
		fi
		#End of mainpage.dox analysis

		#Print the line with the package status
		printf "$strFormat" \
				"${auth}" ${shortd} ${pack_set} ${pack_subset} ${creationDate} $errs $warns $mainPageHead $packageLongDesc \
				${badfileRatio}\
				${badbriefRatio}\
				${numFiles} ${totLines} ${effLines} ${lastChangeAuthor} ${lastChangeDate} \
				${sRev} \
				>> $tmp_statFile

				#${sRev} ${fuerteOK} ${groovyOK} ${buildFail} \

done

#=========================

pageTitle="Statistics of LAR Toolkit V${tkVersion}"
tableCaption="Statistics of LAR Toolkit V${tkVersion} R"$svnRevision

if [ $unattended = 1 -o $ForceEmail = 1 ]; then
	ftitle=$pageTitle
	cat << EOT > $statFile
		$tableCaption
    Data includes the per package number of errors and warnings from Doxygen execution,
    as well as some numbers concerning the size of packages in terms of source code.
    Check the script in scripts/myrosstats for more details and explanations.

    More details on packages status and their dependencies can be checked at http://lars.mec.ua.pt/lartk${tkVersion}/struct/lar${tkVersion}deps.html

$tableCaption
EOT
else   #not unattended, meaning normally that a user launched it :-)
	ftitle=$pageTitle
	cat << EOT > $statFile
    More details on packages status and their dependencies can be checked at http://lars.mec.ua.pt/lartk4/struct/lar4deps.html

$tableCaption
EOT
fi

#complete the output file with results
#and sort by revision
#Remind: the fields are separated by ":" and there is a separator at
#the begin so the number of fields for the sort operaion is actually +1

head -1 $tmp_statFile >> $statFile
#tail --lines=+2 $tmp_statFile | sort -i >> $statFile
#tail --lines=+2 $tmp_statFile | sort -r -n -t: -k15 >> $statFile
#tail --lines=+2 $tmp_statFile | sort -t: -k15,15nr -k3,3 >> $statFile
#tail --lines=+2 $tmp_statFile | sort -t: -k16,16nr -k3,3 >> $statFile
#tail --lines=+2 $tmp_statFile | sort -t: -k6nr -k6.7nr -k6.10nr >> $statFile    #sort a date... ": 2012-12-4 :"
tail --lines=+2 $tmp_statFile | sort -t: -k18,18nr -k5,5 >> $statFile
cat << EOT >> $statFile

Meaning of the fields:
 - AUTHOR - name of author as indicated in the ${pkgFile}
 - PACKAGE (ROS type) - The package as registered in the ROS framework.
 - PACK_SET - The package set where package lays
 - PKSUBSET - The package subset within the package set 
 - Creation - Creation date of the package.
 - ERR - number of errors in doxygen output (doxygen.warnings file).
 - WARN - number of warnings in doxygen output (doxygen.warnings file).
 - .dox HEAD - status of the mainpage.dox \mainpage header (entry point of package documentation).
 - .dox DESC - status of the mainpage.dox description of package .
 - BAD@file - Number of wrong or absent @file directives in total of source files.
 - BAD@brief - Number of poor or absent @brief file description in total of source files.
 - #files - Number of source code files developed in src directory (*.cpp *.h *.c *.hpp *.cc).
 - #lines - Total number of lines of previous files (*.py still excluded).
 - #eff. lines - The same as before but removing all empty lines and lines with comments at the line beginning.
 - Changed - Author of last change.
 - Date - Date of last change.
 - Rev. - Revision of last change.
EOT

# - FUERTE-Marks that the package has been sucessfully compiled in ROS Fuerte.
# - GROOVY-Marks that the package has been sucessfully compiled in ROS Groovy.
# - FAIL - Indicates that the package failed on the last automatic documented build attempt!

#Generate the HTML
# the -8 preserves 8 bit chars which must be later converted in HTML defitions
txt2html --xhtml \
		 -8\
		 --titlefirst \
		 --tables \
		 --bold_delimiter "" \
		 --italic_delimiter "" \
		 -t "$ftitle" \
		 $statFile > $HTMLout

#----- This block inserts a special line for the char set.
# could be done with a more elegant script but I have no time for it now :-)
beginLine=`grep -n "<head>" $HTMLout | awk -F: '{print $1}'`
head -$beginLine $HTMLout > /tmp/tmp1
beginLine=`grep -n "<head>" $HTMLout | awk -F: '{print $1+1}'`
cat << EOT >> /tmp/tmp1
<meta http-equiv="Content-Type" Content="text/html; charset=UTF-8">
EOT
cat $HTMLout | tail --lines=+$beginLine >> /tmp/tmp1
mv /tmp/tmp1 $HTMLout
#----- End of block for the special char set

#Block to include links in HTML file for direct linking to on-line manual
#For a package named atlascar_teleop create the link to
#http://lars.mec.ua.pt/lartk/doc/atlascar_teleop/html/
#or more precisely
#<a href="http://lars.mec.ua.pt/lartk/doc/$PKG/html/">$PKG</a>
#</td><td>atlascar_teleop</td>   #use delimiters to ensure full words detection ...
for pkg in ${listOfPackges} ; do
	pkg=`echo $pkg | awk -F/ '{print $NF}'`
	if [ "`hostname`" = "LARS" ] ; then
		sed -i "s/<\/td><td>$pkg<\/td>/<\/td><td><a href=\"http:\/\/lars\.mec\.ua\.pt\/lartk\/doc\/$pkg\/html\/\">$pkg<\/a><\/td>/" $HTMLout
	else   ## It's local. Use a local dir... hopefully this can work
		#Man, the next trick is awesome... just to escape the / slash in paths :-)
		#echo $HOME | sed 's/\//\\\//g'  #This would work but to put into a variable you must do more!
		escHOME=`echo "$HOME" | sed 's/\\//\\\\\\//g'`   #actually you can use other sed delimiters
		sed -i "s/<\/td><td>$pkg<\/td>/<\/td><td><a href=\"file:\/\/\/$escHOME\/lar4\/doc\/$pkg\/html\/index\.html\">$pkg<\/a><\/td>/" $HTMLout
	fi
done

gen=`date`
cat << EOT >> $HTMLout
<hr>
<i>File generated on $HOSTNAME, $gen</i>
EOT

#Now do some embelishments in HTML :-)
#<table border="1" summary="">
#<table border="1" summary="" style="font-size:10pt;font-family:Helvetica">

cat ${HTMLout} \
	| sed 's/\(^<table border=.*\)>$/\1 style=\"font-size:10pt;font-family:Helvetica\">/g' \
	| sed 's/Missing\!/<b><font color=\"red\">Missing\!<\/b><\/font>/g' \
        | sed 's/FINE/<b><font color=\"green\">FINE<\/b><\/font>/g' \
        | sed 's/_fine/_<b><font color=\"green\">fine<\/b><\/font>/g' \
        | sed 's/_bad/_<b><font color=\"red\">bad<\/b><\/font>/g' \
        | sed 's/_poor/_<b><font color=\"orange\">poor<\/b><\/font>/g' \
        | sed 's/POOR/<b><font color=\"orange\">POOR<\/b><\/font>/g' \
        | sed 's/NOT_OK/<b><font color=\"red\">NOT_OK<\/b><\/font>/g' \
	> /tmp/aa

cp /tmp/aa $HTMLout

#For lar4 and lar5
cp $HTMLout $LARDOC


#Finally send info to output (file and/or E-mail)

if [ $unattended = 1 -o $ForceEmail = 1 ]; then
		mail -s "[LARtk] LAR${tkVersion} rev. $svnRevision statistics" -a "From: LAR Toolkit Watcher <root@lars.mec.ua.pt>" -a "Content-Type: text/html; charset=\"UTF-8\"" -t $userList < $HTMLout
else   #lauched in the console command line. A simple cat is required (OK, with some cleaning!)
	cat $statFile | sed 's/^ :  //' | sed 's/ :$//'
	if [ "$HOSTNAME" ==  "robotica3" -o "$HOSTNAME" ==  "robotica4" -o "$HOSTNAME" ==  "diplodoco2" ]; then
		cp $HTMLout ${homeLarScripts}   #also copy the HTML to current dir for inspection
		cp $statFile ${homeLarScripts}   #also copy the HTML to current dir for inspection
	fi
fi

/bin/rm -f $tmp_statFile

