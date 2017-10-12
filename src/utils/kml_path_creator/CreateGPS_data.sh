#!/bin/bash

#Start log coordinates

# ALBOI_1 START PARAMETERS
OX=-731113.42
OY=4536478.78
THETA=0.683411946;

# ALBOI_2 START PARAMETERS
# OX=-731119.42
# OY=4536484.78
# THETA=0.503411946;

# LICEU_1 START PARAMETERS
#OX=-730263
#OY=4535623.78
#THETA=-0.72;

# File to open, first argument
XY_original=$1

XY_corrected=XY_correct_odo.txt
LL=LL_odo.txt

# Output file, second argument
KML_file=$2

# Convert from local Cartesian coordinates to global cartesian coordinates
kmlPathCreator $XY_original conv $OX $OY $THETA > $XY_corrected
# Convert from cartesian to gps coordinates
invproj +proj=poly -f "%.12f" $XY_corrected -s > $LL
# Convert from gps coordinates to kml format
kmlPathCreator $LL > $KML_file

rm $XY_corrected $LL
