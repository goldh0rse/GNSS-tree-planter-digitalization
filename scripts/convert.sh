#!/bin/bash

# Check number of input arguments
if [ $# -lt 3 ]; then
	echo "Too few arguments."
	echo "Usage: convert <base.ubx> <rover.ubx> <configfile.conf>"
	exit 1
fi

if [ $# -gt 4  ]; then
	echo "Too many arguments."
	echo "Usage: convert <base.ubx> <rover.ubx> <configfile.conf>"
        exit 1
fi

# Check argument file format
BASE=$1
ROVER=$2
CONFIG=$3

# -od = include Doppler 			[default on]
# -os = include SNR in rinex 			[default on]
# -oi = include iono correction in rinex nav 	[default off]
# -ot = include time correction in rinex nav	[default off]
# -f  = include number of frequencies		[default 5]

CONVBIN_FLAGS='-v 3.03 -od -os -f 2 -r ubx'

shopt -s nocasematch

# If base file exists and is .ubx format
if [[ -f "$BASE" &&  -f "$ROVER" ]]; then
	if [[ $BASE != *.ubx ||  $ROVER != *.ubx ]]; then
		echo "Input files are not .ubx format"
		exit 1
	fi

	# Check if convbin cmd exist
	if [ ! command -v $convbin &> /dev/null ]; then
	    echo "The 'convbin' command could not be found"
			echo "Please run the installer"
	    exit 1
	fi

	# RUN CONVBIN
	convbin $CONVBIN_FLAGS $BASE
	convbin $CONVBIN_FLAGS $ROVER
else
	echo "Input .ubx files does not exist"
	exit 1
fi

if [ ! -f "$CONFIG" ]; then
	echo "Config file does not exist"
	exit 1
fi

if [[ $CONFIG != *.conf ]]; then
	echo "Input config file is not .conf format"
	exit 1
fi

if [ ! command -v $rnx2rtkp &> /dev/null ]; then
		echo "The 'rnx2rtkp' command could not be found"
		echo "Please run the installer"
		exit 1
fi

BASE_OBS=${BASE/UBX/obs}
BASE_NAV=${BASE/UBX/nav}
ROVER_OBS=${ROVER/UBX/obs}
ROVER_POS=${ROVER/UBX/pos}

echo ""
echo "Start post processing..."
# echo $BASE_OBS
# echo $BASE_NAV
# echo $ROVER_OBS
rnx2rtkp -k $CONFIG -o $ROVER_POS $ROVER_OBS $BASE_OBS $BASE_NAV
