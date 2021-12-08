#!/bin/bash

if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root, use sudo"
   exit 1
fi

if [[ $0 != './install_dependencies.sh' ]] ; then
  SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
  cd $SCRIPT_DIR
fi

## Update packages and Upgrade system
echo '###Updating OS...'
sudo apt-get update -y
echo ''

echo '###Installing build-essential'
sudo apt-get install build-essential -y
echo ''

## Python ##
echo '###Installing Python'
sudo apt-get install python3 python3-pip -y
echo ''

## Git ##
echo '###Installing Git..'
sudo apt-get install git -y
echo ''

echo '###Cloning Repository...'
git clone https://github.com/rtklibexplorer/RTKLIB
echo ''

echo '###Installing RTKLIB console applications...'
cd RTKLIB/app/consapp/convbin/gcc
make

if [ $? -eq 0 ] ; then
  echo 'convbin build successful.'
  echo ''
else
  echo 'convbin build failed. Exiting.'
  exit -1
fi

make install
if [ $? -eq 0 ] ; then
  echo 'convbin installation successful.'
  echo ''
else
  echo 'convbin installation failed. Exiting.'
  exit -1
fi
make clean

cd ../../rnx2rtkp/gcc
make

if [ $? -eq 0 ] ; then
  echo 'rnx2rtkp build successful.'
  echo ''
else
  echo 'rnx2rtkp build failed. Exiting.'
  exit -1
fi

make install
if [ $? -eq 0 ] ; then
  echo 'rnx2rtkp installation successful.'
  echo ''
else
  echo 'rnx2rtkp installation failed. Exiting.'
  exit -1
fi
echo 'Installation finished.'
echo ''
echo '###Cleaning up'
cd ../../../../..
echo $PWD
rm -rf RTKLIB
echo 'Finished.'
