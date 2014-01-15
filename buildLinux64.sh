#!/bin/sh
# --------------------------------------------------------------------------
# buildscript for linux 64bit
# --------------------------------------------------------------------------
# Processing Wrapper for the OpenNI/Kinect 2.0 library
# http://code.google.com/p/simple-openni
# --------------------------------------------------------------------------
# prog:  Max Rheiner / Interaction Design / zhdk / http://iad.zhdk.ch/
# date:  06/11/2011 (m/d/y)
# ----------------------------------------------------------------------------
# Change those vars to the folders you have on your system:
#	-DOPEN_NI_BASE 		= folder of OpenNI headers
#	-DNITE_BASE	 	= folder of Nite headers
#	-DEIGEN3D_INCLUDE 	= folder of Eigen3d headers
#	-DBOOST_ROOT 		= folder of Boost root
#	-DBOOST_LIBRARYDIR 	= folder of Boost library folder
#	-DP5_JAR 		= filepath to your core.jar (Processing)
# ----------------------------------------------------------------------------

# optional, but gives a clean build
rm -r build64

# check if build folder exists
if [ ! -d "build64" ]; then
    mkdir build64
fi

cd ./build64

echo "--- generate cmake ---"
# changes this according to your environment
cmake -DCMAKE_BUILD_TYPE=Release \
	  -DOPEN_NI_BASE=/media/dataDisk/ownDev/libs/openni/git/OpenNI2/ \
          -DOPEN_NI_LIBDIR=./dist/all/SimpleOpenNI/library/linux64/ \
          -DNITE_BASE=/media/dataDisk/ownDev/libs/openni/git/NiTE-Linux-x64-2.2/ \
	  -DEIGEN3D_INCLUDE=/usr/local/include/eigen3/ \
	  -DBOOST_ROOT=/usr/include/boost/ \
	  -DBOOST_LIBRARYDIR=/usr/lib/ \
          -DP5_JAR=/media/dataDisk/ownDev/locApp/processing-2.0.1/core/library/core.jar \
          ..


echo "--- build ---"
# build with 6 threads, verbose is optional, but otherwise you can't see the compiler directives
#make -j 6 VERBOSE=1
make -j6

echo "--- copy ---"
# copy the library
cp SimpleOpenNI.jar ../dist/all/SimpleOpenNI/library
cp libSimpleOpenNI*.so ../dist/all/SimpleOpenNI/library

# copy the doc
cp -r ./doc/* ../dist/all/SimpleOpenNI/documentation/

