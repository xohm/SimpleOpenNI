#!/bin/sh
# --------------------------------------------------------------------------
# install script for linux 64bit
# --------------------------------------------------------------------------
# Processing Wrapper for the OpenNI/Kinect 2.0 library
# http://code.google.com/p/simple-openni
# --------------------------------------------------------------------------
# prog:  Max Rheiner / Interaction Design / zhdk / http://iad.zhdk.ch/
# date:  08/27/2013 (m/d/y)
# ----------------------------------------------------------------------------


# Check if user is root/running with sudo
if [ `whoami` != root ]; then
    echo Please run this script with sudo
    exit
fi

if [ "`uname -s`" != "Darwin" ]; then
    # Install UDEV rules for USB device
    cp ./primesense-usb.rules /etc/udev/rules.d/557-primesense-usb.rules 
fi
