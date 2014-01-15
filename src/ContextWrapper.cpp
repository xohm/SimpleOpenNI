/* 
 *----------------------------------------------------------------------------
 * SimpleOpenNI
 * ----------------------------------------------------------------------------
 * Copyright (C) 2011 Max Rheiner / Interaction Design Zhdk
 *
 * This file is part of SimpleOpenNI.
 *
 * SimpleOpenNI is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version (subject to the "Classpath" exception
 * as provided in the LICENSE.txt file that accompanied this code).
 *
 * SimpleOpenNI is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with SimpleOpenNI.  If not, see <http://www.gnu.org/licenses/>.
 * ----------------------------------------------------------------------------
 */

#ifdef WIN32
#define _CRT_SECURE_NO_WARNINGS
#define NOMINMAX	// eigen + windows.h will have conflict without this

#include <direct.h>
#endif

#include <cmath>

// opengl extensions
/*
#include <GL/glew.h>
*/
#include <stdarg.h>

// openni
#include <OpenNI.h>

#include "ContextWrapper.h"

using namespace sOpenNI;
using namespace openni;

#define		SIMPLEOPENNI_VERSION	196		// 1234 = 12.24

// if you plan to debug on linux, don't forget to call the following commands, otherwise you want be able to
// attacht to the shared library
// echo 0 | sudo tee /proc/sys/kernel/yama/ptrace_scope
// http://www.deder.at/wordpress/?p=307

float Colors[][3] =
{
    {0,1,1},
    {0,0,1},
    {0,1,0},
    {1,1,0},
    {1,0,0},
    {1,.5,0},
    {.5,1,0},
    {0,.5,1},
    {.5,0,1},
    {1,1,.5},
    {1,1,1}
};
unsigned int nColors = 10;

///////////////////////////////////////////////////////////////////////////////
// eigen defs

typedef Eigen::Hyperplane<float,3> HyperPlane3d;

///////////////////////////////////////////////////////////////////////////////
// ContextWrapper

bool ContextWrapper::_globalContextFlag = false;
/*
KinectMotors ContextWrapper::_kinectMotors;
*/
int ContextWrapper::_deviceCount=0;
std::vector<class ContextWrapper*> ContextWrapper::_classList;


ContextWrapper::ContextWrapper():
    _pDepthImage(NULL),
    _depthMapRealWorld(NULL),
    _depthImageColorMode(DepthImgMode_Default),
    _irBufSize(0),
    _nodes(Node_None),
    _threadMode(RunMode_Default),
    _threadRun(false),
    _deviceIndex(0),
    _firstTimeUpdate(true),
    _initFlag(false),
    _generatingFlag(false),
    _userCoordsysFlag(false),
    _depthSensorInfo(NULL),
    _imageSensorInfo(NULL),
    _irSensorInfo(NULL),
    _streams(NULL),
    _streamCount(0),
    _player(NULL),
    _playerRepeat(true),
    _playerPlay(false),
    _pUserImage(NULL),
    _niteFlag(false),
    _mirrorFlag(false),
    _depthMapBuffer(NULL),
    _imgBuffer(NULL),
    _pIrImage(NULL)
{
    // still not working
    //_kinectMotors.open();

    _depthImageColor[0] = 1.0f;
    _depthImageColor[1] = 1.0f;
    _depthImageColor[2] = 1.0f;

    _depthMapOutputMode.setResolution(640,480);
    _depthMapOutputMode.setFps(30);

    _imageMapOutputMode.setResolution(640,480);
    _imageMapOutputMode.setFps(30);
    _imageMapOutputMode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);

    _irMapOutputMode.setResolution(640,480);
    _irMapOutputMode.setFps(30);
    //_irMapOutputMode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);

    /*
    _sceneMapOutputMode.nXRes = 640;
    _sceneMapOutputMode.nYRes = 480;
    _sceneMapOutputMode.nFPS = 30;
    */

    // calc gamma map
    for(int i=0; i< (MAX_DEPTH /1); ++i)
    {
        float v = i/(float)(MAX_DEPTH/1);
        v = std::pow(v, 3)* 6;
        _pDepthGamma[i] = v*6*256;
    }

    // set timeStamps
    _depthMapTimeStamp		= 0;
    _depthImageTimeStamp	= 0;
    _depthRealWorldTimeStamp= 0;

    _imageTimeStamp         = 0;
    _irTimeStamp            = 0;
    _sceneTimeStamp         = 0;
    _userTimeStamp          = 0;
    _gestureTimeStamp       = 0;
    _handTimeStamp         = 0;

    _updateTimeStamp		= 1;	// set to 1 for the first update
    _updateSubTimeStamp		= 1;
}

ContextWrapper::~ContextWrapper()
{
    close();
}

void ContextWrapper::chdir(const char* dir)
{
#ifdef WIN32
    int ret = _chdir(dir);
#else
    int ret = ::chdir(dir);
#endif
	/*
    char * pPath;
    pPath = getenv ("OPENNI2_REDIST");
    if (pPath!=NULL)
        std::cout << "OPENNI2_REDIST read 1= " << pPath << std::endl;

    char buf[1024];
    if(::getcwd(buf, 1024) != NULL)
    {
        std::cout << "pwd before= " << buf << std::endl;
    }

    std::cout << "dir = " << dir << std::endl;
    ::chdir(dir);

    if(::getcwd(buf, 1024) != NULL)
    {
        std::cout << "pwd after= " << buf << std::endl;
    }

    char envVar[1024];
    sprintf(envVar,"OPENNI2_REDIST=%s",dir);
    std::cout << "set envVar = " << envVar << std::endl;
    putenv(envVar);

    pPath = getenv ("OPENNI2_REDIST");
    if (pPath!=NULL)
        std::cout << "OPENNI2_REDIST read 2= " << pPath << std::endl;
*/
}

std::string ContextWrapper::getcwd()
{
    char buf[1024];
    if(::getcwd(buf, 1024) != NULL)
        return std::string(buf);
    else
        return std::string("");
}

void ContextWrapper::close()
{
    if(!_initFlag)
        return;

    if(_threadMode == RunMode_MultiThreaded)
    {
        _threadRun = false;
        if(_thread.get() != NULL)
        {
            // wait till thread is done
            _thread->join();

            // delete the thread
            _thread.reset();
        }
    }

    _initFlag = false;
    _generatingFlag = false;

    // cleanup streams

    // end the recording
    if(_recorder.isValid())
    {
        std::cout << "end record" << std::endl;
        _recorder.stop();
        _recorder.destroy();
    }

    // depth
    if(_depthStream.isValid())
        _depthStream.destroy();

    if(_depthMapRealWorld != NULL)
    {
        delete []_depthMapRealWorld;
        _depthMapRealWorld = NULL;
    }

    if(_pDepthImage != NULL)
    {
        delete []_pDepthImage;
        _pDepthImage = NULL;
    }

    if(_depthMapBuffer)
    {
        delete []_depthMapBuffer;
        _depthMapBuffer = NULL;
    }

    // image
    if(_imageStream.isValid())
        _imageStream.destroy();

    // ir
    if(_irStream.isValid())
        _irStream.destroy();

    if(_imgBuffer)
    {
        delete []_imgBuffer;
        _imgBuffer = NULL;
    }

    if(_pIrImage)
    {
        delete []_pIrImage;
        _pIrImage = NULL;
    }

    // user
    if(_userTracker.isValid())
    {
        // crashes with the libFreenect driver, so kill the hole think later with
        // nite::NiTE::shutdown();
        _userTracker.destroy();
    }

    if(_pUserImage)
    {
        delete []_pUserImage;
        _pUserImage = NULL;
    }

    // hand
    if(_handTracker.isValid())
        _handTracker.destroy();

    _handIdTempList.clear();
    _handLastState.clear();
    _gestureIdTempList.clear();
    _gestureLastState.clear();

    // close the device
    if(_device.isValid())
        _device.close();


    _classList.clear();

/*
    if(_depthStream.isValid)
        free(_depthMap);
    if(_pDepthImage)
        free(_pDepthImage);
    if(_pIrImage)
        free(_pIrImage);
    if(_pSceneImage)
        free(_pSceneImage);
    if(_depthMapRealWorld)
        free(_depthMapRealWorld),

    _pDepthImage= NULL;
    _pIrImage	= NULL;
    _pSceneImage= NULL;
    _depthMapRealWorld= NULL;

    _depthMapTimeStamp		= 0;
    _depthImageTimeStamp	= 0;
    _depthRealWorldTimeStamp	= 0;

    _imageTimeStamp		= 0;
    _irTimeStamp		= 0;
    _sceneTimeStamp		= 0;
    _userTimeStamp		= 0;
    _gestureTimeStamp		= 0;
    _handTimeStamp		= 0;

    _updateTimeStamp		= 1;	// set to 1 for the first update
    _updateSubTimeStamp		= 1;

    // shutdown the context
    _globalContext.Shutdown();
*/

    _playerPlay = false;

    nite::NiTE::shutdown();
    _niteFlag = false;
    openni::OpenNI::shutdown();
}

int ContextWrapper::version() 
{ 
    return SIMPLEOPENNI_VERSION;
}

void ContextWrapper::logOut(int msgType,const char* msg,...)
{
    char strBuffer[STRING_BUFFER];

    switch(msgType)
    {
    case MsgNode_Error:
        std::cout << "SimpleOpenNI Error: ";
        break;
    case MsgNode_Deprecated:
        std::cout << "SimpleOpenNI Deprecated: ";
        break;
    case MsgNode_Info:
    default:
        std::cout << "SimpleOpenNI Info: ";
        break;
    };

    va_list args;
    va_start(args, msg);
    vsnprintf(strBuffer,STRING_BUFFER,msg, args);
    va_end(args);

    std::cout << strBuffer << std::endl;
}

void ContextWrapper::logDeprecated(int msgType,const char* oldFunc,const char* newFunc)
{
     logOut(MsgNode_Deprecated,"%s is deprecated, please use %s.",oldFunc,newFunc);
}

bool ContextWrapper::initContext()
{
    if(_globalContextFlag == false)
    {   // create context
        std::cout << "SimpleOpenNI Version " << (SIMPLEOPENNI_VERSION / 100) << "." <<  (SIMPLEOPENNI_VERSION % 100) << std::endl;

        openni::Status rc = openni::OpenNI::initialize();
        printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());

        if(rc != openni::STATUS_OK)
            // error;
            _globalContextFlag = false;
        else
        {
            _globalContextFlag = true;

            // check the list of all devices
            openni::Array<openni::DeviceInfo> deviceInfoList;
            openni::OpenNI::enumerateDevices(&deviceInfoList);

            _deviceCount = deviceInfoList.getSize();

             // init NITE
            nite::Status niteRc = nite::NiTE::initialize();
            if(niteRc != nite::STATUS_OK)
                // error;
                logOut(MsgNode_Error,"ContextWrapper::initContext / nite::NiTE::initialize\n");

        }

        return _globalContextFlag;
    }
    else
        return _globalContextFlag;
}

/*
bool ContextWrapper::getNodeInfo(int nodeType,int index,
                                 xn::NodeInfoList*   list,
                                 xn::NodeInfo* pNodeInfo,
                                 int offset)
{
    int                 i=0;
    XnStatus            rc;

    index *= offset;

    rc = _globalContext.EnumerateProductionTrees(nodeType, NULL,*list);
    //logOut(MsgNode_Info,"Device List:");
    for(xn::NodeInfoList::Iterator iter = list->Begin(); iter != list->End(); ++iter)
    {
        xn::NodeInfo nodeInfo = *iter;

        if(strcmp(nodeInfo.GetInstanceName(),"") == 0)
        {   // prepared one
            if(i == index)
            {   // found, copy the InfoNode
                *pNodeInfo = nodeInfo;
                return true;
            }
            else
                i++;
        }
    }
    return false;
}
*/

bool ContextWrapper::init(int runMode)
{
    _threadMode = runMode;

    if (initContext() == false)
    {
        logOut(MsgNode_Error,"ContextWrapper::init\n");
        return false;
    }

    // open the default device
    if(_device.open(ANY_DEVICE) != openni::STATUS_OK)
    {
        logOut(MsgNode_Error,"Can't open device:\t%s\n",openni::OpenNI::getExtendedError());
        return false;
    }

    /*
    // init NITE
    if(_niteFlag == false)
    {
        nite::Status niteRc = nite::NiTE::initialize();
        if(niteRc != nite::STATUS_OK)
            // error;
            logOut(MsgNode_Error,"ContextWrapper::initContext / nite::NiTE::initialize\n");
        else
            _niteFlag = true;
    }
*/

    _initFlag = true;

    // read out all infos per sensor
    _depthSensorInfo    = _device.getSensorInfo(openni::SENSOR_DEPTH);
    _imageSensorInfo    = _device.getSensorInfo(openni::SENSOR_COLOR);
    _irSensorInfo       = _device.getSensorInfo(openni::SENSOR_IR);

    // add to list
    _classList.push_back(this);

    return true;
}

bool ContextWrapper::init(const char* record,int runMode)
{
    _threadMode = runMode;

    if (initContext() == false)
    {
        logOut(MsgNode_Error,"ContextWrapper::init\n");
        return false;
    }

    std::cout << "record: " << record << std::endl;


    // open the default device
    if(_device.open(record) != openni::STATUS_OK)
    {
        logOut(MsgNode_Error,"Can't open device:\t%s\n",openni::OpenNI::getExtendedError());
        return false;
    }

    _player = _device.getPlaybackControl();

    _initFlag = true;
    _nodes |= Node_Player;

    // read out all infos per sensor
    createDepth(false);
    createRgb(false);
    createIr(false);

    _playerPlayFrame = 0;
    _playerPlay = true;

    // add to list
    _classList.push_back(this);

    return true;
}

bool ContextWrapper::init(int deviceIndex,int runMode)
{
  //  logOut(MsgNode_Error,"start index init");
    _threadMode = runMode;
    _deviceIndex = -1;

    if (initContext() == false)
    {
        logOut(MsgNode_Error,"ContextWrapper::init\n");
        return false;
    }

    openni::Array<openni::DeviceInfo> deviceInfoList;
    openni::OpenNI::enumerateDevices(&deviceInfoList);
    if(deviceIndex >=0 && deviceIndex < deviceInfoList.getSize())
    {
        // open the default device
        if(_device.open(deviceInfoList[deviceIndex].getUri()) != openni::STATUS_OK)
        {
            logOut(MsgNode_Error,"Can't open device:\t%s\n",openni::OpenNI::getExtendedError());
            return false;
        }
/*
        // init NITE
        if(_niteFlag == false)
        {
            nite::Status niteRc = nite::NiTE::initialize();
            if(niteRc != nite::STATUS_OK)
                // error;
                logOut(MsgNode_Error,"ContextWrapper::initContext / nite::NiTE::initialize\n");
            else
                _niteFlag = true;
        }
*/
        _initFlag = true;

        // read out all infos per sensor
        _depthSensorInfo = _device.getSensorInfo(openni::SENSOR_DEPTH);
        _imageSensorInfo = _device.getSensorInfo(openni::SENSOR_COLOR);
        _irSensorInfo = _device.getSensorInfo(openni::SENSOR_IR);

        _deviceIndex = deviceIndex;

        // add to list
        _classList.push_back(this);
        return true;
    }

    return false;
}


int ContextWrapper::nodes()
{
    return _nodes;
}

int ContextWrapper::deviceCount()
{
    return _deviceCount;
}


int ContextWrapper::deviceNames(std::vector<std::string>* nodeNames)
{
    if(!_globalContextFlag)
        return 0;

    openni::Array<openni::DeviceInfo> deviceInfoList;
    openni::OpenNI::enumerateDevices(&deviceInfoList);

    if(deviceInfoList.getSize() <= 0)
        return 0;

    nodeNames->clear();
    for(int i=0;i < deviceInfoList.getSize();i++)
    {
        std::string str = deviceInfoList[i].getName();
        /*
        str += " / ";
        str += deviceInfoList[i].getUri();
        */
        nodeNames->push_back(str);
    }

    return nodeNames->size();
}

#ifdef OPENNI2_DEBUG

#include <PS1080.h>
#include <cstring>
#include <fstream>
#include <iostream>

void saveDataStream(const char* name,unsigned char* data,int dataSize)
{
    std::ofstream myfile;
    myfile.open (name, std::ios::out | std::ios::binary | std::ios::trunc);
    myfile.write((const char*)data,dataSize);
    myfile.close();
}

void printAllProperties(openni::VideoStream* stream)
{
    unsigned char   data[50000];
    int             dataSize;
    float           fValue;
    int             iValue;

    std::cout << "printAllProperties--------------------------" << std::endl;

    // ONI_STREAM_PROPERTY_HORIZONTAL_FOV
    dataSize = sizeof(float);
    stream->getProperty(ONI_STREAM_PROPERTY_HORIZONTAL_FOV,(void*)&fValue,&dataSize);
    std::cout << "ONI_STREAM_PROPERTY_HORIZONTAL_FOV = " << fValue << std::endl;

    // ONI_STREAM_PROPERTY_VERTICAL_FOV
    dataSize = sizeof(float);
    stream->getProperty(ONI_STREAM_PROPERTY_VERTICAL_FOV,(void*)&fValue,&dataSize);
    std::cout << "ONI_STREAM_PROPERTY_VERTICAL_FOV = " << fValue << std::endl;

    // ONI_STREAM_PROPERTY_MAX_VALUE
    dataSize = sizeof(int);
    stream->getProperty(ONI_STREAM_PROPERTY_MAX_VALUE,(void*)&iValue,&dataSize);
    std::cout << "ONI_STREAM_PROPERTY_MAX_VALUE = " << iValue << std::endl;



    // --------------------------
    unsigned long long  ullValue;
    XnDepthAGCBin       depthAGCBin;
    double              dValue;
    XnPixelRegistration pixelRegistration;


    // XN_STREAM_PROPERTY_CLOSE_RANGE
    dataSize = sizeof(unsigned long long);
    stream->getProperty(XN_STREAM_PROPERTY_CLOSE_RANGE,(void*)&ullValue,&dataSize);
    std::cout << "XN_STREAM_PROPERTY_CLOSE_RANGE = " << ullValue << std::endl;

    // XN_STREAM_PROPERTY_PIXEL_REGISTRATION
    dataSize = sizeof(XnPixelRegistration);
    stream->getProperty(XN_STREAM_PROPERTY_PIXEL_REGISTRATION,(void*)&pixelRegistration,&dataSize);
    std::cout << "XN_STREAM_PROPERTY_PIXEL_REGISTRATION.nDepthX = " << pixelRegistration.nDepthX << std::endl;
    std::cout << "XN_STREAM_PROPERTY_PIXEL_REGISTRATION.nDepthY = " << pixelRegistration.nDepthY << std::endl;
    std::cout << "XN_STREAM_PROPERTY_PIXEL_REGISTRATION.nDepthValue = " << pixelRegistration.nDepthValue << std::endl;
    std::cout << "XN_STREAM_PROPERTY_PIXEL_REGISTRATION.nImageXRes = " << pixelRegistration.nImageXRes << std::endl;
    std::cout << "XN_STREAM_PROPERTY_PIXEL_REGISTRATION.nImageYRes = " << pixelRegistration.nImageYRes << std::endl;
    std::cout << "XN_STREAM_PROPERTY_PIXEL_REGISTRATION.nImageX = " << pixelRegistration.nImageX << std::endl;
    std::cout << "XN_STREAM_PROPERTY_PIXEL_REGISTRATION.nImageY = " << pixelRegistration.nImageY << std::endl;

    // XN_STREAM_PROPERTY_WHITE_BALANCE_ENABLED
    dataSize = sizeof(unsigned long long);
    stream->getProperty(XN_STREAM_PROPERTY_WHITE_BALANCE_ENABLED,(void*)&ullValue,&dataSize);
    std::cout << "XN_STREAM_PROPERTY_WHITE_BALANCE_ENABLED = " << ullValue << std::endl;

    // XN_STREAM_PROPERTY_GAIN
    dataSize = sizeof(unsigned long long);
    stream->getProperty(XN_STREAM_PROPERTY_GAIN,(void*)&ullValue,&dataSize);
    std::cout << "XN_STREAM_PROPERTY_GAIN = " << ullValue << std::endl;

    // XN_STREAM_PROPERTY_HOLE_FILTER
    dataSize = sizeof(unsigned long long);
    stream->getProperty(XN_STREAM_PROPERTY_HOLE_FILTER,(void*)&ullValue,&dataSize);
    std::cout << "XN_STREAM_PROPERTY_HOLE_FILTER = " << ullValue << std::endl;

    // XN_STREAM_PROPERTY_REGISTRATION_TYPE
    dataSize = sizeof(unsigned long long);
    stream->getProperty(XN_STREAM_PROPERTY_REGISTRATION_TYPE,(void*)&ullValue,&dataSize);
    std::cout << "XN_STREAM_PROPERTY_REGISTRATION_TYPE = " << ullValue << std::endl;

    // XN_STREAM_PROPERTY_AGC_BIN
    dataSize = sizeof(XnDepthAGCBin);
    stream->getProperty(XN_STREAM_PROPERTY_AGC_BIN,(void*)&depthAGCBin,&dataSize);
    std::cout << "XN_STREAM_PROPERTY_AGC_BIN.nBin = " << depthAGCBin.nBin << std::endl;
    std::cout << "XN_STREAM_PROPERTY_AGC_BIN.nMin = " << depthAGCBin.nMin << std::endl;
    std::cout << "XN_STREAM_PROPERTY_AGC_BIN.nMax = " << depthAGCBin.nMax << std::endl;

    // XN_STREAM_PROPERTY_CONST_SHIFT
    dataSize = sizeof(unsigned long long);
    stream->getProperty(XN_STREAM_PROPERTY_CONST_SHIFT,(void*)&ullValue,&dataSize);
    std::cout << "XN_STREAM_PROPERTY_CONST_SHIFT = " << ullValue << std::endl;

    // XN_STREAM_PROPERTY_PIXEL_SIZE_FACTOR
    dataSize = sizeof(unsigned long long);
    stream->getProperty(XN_STREAM_PROPERTY_PIXEL_SIZE_FACTOR,(void*)&ullValue,&dataSize);
    std::cout << "XN_STREAM_PROPERTY_PIXEL_SIZE_FACTOR = " << ullValue << std::endl;

    // XN_STREAM_PROPERTY_MAX_SHIFT
    dataSize = sizeof(unsigned long long);
    stream->getProperty(XN_STREAM_PROPERTY_MAX_SHIFT,(void*)&ullValue,&dataSize);
    std::cout << "XN_STREAM_PROPERTY_MAX_SHIFT = " << ullValue << std::endl;

    // XN_STREAM_PROPERTY_PARAM_COEFF
    dataSize = sizeof(unsigned long long);
    stream->getProperty(XN_STREAM_PROPERTY_PARAM_COEFF,(void*)&ullValue,&dataSize);
    std::cout << "XN_STREAM_PROPERTY_PARAM_COEFF = " << ullValue << std::endl;

    // XN_STREAM_PROPERTY_SHIFT_SCALE
    dataSize = sizeof(unsigned long long);
    stream->getProperty(XN_STREAM_PROPERTY_SHIFT_SCALE,(void*)&ullValue,&dataSize);
    std::cout << "XN_STREAM_PROPERTY_SHIFT_SCALE = " << ullValue << std::endl;

    // XN_STREAM_PROPERTY_ZERO_PLANE_DISTANCE
    dataSize = sizeof(unsigned long long);
    stream->getProperty(XN_STREAM_PROPERTY_ZERO_PLANE_DISTANCE,(void*)&ullValue,&dataSize);
    std::cout << "XN_STREAM_PROPERTY_ZERO_PLANE_DISTANCE = " << ullValue << std::endl;

    // XN_STREAM_PROPERTY_ZERO_PLANE_PIXEL_SIZE
    dataSize = sizeof(double);
    stream->getProperty(XN_STREAM_PROPERTY_ZERO_PLANE_PIXEL_SIZE,(void*)&dValue,&dataSize);
    std::cout << "XN_STREAM_PROPERTY_ZERO_PLANE_PIXEL_SIZE = " << dValue << std::endl;

    // XN_STREAM_PROPERTY_EMITTER_DCMOS_DISTANCE
    dataSize = sizeof(double);
    stream->getProperty(XN_STREAM_PROPERTY_EMITTER_DCMOS_DISTANCE,(void*)&dValue,&dataSize);
    std::cout << "XN_STREAM_PROPERTY_EMITTER_DCMOS_DISTANCE = " << dValue << std::endl;

    // XN_STREAM_PROPERTY_DCMOS_RCMOS_DISTANCE
    dataSize = sizeof(double);
    stream->getProperty(XN_STREAM_PROPERTY_DCMOS_RCMOS_DISTANCE,(void*)&dValue,&dataSize);
    std::cout << "XN_STREAM_PROPERTY_DCMOS_RCMOS_DISTANCE = " << dValue << std::endl;

    //XN_STREAM_PROPERTY_S2D_TABLE
    dataSize = 4096;
    stream->getProperty(XN_STREAM_PROPERTY_S2D_TABLE,(void*)data,&dataSize);
    std::cout << "XN_STREAM_PROPERTY_S2D_TABLE safed ! size=" << dataSize<< std::endl;
    saveDataStream("S2D_table.bin",data,dataSize);

    //XN_STREAM_PROPERTY_D2S_TABLE
    dataSize = 20002;
    stream->getProperty(XN_STREAM_PROPERTY_D2S_TABLE,(void*)data,&dataSize);
    std::cout << "XN_STREAM_PROPERTY_D2S_TABLE safed ! size=" << dataSize << std::endl;
    saveDataStream("D2S_table.bin",data,dataSize);

    // XN_STREAM_PROPERTY_DEPTH_SENSOR_CALIBRATION_INFO
    dataSize = 50000;
    stream->getProperty(XN_STREAM_PROPERTY_DEPTH_SENSOR_CALIBRATION_INFO,(void*)data,&dataSize);
    std::cout << "XN_STREAM_PROPERTY_DEPTH_SENSOR_CALIBRATION_INFO safed ! size=" << dataSize << std::endl;
    saveDataStream("Calib_table.bin",data,dataSize);

    // XN_STREAM_PROPERTY_GMC_MODE
    bool bValue;
    dataSize = sizeof(bool);
    stream->getProperty(XN_STREAM_PROPERTY_GMC_MODE,(void*)&bValue,&dataSize);
    std::cout << "XN_STREAM_PROPERTY_GMC_MODE = " << bValue << std::endl;

    // XN_STREAM_PROPERTY_GMC_DEBUG
    dataSize = sizeof(bool);
    stream->getProperty(XN_STREAM_PROPERTY_GMC_DEBUG,(void*)&bValue,&dataSize);
    std::cout << "XN_STREAM_PROPERTY_GMC_DEBUG = " << bValue << std::endl;

    // XN_STREAM_PROPERTY_WAVELENGTH_CORRECTION
    dataSize = sizeof(bool);
    stream->getProperty(XN_STREAM_PROPERTY_WAVELENGTH_CORRECTION,(void*)&bValue,&dataSize);
    std::cout << "XN_STREAM_PROPERTY_WAVELENGTH_CORRECTION = " << bValue << std::endl;

    // XN_STREAM_PROPERTY_WAVELENGTH_CORRECTION_DEBUG
    dataSize = sizeof(bool);
    stream->getProperty(XN_STREAM_PROPERTY_WAVELENGTH_CORRECTION_DEBUG,(void*)&bValue,&dataSize);
    std::cout << "XN_STREAM_PROPERTY_WAVELENGTH_CORRECTION_DEBUG = " << bValue << std::endl;


}
#endif // OPENNI2_DEBUG

bool ContextWrapper::createDepth(bool force)
{
    if(!_initFlag)
        return false; 

    if(_depthStream.create(_device, openni::SENSOR_DEPTH) != openni::STATUS_OK)
        return false;

    // find default mode
    if(force && _depthSensorInfo != NULL)
    {
        if(_depthSensorInfo->getSupportedVideoModes().getSize() > 0)
        {
            for(int i=0;i < _depthSensorInfo->getSupportedVideoModes().getSize();i++)
            {
                VideoMode videoMode = _depthSensorInfo->getSupportedVideoModes()[i];

                if(videoMode.getResolutionX() == _depthMapOutputMode.getResolutionX() &&
                   videoMode.getResolutionY() == _depthMapOutputMode.getResolutionY() &&
                   videoMode.getFps() == _depthMapOutputMode.getFps())
                {   // found
                    if(_depthStream.setVideoMode(videoMode) != openni::STATUS_OK)
                    {
                        logOut(MsgNode_Error,"createDepth: can't set videoMode!");
                        return false;
                    }
                    else
                        break;
                }
            }
        }
    }
    _depthStream.start();
    _depthStream.setMirroringEnabled(_mirrorFlag);

    _depthVideoMode = _depthStream.getVideoMode();

    _depthBufSize       = _depthVideoMode.getResolutionX() * _depthVideoMode.getResolutionY();
    _depthMapBuffer     = new int[_depthBufSize];
    memset(_depthMapBuffer,0,sizeof(int) * _depthBufSize);

    _pDepthImage        = new int[_depthBufSize];
    _depthMapRealWorld  = new float[_depthBufSize * 3];

    _nodes |= Node_Depth;

    //printAllProperties(&_depthStream);

    return true;
}

bool ContextWrapper::enableDepth()
{
    if(!_initFlag)
    {
        printf("SimpleOpenNI not initialised\n");
        return false;
    }
    else if(_depthStream.isValid())
        // already init.
        return true;

    return createDepth();
}

bool ContextWrapper::enableDepth(int width,int height,int fps)
{
    // set default data
    _depthMapOutputMode.setResolution(width,height);
    _depthMapOutputMode.setFps(fps);

    return ContextWrapper::enableDepth();
}

bool ContextWrapper::createRgb(bool force)
{
    if(!_initFlag)
        return false;

    if(_imageStream.create(_device, openni::SENSOR_COLOR) != openni::STATUS_OK)
        return false;

    // find default mode
    if(force && _imageSensorInfo != NULL)
    {
        if(_imageSensorInfo->getSupportedVideoModes().getSize() > 0)
        {
            for(int i=0;i < _imageSensorInfo->getSupportedVideoModes().getSize();i++)
            {
                VideoMode videoMode = _imageSensorInfo->getSupportedVideoModes()[i];

                if(videoMode.getResolutionX() == _imageMapOutputMode.getResolutionX() &&
                   videoMode.getResolutionY() == _imageMapOutputMode.getResolutionY() &&
                   videoMode.getPixelFormat() == _imageMapOutputMode.getPixelFormat() &&
                   videoMode.getFps() == _imageMapOutputMode.getFps())
                {   // found
                    if(_imageStream.setVideoMode(videoMode) != openni::STATUS_OK)
                    {
                        logOut(MsgNode_Error,"createRgb: can't set videoMode!");
                        return false;
                    }
                    else
                        break;
                }
            }
        }
    }

    _imageStream.start();
    _imageStream.setMirroringEnabled(_mirrorFlag);

    _imageVideoMode     = _imageStream.getVideoMode();
    _imgBufSize         = _imageVideoMode.getResolutionX() * _imageVideoMode.getResolutionY();
    _imgBuffer          = new int[_imgBufSize];
    memset(_imgBuffer,0,sizeof(int) * _imgBufSize);

    _nodes |= Node_Image;
    return true;
}

bool ContextWrapper::enableRGB()
{
    if(!_initFlag)
    {
        printf("SimpleOpenNI not initialised\n");
        return false;
    }
    else if(_imageStream.isValid())
        // already init.
        return true;

    return createRgb();
}

bool ContextWrapper::enableRGB(int width,int height,int fps)
{
    // set default data
    _imageMapOutputMode.setResolution(width,height);
    _imageMapOutputMode.setFps(fps);

    return ContextWrapper::enableRGB();
}

bool ContextWrapper::createIr(bool force)
{
    if(!_initFlag)
        return false;

    if(_irStream.create(_device, openni::SENSOR_IR) != openni::STATUS_OK)
        return false;

    // find default mode
    if(force && _irSensorInfo != NULL)
    {
        if(_irSensorInfo->getSupportedVideoModes().getSize() > 0)
        {
            for(int i=0;i < _irSensorInfo->getSupportedVideoModes().getSize();i++)
            {
                VideoMode videoMode = _irSensorInfo->getSupportedVideoModes()[i];

                if(videoMode.getResolutionX() == _irMapOutputMode.getResolutionX() &&
                   videoMode.getResolutionY() == _irMapOutputMode.getResolutionY() &&
                  // videoMode.getPixelFormat() == _irMapOutputMode.getPixelFormat() &&
                   videoMode.getFps() == _irMapOutputMode.getFps())
                {   // found
                    if(_irStream.setVideoMode(videoMode) != openni::STATUS_OK)
                    {
                        logOut(MsgNode_Error,"createDepth: can't set videoMode!");
                        return false;
                    }
                    else
                           break;
                }
            }
        }
    }
    _irStream.start();
    _irStream.setMirroringEnabled(_mirrorFlag);

    _irVideoMode = _irStream.getVideoMode();

    _irBufSize          = _irVideoMode.getResolutionX() * _irVideoMode.getResolutionY();
    _pIrImage           = new int[_irBufSize];

    _nodes |= Node_Ir;

    return true;
}

bool ContextWrapper::enableIR()
{
    if(!_initFlag)
    {
        printf("SimpleOpenNI not initialised\n");
        return false;
    }
    else if(_irStream.isValid())
        // already init.
        return true;

    return createIr();
}

bool ContextWrapper::enableIR(int width,int height,int fps)
{
    // set default data
    _irMapOutputMode.setResolution(width,height);
    _irMapOutputMode.setFps(fps);
    _irMapOutputMode.setPixelFormat(openni::PIXEL_FORMAT_GRAY8);

    return ContextWrapper::enableIR();
}

/*
bool ContextWrapper::createScene(bool force)
{
    if(!_initFlag)
        return false;

    if(force == false)
    {
        _rc = _globalContext.FindExistingNode(XN_NODE_TYPE_SCENE, _sceneAnalyzer);
        if(_rc != XN_STATUS_OK)
            return false;

        if(_sceneAnalyzer.IsValid())
        {
            _sceneAnalyzer.GetMetaData(_sceneMD);

            _sceneBufSize = _sceneMD.XRes() * _sceneMD.YRes();
            _pSceneImage  = (XnRGB24Pixel*)malloc( _sceneBufSize * sizeof(XnRGB24Pixel));

            _nodes |= Node_Scene;
            return true;
        }
        else
            return false;
    }
    else
    {

        xn::NodeInfoList    list;
        xn::NodeInfo        sceneNodeInfo(NULL);

        if(getNodeInfo(XN_NODE_TYPE_SCENE,_deviceIndex,
                       &list,&sceneNodeInfo,
                       1))
        {
            _rc = _globalContext.CreateProductionTree(sceneNodeInfo,_sceneAnalyzer);
            if(_rc == XN_STATUS_OK)
            {
                if(_sceneAnalyzer.IsValid())
                {
                    //std::cout << "sceneValdi:  " << _deviceIndex << std::endl;

                    _rc = _sceneAnalyzer.SetMapOutputMode(_sceneMapOutputMode);
                    _sceneAnalyzer.GetMetaData(_sceneMD);

                    _sceneBufSize = _sceneMD.XRes() * _sceneMD.YRes();
                    _pSceneImage  = (XnRGB24Pixel*)malloc( _sceneBufSize * sizeof(XnRGB24Pixel));

                    _nodes |= Node_Scene;
                    return true;
                }
            }
        }
        return false;
    }

}

bool ContextWrapper::enableScene()
{
    if(!_initFlag)
    {
        printf("SimpleOpenNI not initialised\n");
        return false;
    }
    else if(_sceneAnalyzer.IsValid())
        // already init.
        return true;

    return createScene();
}


bool ContextWrapper::enableScene(int width,int height,int fps)
{
    // set default data
    _sceneMapOutputMode.nXRes = width;
    _sceneMapOutputMode.nYRes = height;
    _sceneMapOutputMode.nFPS = fps;

    return ContextWrapper::enableScene();
}
*/

bool ContextWrapper::createUser(bool force)
{
    if(!_initFlag)
        return false;

    if (_userTracker.create(&_device) != nite::STATUS_OK)
        return false;

    // depthStream has to be active
    if(_depthStream.isValid() == false)
        return false;

    _userVideoMode = _depthStream.getVideoMode();

    _userBufSize    = _userVideoMode.getResolutionX() * _userVideoMode.getResolutionY();
    _pUserImage     = new int[_userBufSize];

    _nodes |= Node_User;

    return true;

}

bool ContextWrapper::enableUser()
{
    if(!_initFlag)
    {
        printf("SimpleOpenNI not initialised\n");
        return false;
    }
    else if(_userTracker.isValid())
        // already init.
        return true;

    return createUser();
}


bool ContextWrapper::createHand(bool force)
{
    if(!_initFlag)
        return false;

    if (_handTracker.create(&_device) != nite::STATUS_OK)
        return false;

    // print out all properties
    //printAllProperties(&_depthStream);

    _nodes |= Node_Hand;
    return true;
}

// hand
bool ContextWrapper::enableHand()
{
    if(!_initFlag)
    {
       // logOut(MsgNode_Error,"enableHand: context is not initialized!");
        return false;
    }
    else if(_handTracker.isValid())
        // already init.
        return true;

    return createHand();
}

int ContextWrapper::startTrackingHand(float* pos)
{	
    if(!_handTracker.isValid())
        return -1;

    nite::HandId handId;
    _handTracker.startHandTracking(nite::Point3f(pos[0],pos[1],pos[2]) , &handId);
    return handId;
}

void ContextWrapper::stopTrackingHand(int handId)
{	
    if(!_handTracker.isValid())
        return;

    _handTracker.stopHandTracking((nite::HandId)handId);
}

void ContextWrapper::stopTrackingAllHand()
{	
    if(!_handTracker.isValid())
        return;

    const nite::Array<nite::HandData>& hands = _handRefFrame.getHands();
    for (int i = 0; i < hands.getSize(); ++i)
        _handTracker.stopHandTracking(hands[i].getId());
}

void ContextWrapper::setSmoothingHand(float smoothingFactor)
{
    if(!_handTracker.isValid())
        return;

    _handTracker.setSmoothingFactor(smoothingFactor);
}

float ContextWrapper::getSmoothingHand()
{
    if(!_handTracker.isValid())
        return 0.0f;

    return _handTracker.getSmoothingFactor();
}

void ContextWrapper::startGesture(int gesture)
{
    if(!_handTracker.isValid())
        return;

    _handTracker.startGestureDetection((nite::GestureType)gesture);
}

void ContextWrapper::endGesture(int gesture)
{
    if(!_handTracker.isValid())
        return;

    _handTracker.stopGestureDetection((nite::GestureType)gesture);
}


///////////////////////////////////////////////////////////////////////////////
// recorder

bool ContextWrapper::enableRecorder(const char* filePath)
{
    if(!_initFlag)
    {
        logOut(MsgNode_Error,"enableRecorder: context is not initialized!");
        return false;
    }
    else if(_recorder.isValid())
        // already init.
        return true;

    _rc = _recorder.create(filePath);
    if(_rc != openni::STATUS_OK)
    {
       logOut(MsgNode_Error,"enableRecorder: could not create recorder!");
       return false;
    }

    _nodes |= Node_Recorder;
    return true;
}

bool ContextWrapper::addNodeToRecording(int nodeType,bool lossy)
{
    if(!_initFlag)
    {
        logOut(MsgNode_Error,"enableRecorder: context is not initialized!");
        return false;
    }
    else if(!_recorder.isValid())
    {
        logOut(MsgNode_Error,"addNodeToRecording: recording is not enabled");
        return false;
    }

    switch(nodeType)
    {
    case Node_Depth:
        if(_depthStream.isValid())
        {
            _rc = _recorder.attach(_depthStream,lossy);
            return(_rc == openni::STATUS_OK);
        }
        break;
    case Node_Image:
        if(_imageStream.isValid())
        {
            _rc = _recorder.attach(_imageStream,lossy);
            return(_rc == openni::STATUS_OK);
        }
        break;
    case Node_Ir:
        if(_irStream.isValid())
        {
            _rc = _recorder.attach(_irStream,lossy);
            return(_rc == openni::STATUS_OK);
        }
        break;

    }
    return false;
}

bool ContextWrapper::openFileRecording(const char* filePath)
{
    /*
    if(!_initFlag)
    {
        logOut(MsgNode_Error,"openFileRecording: context is not initialized!");
        return false;
    }

    _playerRepeat = true;

    _rc = _globalContext.OpenFileRecording(filePath,_player);
    if(_rc != XN_STATUS_OK)
        return false;

    // add all the nodes to the player line
    createDepth(false);
    createRgb(false);
    createIr(false);
    createScene(false);
    createUser(XN_SKEL_PROFILE_ALL,false);
    createGesture(false);
    createHands(false);

    _nodes |= Node_Player;
    */

    return true;
}

void ContextWrapper::playbackPlay(bool play)
{
    _playerPlay = play;
    if(_playerPlay)
    {
        seekPlayer(_playerPlayFrame);
    }
    else
    {
        _playerPlayFrame = curFramePlayer();
    }
}
bool ContextWrapper::isPlaybackPlay() const
{
    return _playerPlay;
}

void ContextWrapper::setPlaybackSpeedPlayer(float speed)
{
    if(_player == NULL || _player->isValid() == false)
        return;
    _player->setSpeed (speed);
}

float ContextWrapper::playbackSpeedPlayer()
{
    if(_player == NULL || _player->isValid() == false)
        return 0.0f;
    return  _player->getSpeed();
}

void ContextWrapper::setRepeatPlayer(bool loop)
{
    if(_player == NULL || _player->isValid() == false)
        return;
    _playerRepeat = loop;
    _player->setRepeatEnabled(loop);
}

bool ContextWrapper::repeatPlayer()
{
    if(_player == NULL || _player->isValid() == false)
        return false;
    return _player->getRepeatEnabled();
}

int ContextWrapper::curFramePlayer()
{
    if(_player == NULL || _player->isValid() == false)
        return 0;

    if(_depthFrame.isValid())
        return  _depthFrame.getFrameIndex();
    else if(_imageFrame.isValid())
        return  _imageFrame.getFrameIndex();
    else if(_irFrame.isValid())
        return  _irFrame.getFrameIndex();

    return 0;
}

int ContextWrapper::framesPlayer()
{
    if(_player == NULL || _player->isValid() == false)
        return 0;

    openni::VideoStream* videoStream = NULL;
    for(int i=0;i < _streamCount;i++)
    {
        if(_streams[i]->isValid())
        {
            videoStream = _streams[i];
            break;
        }

    }

    if(videoStream)
        return _player->getNumberOfFrames(*videoStream);
    else
        return 0;
}

void ContextWrapper::seekPlayer(int pos)
{
    if(_player == NULL || _player->isValid() == false)
        return;

    bool seekFlag = false;
    for(int i=0;i < _streamCount;i++)
    {
        if(_streams[i]->isValid())
        {
            if(pos < 0)
                pos = 0;
            else if(pos >= framesPlayer() - 1)
                pos = framesPlayer() -1;

            _rc = _player->seek(*(_streams[i]),pos);
            if(_rc == openni::STATUS_OK)
            {
                seekFlag = true;
                break;
            }
            else if((_rc == openni::STATUS_NOT_IMPLEMENTED) || (_rc == openni::STATUS_NOT_SUPPORTED) || (_rc == openni::STATUS_BAD_PARAMETER) || (_rc == openni::STATUS_NO_DEVICE))
            {
                logOut(MsgNode_Error,"Seeking is not supported");
            }
            else
            {
                logOut(MsgNode_Error,"Error seeking to frame:\n%s", openni::OpenNI::getExtendedError());
            }
        }
    }

    if(seekFlag)
    {
        if(_playerPlay == false)
        {
            boost::mutex::scoped_lock l(_mainMutex);
            updateOpenNI(true);
        }
    }

}

bool ContextWrapper::isEndPlayer()
{
    if(_player == NULL || _player->isValid() == false)
        return false;

    if(_depthStream.isValid())
        return  _depthFrame.getFrameIndex() >= framesPlayer();
    else if(_imageStream.isValid())
        return  _imageFrame.getFrameIndex() >= framesPlayer();

    return false;
}


///////////////////////////////////////////////////////////////////////////////
// update

// depth
bool ContextWrapper::updateDepthData()
{
    if(_depthMapTimeStamp == _updateTimeStamp)
        return false;

    // copy the data
    {
        boost::mutex::scoped_lock l(_depthMutex);

        if(_depthFrame.isValid())
        {
            const openni::DepthPixel* pDepth = (const openni::DepthPixel*)_depthFrame.getData();
            for(int i=0;i < _depthBufSize;i++)
                _depthMapBuffer[i] = (int)(pDepth[i]);
        }
    }

    _depthMapTimeStamp  = _updateTimeStamp;

    // check if a user defined coordinate system is used
    if(_userCoordsysFlag)
    {
 //       updateDepthRealWorldData();
/*
        // update depthMap according to the real world data
        float*  pVec    = _depthMapRealWorld;
        int*    pDepth  = _depthMapBuffer;
        for(int i = 0; i < _depthBufSize; i++)
        {
            *pDepth = pVec[2];  // set z-value

            pVec+=3;
            pDepth++;
        }
        */
    }

    return true;
}

bool ContextWrapper::updateDepthImageData()
{
    if(_depthImageTimeStamp == _updateTimeStamp)
        return false;

    // updtate the base data
    updateDepthData();

    calcHistogram();
    createDepthImage();

    _depthImageTimeStamp = _updateTimeStamp;

    return true;
}

bool ContextWrapper::updateDepthRealWorldData()
{
    if(_depthRealWorldTimeStamp == _updateTimeStamp)
        return false;

    // updtate the base data
    updateDepthData();
    calcDepthImageRealWorld();

    _depthRealWorldTimeStamp = _updateTimeStamp;

    return true;
}


// image
void ContextWrapper::updateRgbData()
{
    if(_imageTimeStamp == _updateTimeStamp)
        return;

    {
        boost::mutex::scoped_lock l(_imgMutex);
        copyRgbImage(_imgBuffer);
    }

    _imageTimeStamp = _updateTimeStamp;
}

// ir
void ContextWrapper::updateIrData()
{	
    if(_irTimeStamp == _updateTimeStamp)
        return;

    {
        boost::mutex::scoped_lock l(_irMutex);
        createIrImage();
    }

    _irTimeStamp = _updateTimeStamp;
}

/*
// user
void ContextWrapper::updateSceneData()
{
    if(_sceneTimeStamp == _updateTimeStamp)
        return;

    // get the new data
    _sceneAnalyzer.GetMetaData(_sceneMD);

    //boost::mutex::scoped_lock l(_mainMutex);
    _sceneTimeStamp = _updateTimeStamp;
}

void ContextWrapper::updateSceneImageData()
{
    if(_sceneImageTimeStamp == _updateTimeStamp)
        return;

    updateSceneData();
    if(_depth.IsValid())
        updateDepthImageData();
    calcSceneData();

    //boost::mutex::scoped_lock l(_mainMutex);
    _sceneImageTimeStamp = _updateTimeStamp;
}
*/
void ContextWrapper::updateUser()
{
    if(_userTimeStamp == _updateTimeStamp)
        return;

    if(_depthStream.isValid())
        updateDepthImageData();

    calcUserData();

    //boost::mutex::scoped_lock l(_mainMutex);
    _userTimeStamp = _updateTimeStamp;
}



/*
void ContextWrapper::updateHands()
{
    if(_handsTimeStamp == _updateTimeStamp)
        return;

    //boost::mutex::scoped_lock l(_mainMutex);
    _handsTimeStamp = _updateTimeStamp;
}

void ContextWrapper::updateGesture()
{
    if(_gestureTimeStamp == _updateTimeStamp)
        return;

    //boost::mutex::scoped_lock l(_mainMutex);
    _gestureTimeStamp = _updateTimeStamp;
}
*/
///////////////////////////////////////////////////////////////////////////////
// time measure



void ContextWrapper::updateAll()
{
    static bool start = false;

    if(!start)
    {
        //_globalContext.StartGeneratingAll();
    }


    std::vector<class ContextWrapper*>::iterator itr = _classList.begin();
    for(;itr != _classList.end();itr++)
    {
        // update timestamp
        (*itr)->update();
                /*
        (*itr)->_updateTimeStamp = (*itr)->_updateSubTimeStamp;
        (*itr)->_updateSubTimeStamp++;;
                */
    }

}


void ContextWrapper::genFirstTime()
{

    // setup the streams
    std::vector<openni::VideoStream*>   streamList;
    std::vector<openni::VideoFrameRef*> streamFrameRefList;

    // depth
    if(_depthStream.isValid() &&
       (_userTracker.isValid() == false ||
        _handTracker.isValid() == false) )
    {
        streamList.push_back(&_depthStream);
        streamFrameRefList.push_back(&_depthFrame);
    }

    // cam image
    if(_imageStream.isValid())
    {
        streamList.push_back(&_imageStream);
        streamFrameRefList.push_back(&_imageFrame);
    }

    // ir
    if(_irStream.isValid())
    {
        streamList.push_back(&_irStream);
        streamFrameRefList.push_back(&_irFrame);
    }

    if(streamList.size() > 0)
    {
        _streams = new openni::VideoStream*[streamList.size()];
        _streamsFrameRef = new openni::VideoFrameRef*[streamList.size()];
        _streamCount = streamList.size();

        // start all the streams
        for(int i=0;i < _streamCount;i++)
        {
            _streams[i]         = streamList[i];
            _streamsFrameRef[i] = streamFrameRefList[i];
        }
    }    

/*
    if(_recorder.isValid())
        _recorder.start();
*/
    _generatingFlag = true;
}


void ContextWrapper::update()
{
    if(!_initFlag)
        return;

    if(!_generatingFlag)
        genFirstTime();

    // check if everything is all right for the first round
    if(_firstTimeUpdate)
    {
        _firstTimeUpdate = false;

        // check if we use multithreading
        if(_threadMode == RunMode_MultiThreaded)
        {
            // create + start the thread
            _thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&ContextWrapper::run, this)));
        }
    }

    // update timestamp
    _updateTimeStamp = _updateSubTimeStamp;

    // update the data
    if(_threadMode < RunMode_MultiThreaded)
        updateSub();        
}

void ContextWrapper::updateSub()
{
    boost::mutex::scoped_lock l(_mainMutex);

    // update NITE
    if(_userTracker.isValid())
    {
        {
            boost::mutex::scoped_lock l(_depthMutex);
/*
            if(_player)
            {   // recorded scene
                if(_playerPlay == false && force == false)
*/

            nite::Status rc = _userTracker.readFrame(&_userFrameRef);
            if (rc != nite::STATUS_OK)
            {
                printf("GetNextData failed\n");
                return;
            }

            // read the data
            _depthFrame = _userFrameRef.getDepthFrame();
        }

        // get the temp user list
        _userIdTempList.clear();
        const nite::Array<nite::UserData>& users = _userFrameRef.getUsers();
        for (int i = 0; i < users.getSize(); ++i)
        {
            _userIdTempList[ (int)users[i].getId() ] = &(users[i]);

            // update callbacks
            doNiteUserCb(users[i]);
        }

    }

    if(_handTracker.isValid())
    {
        {
            boost::mutex::scoped_lock l(_depthMutex);

            nite::Status rc = _handTracker.readFrame(&_handRefFrame);
            if (rc != nite::STATUS_OK)
            {
                printf("GetNextData failed\n");
                return;
            }

            if(_userTracker.isValid() == false)
                _depthFrame = _handRefFrame.getDepthFrame();
        }

        // gesture
        _gestureIdTempList.clear();
        const nite::Array<nite::GestureData>& gestures = _handRefFrame.getGestures();
        for (int i = 0; i < gestures.getSize(); ++i)
        {
            _gestureIdTempList[ (int)gestures[i].getType() ] = &(gestures[i]);

            // update callbacks
            doNiteGestureCb(gestures[i]);
        }

        // get the temp hand list
        _handIdTempList.clear();
        const nite::Array<nite::HandData>& hands = _handRefFrame.getHands();
        for (int i = 0; i < hands.getSize(); ++i)
        {
            _handIdTempList[ (int)hands[i].getId() ] = &(hands[i]);

            // update callbacks
            doNiteHandCb(hands[i]);
        }
    }

    // update openni
    updateOpenNI();

    // update timestamp
    _updateSubTimeStamp++;
}

bool ContextWrapper::updateOpenNI(bool force)
{
    static bool skipFlag = true;

    // update openni
    if(_streamCount > 0)
    {
        if(_player)
        {   // recorded scene
            if(_playerPlay == false && force == false)
            {
                return false;
            }

            /*
            int changedIndex;
            std::cout << "111" << std::endl;
            _rc = openni::OpenNI::waitForAnyStream(_streams, _streamCount, &changedIndex);
            std::cout << "222" << std::endl;
            if (_rc != openni::STATUS_OK)
            {
                printf("Wait failed: %s\n",openni::OpenNI::getExtendedError());
                return false;
            }
            else
                _streams[changedIndex]->readFrame(_streamsFrameRef[changedIndex]);
            std::cout << "333" << std::endl;

            return true;
            */
        }

        if(skipFlag && _recorder.isValid())
        {
            skipFlag = false;
            return true;
        }


        // read out depthMap
        if(_depthStream.isValid() &&
           _userTracker.isValid() == false &&
           _handTracker.isValid() == false)
        {
            boost::mutex::scoped_lock l(_depthMutex);
            _depthStream.readFrame(&_depthFrame);
        }

        // read out rgb image
        if(_imageStream.isValid())
        {
            boost::mutex::scoped_lock l(_imgMutex);
            _imageStream.readFrame(&_imageFrame);
        }

        // read out infrared image
        if(_irStream.isValid())
        {
            boost::mutex::scoped_lock l(_irMutex);
            _irStream.readFrame(&_irFrame);
        }

        static bool oneShot = true;
        if(_recorder.isValid() && oneShot)
        {
            oneShot = false;
            _recorder.start();
        }

        return true;
    }

    return false;
}

void ContextWrapper::doNiteUserCb(const nite::UserData& userData)
{
    if(userData.isNew())
    {
        SOPENNI_CB_CALL(NewUser,userData.getId());

        // add the user state list, if the userId is already there, no insert will be made
        _userLastState[ (int)userData.getId() ] = -1;
    }
    else if(userData.isLost())
    {
        SOPENNI_CB_CALL(LostUser,userData.getId());

        _userLastState.erase((int)userData.getId());
    }
    else if(userData.isVisible())
    {
        SOPENNI_CB_CALL(VisibleUser,userData.getId());
        //_userLastState[ (int)userData.getId() ] = userData.getState();
            /*
            std::map<int,int>::iterator findItr = _userLastState.find((int)userData.getId() );
            if(findItr != _userLastState.end() && findItr->second != userData.getState())
            {
                SOPENNI_CB_CALL(VisibleUser,userData.getId());
                _userLastState[ (int)userData.getId() ] = userData.getState();
            }
            */
    }

/*
    case nite::USER_STATE_OUT_OF_SCENE:
        {
            std::map<int,int>::iterator findItr = _userLastState.find((int)userData.getId() );
            if(findItr != _userLastState.end() && findItr->second != userData.getState())
            {
                SOPENNI_CB_CALL(OutOfSceneUser,userData.getId());
                _userLastState[ (int)userData.getId() ] = userData.getState();
            }
        }
        break;
    }
*/
}

void ContextWrapper::doNiteHandCb(const nite::HandData& handData)
{
    if(handData.isNew())
    {
        nite::Point3f   pos = handData.getPosition();

        SOPENNI_CB_CALL(NewHand,handData.getId(),Vec3f(pos.x,pos.y,pos.z));

        // add the hand state list, if the userId is already there, no insert will be made
        _handLastState[ (int)handData.getId() ] = -1;
    }
    else if(handData.isLost())
    {
        SOPENNI_CB_CALL(LostHand,handData.getId());

        _handLastState.erase((int)handData.getId());
    }
    else if(handData.isTracking())
    {
        nite::Point3f   pos = handData.getPosition();

        SOPENNI_CB_CALL(TrackedHand,handData.getId(),Vec3f(pos.x,pos.y,pos.z));
    //    _handLastState[ (int)handData.getId() ] = handData.getState();
    }
    else if(handData.isTouchingFov())
    {

    }

    /*
    switch(handData.getState())
    {
    case nite::HAND_STATE_NEW:
        {
            nite::Point3f   pos = handData.getPosition();

            SOPENNI_CB_CALL(NewHand,handData.getId(),Vec3f(pos.x,pos.y,pos.z));

            // add the hand state list, if the userId is already there, no insert will be made
            _handLastState[ (int)handData.getId() ] = -1;
        }
        break;
    case nite::HAND_STATE_LOST:
        SOPENNI_CB_CALL(LostHand,handData.getId());

        _handLastState.erase((int)handData.getId());
        break;
    case nite::HAND_STATE_TRACKED:
        {
            nite::Point3f   pos = handData.getPosition();

            SOPENNI_CB_CALL(TrackedHand,handData.getId(),Vec3f(pos.x,pos.y,pos.z));
            _handLastState[ (int)handData.getId() ] = handData.getState();
        }
        break;
    }
    */
}

void ContextWrapper::doNiteGestureCb(const nite::GestureData& gestureData)
{
    if(gestureData.isComplete())
    {
        nite::Point3f   pos = gestureData.getCurrentPosition();

        SOPENNI_CB_CALL(CompletedGesture,
                        gestureData.getType(),
                        Vec3f(pos.x,pos.y,pos.z));

   //     _gestureLastState[ (int)gestureData.getType() ] = gestureData.getState();
    }
    else if(gestureData.isInProgress())
    {
    }

    /*
        switch(gestureData.getState())
    {
    case nite::GESTURE_STATE_NEW:
        SOPENNI_CB_CALL(NewGesture,gestureData.getType());

        // add the hand state list, if the userId is already there, no insert will be made
        _gestureLastState[ (int)gestureData.getType() ] = -1;
        break;
    case nite::GESTURE_STATE_IN_PROGRESS:
        SOPENNI_CB_CALL(InProgressGesture,gestureData.getType());

        _gestureLastState.erase((int)gestureData.getType());
        break;
    case nite::GESTURE_STATE_ABORTED:
        SOPENNI_CB_CALL(AbortedGesture,gestureData.getType());

        _gestureLastState.erase((int)gestureData.getType());
        break;
    case nite::GESTURE_STATE_COMPLETED:
        {
        nite::Point3f   pos = gestureData.getCurrentPosition();

        SOPENNI_CB_CALL(CompletedGesture,
                        gestureData.getType(),
                        Vec3f(pos.x,pos.y,pos.z));

        _gestureLastState[ (int)gestureData.getType() ] = gestureData.getState();
        }
        break;
    }
    */
}

void ContextWrapper::calcHistogram()
{
    memset(_pDepthHist, 0, MAX_DEPTH * sizeof(float));

    int*   pDepth = _depthMapBuffer;

    unsigned int nNumberOfPoints = 0;
    for(int y = 0; y < _depthVideoMode.getResolutionY(); ++y)
    {
        for(int x = 0; x < _depthVideoMode.getResolutionX(); ++x, ++pDepth)
        {
            if(*pDepth != 0)
            {
                _pDepthHist[*pDepth]++;
                nNumberOfPoints++;
            }
        }
    }

    for(int nIndex=1; nIndex < MAX_DEPTH; nIndex++)
    {
        _pDepthHist[nIndex] += _pDepthHist[nIndex-1];
    }

    if(nNumberOfPoints)
    {
        for(int nIndex=1; nIndex<MAX_DEPTH; nIndex++)
            _pDepthHist[nIndex] = (float)(unsigned int)(256 * (1.0f - (_pDepthHist[nIndex] / nNumberOfPoints)));
    }

}

void ContextWrapper::calcDepthImageRealWorld()
{
    boost::mutex::scoped_lock l(_depthMutex);

    float*  map     = _depthMapRealWorld;
    int*    pDepth  = _depthMapBuffer;

    for(int y = 0; y < _depthVideoMode.getResolutionY(); ++y)
    {
        for(int x = 0; x < _depthVideoMode.getResolutionX() ; ++x, ++pDepth,map+=3)
        {
            convertProjectiveToRealWorld(x,y,*pDepth,
                                         map, map + 1, map + 2);
        }
    }

}

void ContextWrapper::createDepthImage()
{
    memset(_pDepthImage, 0, _depthBufSize * sizeof(int));

    int*   pDepth;
    int*   pPixel;

    for(int y = 0; y < _depthVideoMode.getResolutionY(); ++y)
    {
        pDepth	= _depthMapBuffer + (y * _depthVideoMode.getResolutionX());
        pPixel	= _pDepthImage + (y * _depthVideoMode.getResolutionX());

        for(int x = 0; x < _depthVideoMode.getResolutionX(); x++, pDepth++, pPixel++)
        {
            if(*pDepth != 0)
            {
                int nHistValue = (int)_pDepthHist[*pDepth];

                switch(_depthImageColorMode)
                {
                case DepthImgMode_RgbFade:
                {
                    int pval =_pDepthGamma[(int)((*pDepth) / 1)];
                    //int lb =  255 - (pval & 0xff);
                    int lb =  pval & 0xff;

                    switch (pval>>8)
                    {
                    case 0:
                        *pPixel = 0xff	<< 24 |
                                  255	<< 16|
                                  (255-lb) << 8 |
                                  (255-lb) ;
                        break;
                    case 1:
                        *pPixel = 0xff	<< 24 |
                                  255	<< 16|
                                  lb << 8 |
                                  0 ;
                        break;
                    case 2:
                        *pPixel = 0xff	<< 24 |
                                  (255-lb) << 16|
                                  255 << 8 |
                                  0 ;
                        break;
                    case 3:
                        *pPixel = 0xff	<< 24 |
                                  0	<< 16|
                                  255 << 8 |
                                  lb ;
                        break;
                    case 4:
                        *pPixel = 0xff	<< 24 |
                                  0	<< 16|
                                  (255-lb) << 8 |
                                  0 ;
                        break;
                    case 5:
                        *pPixel = 0xff	<< 24 |
                                  0	<< 16|
                                  0 << 8 |
                                  (255-lb) ;
                        break;
                    default:
                        *pPixel = 0xff	<< 24 |
                                  0	<< 16|
                                  0 << 8 |
                                  0;
                        break;
                    }
                }
                    break;
                case DepthImgMode_Default:
                default:
                    *pPixel = ((int)0xff	<< 24) |
                              ((int)(nHistValue * _depthImageColor[0]))	<< 16 |
                              ((int)(nHistValue * _depthImageColor[1])) << 8 |
                              ((int)(nHistValue * _depthImageColor[2])) ;
                    break;
                }
            }
        }
    }
}


///////////////////////////////////////////////////////////////////////////////
// depth methods

float ContextWrapper::hFieldOfView()
{
    if(_depthStream.isValid() == false)
        return 0.0f;
    else
        return _depthStream.getHorizontalFieldOfView();
}

float ContextWrapper::vFieldOfView()
{
    if(_depthStream.isValid() == false)
        return 0.0f;
    else
        return _depthStream.getVerticalFieldOfView();
}


int ContextWrapper::depthWidth()
{
    return _depthVideoMode.getResolutionX();
}

int	ContextWrapper::depthHeight()
{
    return _depthVideoMode.getResolutionY();
}


int ContextWrapper::depthImage(int* map)
{
    if(_depthStream.isValid() == false)
        return 0;
    else
        updateDepthImageData();

    memcpy((void*)map,(void*)_pDepthImage,_depthBufSize * sizeof(int));

    return _depthBufSize;
}

int ContextWrapper::depthMap(int* map)
{
    updateDepthData();

    memcpy(map,_depthMapBuffer,_depthBufSize * sizeof(int));

    return _depthBufSize;
} 

int ContextWrapper::depthMapRealWorld(float* map)
{
    if(_depthStream.isValid() == false)
        return 0;
    else
        updateDepthRealWorldData();

    // speed up the copy
    memcpy((void*)map,(const void *)_depthMapRealWorld,_depthBufSize * sizeof(float) * 3);

    return _depthBufSize;
}  

float*  ContextWrapper::depthMapRealWorldA()
{
    if(_depthStream.isValid() == false)
        return 0;
    else
        updateDepthRealWorldData();

    return _depthMapRealWorld;
}

int ContextWrapper::depthMapSize()
{
    return _depthBufSize;
}

int ContextWrapper::depthHistSize()
{
    return MAX_DEPTH;
}

int ContextWrapper::depthHistMap(float* histMap)
{
    if(_depthStream.isValid() == false)
        return 0;
    else
        updateDepthImageData();

    memcpy((void*)histMap,(const void *)_pDepthHist,MAX_DEPTH * sizeof(float));

    return MAX_DEPTH;
}

void ContextWrapper::setDepthImageColor(int r,int g,int b)
{
    _depthImageColor[0] = r * 1.0f / 255.0f ;
    _depthImageColor[1] = g * 1.0f / 255.0f ;
    _depthImageColor[2] = b * 1.0f / 255.0f ;

}

void ContextWrapper::setDepthImageColorMode(int mode)
{
    _depthImageColorMode = mode;
}

int ContextWrapper::depthImageColorMode()
{
    return _depthImageColorMode;
}


///////////////////////////////////////////////////////////////////////////////
// rgb image methods

int ContextWrapper::rgbWidth()
{
    return _imageVideoMode.getResolutionX();
}

int	ContextWrapper::rgbHeight()
{
    return _imageVideoMode.getResolutionY();
}

bool ContextWrapper::copyRgbImage(int* map)
{
    if(_imageFrame.isValid() == false)
        return false;

    const openni::RGB888Pixel* pImageRow = (const openni::RGB888Pixel*)_imageFrame.getData();
    int rowSize = _imageFrame.getStrideInBytes() / sizeof(openni::RGB888Pixel);
    rowSize     = rgbWidth();
    int* pMap   = map;

    for(int y=0;y < rgbHeight();y++)
    {
        const openni::RGB888Pixel*  pPixel = pImageRow;
      //  openni::RGB888Pixel*        pTex = pTexRow + m_colorFrame.getCropOriginX();

        for(int x=0;x < rgbWidth();x++,pPixel++,pMap++)
        {
            *pMap = 0xff << 24 |
                    ((int)(0xff & pPixel->r)) << 16|
                    ((int)(0xff & pPixel->g)) << 8 |
                    ((int)(0xff & pPixel->b)) ;
        }

        pImageRow += rowSize;
    }
    return true;
}

int ContextWrapper::rgbImage(int* map)
{
    updateRgbData();

    memcpy(map,_imgBuffer,sizeof(int) * _imgBufSize);

    return _imgBufSize;
}


///////////////////////////////////////////////////////////////////////////////
// ir methods

int ContextWrapper::irWidth()
{
    return _irVideoMode.getResolutionX();
}

int ContextWrapper::irHeight()
{
    return _irVideoMode.getResolutionY();
}

int ContextWrapper::irImage(int* map)
{
    if(_irStream.isValid() == false)
        return 0;
    else
        updateIrData();

    memcpy((void*)map,(void*)_pIrImage,_irBufSize * sizeof(int));

    return _irBufSize;
}

int ContextWrapper::irMap(int* map)
{
    if(_irStream.isValid() == false)
        return 0;
    else
        updateIrData();

    const unsigned char*   pIrData = (const unsigned char*)_irFrame.getData();

    for(int i=0;i < _irBufSize;i++)
        map[i] = (int)(pIrData[i]);
    return _irBufSize;
}


void ContextWrapper::createIrImage()
{
    if(_irFrame.isValid() == false)
        return;

    switch(_irFrame.getVideoMode().getPixelFormat())
    {
        case ONI_PIXEL_FORMAT_GRAY8:
            {
                const unsigned char*   pIrData = (const unsigned char*)_irFrame.getData();
                int*                   pIrImage = _pIrImage;
                unsigned char          val;
                for(int i = 0;i < _irBufSize;i++)
                {
                    val = *pIrData;
                    *pIrImage = ((int)0xff	<< 24) |
                                ((int)val)	<< 16 |
                                ((int)val)  << 8 |
                                ((int)val) ;
                    pIrData++;
                    pIrImage++;
                }
            }
            break;
        case ONI_PIXEL_FORMAT_GRAY16:
            {
                const unsigned short*   pIrData = (const unsigned short*)_irFrame.getData();
                int*                    pIrImage = _pIrImage;
                unsigned char           val;
                for(int i = 0;i < _irBufSize;i++)
                {
                    val = (*pIrData) >> 2;
                    *pIrImage = ((int)0xff	<< 24) |
                                ((int)val)	<< 16 |
                                ((int)val)  << 8 |
                                ((int)val) ;
                    pIrData++;
                    pIrImage++;
                }
            }
            break;
        default:
            break;
    }

}

/*
///////////////////////////////////////////////////////////////////////////////
// scene
int ContextWrapper::sceneWidth()
{
    return _sceneMD.XRes();
}

int	ContextWrapper::sceneHeight()
{
    return _sceneMD.YRes();
}


void ContextWrapper::getSceneFloor(XnVector3D* point,	
                                   XnVector3D* normal)
{
    if(!_sceneAnalyzer.IsValid())
        return;
    else
        updateSceneData();

    XnPlane3D plane;
    _sceneAnalyzer.GetFloor(plane);

    point->X = plane.ptPoint.X;
    point->Y = plane.ptPoint.Y;
    point->Z = plane.ptPoint.Z;

    normal->X = plane.vNormal.X;
    normal->Y = plane.vNormal.Y;
    normal->Z = plane.vNormal.Z;


// // floor orientation test
// Eigen::Vector3f	p(plane.ptPoint.X,plane.ptPoint.Y,plane.ptPoint.Z);
// Eigen::Vector3f	n(plane.vNormal.X,plane.vNormal.Y,plane.vNormal.Z);
// HyperPlane3d	plane3d(n,p);

// Eigen::Vector3f projNull;
// Eigen::Vector3f projNullNormal;
// projNull = plane3d.projection(Eigen::Vector3f(0.0f,0.0f,0.0f));
// projNullNormal = (Eigen::Vector3f(0.0f,0.0f,0.0f) - projNull).normalized();


//    // calc the orientation matrix for the floor
//    // coordinate center is in the projection of the camera eye to the floor


// Quaternion		camRot;
// Quaternion		planeRot;


}
*/

void ContextWrapper::calcUserData()
{
    if(_depthStream.isValid())
    {	// calc with depth shades

        if(_userFrameRef.isValid() == false)
            return;

        // copy the image
        memcpy(_pUserImage,_pDepthImage,_depthBufSize * sizeof(int));

        // set the user colors
        const nite::UserId* pPixels = _userFrameRef.getUserMap().getPixels();
        int nColorID;

        for(int i=0;i < _depthBufSize;i++)
        {
            if(pPixels[i] > 0)
            {
                nColorID = pPixels[i] % nColors;
                _pUserImage[i] = ((int)0xff	<< 24) |
                                 ((int)(255 * Colors[nColorID][0]))	<< 16 |
                                 ((int)(255 * Colors[nColorID][1])) << 8 |
                                 ((int)(255 * Colors[nColorID][2])) ;
            }
        }
    }
    else
    {	// calc with no background        
        memset(_pUserImage,0,_userBufSize * sizeof(int));
/*
        // set the user colors
        const nite::UserId* pPixels = _userFrameRef.getUserMap()->getPixels();
        int nColorID;

        for(int i=0;i < _depthBufSize;i++)
        {
            if(pPixels[i] > 0)
            {
                nColorID = label % nColors;
                _pUserImage[i] = 255 * Colors[nColorID];
            }
        }
*/
    }
}

/*
int ContextWrapper::sceneMap(int* map)
{
    if(!_sceneAnalyzer.IsValid())
        return 0;
    else
        updateSceneData();

    for(int i=0;i < _sceneBufSize;i++)
        map[i] = (int)(_sceneMD.Data())[i];
    return _sceneBufSize;
}

int ContextWrapper::sceneImage(int* map)
{
    if(!_sceneAnalyzer.IsValid())
        return 0;
    else
        updateSceneImageData();

    for(int i=0;i < _sceneBufSize;i++)
    {
        map[i] = 0xff	<< 24 |
                           _pSceneImage[i].nRed	<< 16|
                           _pSceneImage[i].nGreen << 8 |
                           _pSceneImage[i].nBlue ;
    }
    return _sceneBufSize;
}
*/
///////////////////////////////////////////////////////////////////////////////
// user methods

bool ContextWrapper::getCoM(int user, float*  com)
{
    if(!_userTracker.isValid())
        return false;
    else
        updateUser();

    boost::mutex::scoped_lock l(_mainMutex);

    const nite::UserData* userData = getUserData(user);
    if(userData)
    {
        nite::Point3f center = userData->getCenterOfMass();

        com[0] = center.x;
        com[1] = center.y;
        com[2] = center.z;

        calcUserCoordsys(com);
        return true;
    }
    else
        return false;
}

bool ContextWrapper::getBoundingBox(int user, float*  boundingbox)
{
    if(!_userTracker.isValid())
        return false;
    else
        updateUser();

    boost::mutex::scoped_lock l(_mainMutex);

    const nite::UserData* userData = getUserData(user);
    if(userData)
    {
        nite::BoundingBox boundingBox = userData->getBoundingBox();

        boundingbox[0] = boundingBox.min.x;
        boundingbox[1] = boundingBox.min.y;
        boundingbox[2] = boundingBox.min.z;

        boundingbox[3] = boundingBox.max.x;
        boundingbox[4] = boundingBox.max.y;
        boundingbox[5] = boundingBox.max.z;

        calcUserCoordsys(boundingbox);
        calcUserCoordsys(boundingbox + 3);
        return true;
    }
    else
        return false;
}


int ContextWrapper::getNumberOfUsers()
{
    if(!_userTracker.isValid())
        return 0;
    else
        updateUser();

    return _userFrameRef.getUsers().getSize();
}

int ContextWrapper::getUsers(std::vector<int>* userList)
{
    if(!_userTracker.isValid())
        return 0;
    else
        updateUser();

    const nite::Array<nite::UserData>& userNiteList = _userFrameRef.getUsers();
    userList->clear();
    for(int i=0;i < userNiteList.getSize();i++)
        userList->push_back((int)userNiteList[i].getId());

    return userList->size();
}

int	ContextWrapper::userWidth()
{
    return _userVideoMode.getResolutionX();
}

int	ContextWrapper::userHeight()
{
    return _userVideoMode.getResolutionY();
}

int ContextWrapper::userMap(int* map)
{
    if(!_userTracker.isValid())
        return 0;
    else
        updateUser();

    if(_userFrameRef.isValid() == false)
    {
        memset(map,0,_userBufSize * sizeof(int));
        return 0;
    }

    const nite::UserId* pPixels = _userFrameRef.getUserMap().getPixels();
    for(int i=0;i < _userBufSize;i++)
        map[i] = (int)(pPixels[i]);
    return _userBufSize;
}

int ContextWrapper::userImage(int* map)
{
    if(!_userTracker.isValid())
        return 0;
    else
        updateUser();

    memcpy((void*)map,(void*)_pUserImage,_userBufSize * sizeof(int));
    return _userBufSize;
}


/*
int	ContextWrapper::getUserPixels(int user,int* userSceneData)
{
    if(!_userGenerator.IsValid())
        return 0;
    else
        updateUser();

    SceneMetaData  sceneMD;
    _userGenerator.GetUserPixels(user,sceneMD);

    // update the size / it seams like the user changes the camera size(QVGA/VGA)
    _userWidth	= sceneMD.XRes();
    _userHeight = sceneMD.YRes();

    int					index = 0;
    const XnLabel*		pLabels= sceneMD.Data();
    for(unsigned int y=0; y < sceneMD.YRes(); y++)
    {
        for(unsigned int x=0; x < sceneMD.XRes(); x++)
        {
            userSceneData[index] = *pLabels;;

            pLabels++;
            index++;
        }
    }
    return sceneMD.XRes() * sceneMD.YRes();
}

bool ContextWrapper::getUserPostition(int user, XnBoundingBox3D*  pPosition )
{
    if(!_depth.IsValid())
        return false;

    UserPositionCapability  userPosCap = _depth.GetUserPositionCap();
    userPosCap.GetUserPosition(user,*pPosition);

    return true;

}
*/

bool ContextWrapper::isTrackingSkeleton(int user)
{
    if(!_userTracker.isValid())
        return false;
    else
        updateUser();
/*
    niteIsSkeletonTracking
    return _userGenerator.GetSkeletonCap().IsTracking(user) > 0;
*/
    std::map<int,const nite::UserData*>::iterator findItr = _userIdTempList.find(user);
    if(findItr != _userIdTempList.end())
        return (findItr->second->getSkeleton().getState() == nite::SKELETON_TRACKED);
    else
        return false;
}

/*
bool ContextWrapper::isCalibratedSkeleton(int user)
{
    if(!_userGenerator.IsValid())
        return false;
    else
        updateUser();

    return _userGenerator.GetSkeletonCap().IsCalibrated(user) > 0;
}

bool ContextWrapper::isCalibratingSkeleton(int user)
{
    if(!_userGenerator.IsValid())
        return false;
    else
        updateUser();

    return _userGenerator.GetSkeletonCap().IsCalibrating(user) > 0;
}

void ContextWrapper::requestCalibrationSkeleton(int user, bool force)
{
    if(!_userGenerator.IsValid())
        return;
    else
        updateUser();

    _userGenerator.GetSkeletonCap().RequestCalibration(user, force);
}

void ContextWrapper::abortCalibrationSkeleton(int user)
{
    if(!_userGenerator.IsValid())
        return;
    else
        updateUser();

    _userGenerator.GetSkeletonCap().AbortCalibration(user);

}

bool ContextWrapper::saveCalibrationDataSkeleton(int user,int slot)
{
    if(!_userGenerator.IsValid())
        return false;
    else
        updateUser();

    if(_userGenerator.GetSkeletonCap().IsCalibrated(user) == 0)
        return false;

    _rc = _userGenerator.GetSkeletonCap().SaveCalibrationData(user,slot);
    return(_rc == XN_STATUS_OK);
}

bool ContextWrapper::loadCalibrationDataSkeleton(int user,int slot)
{
    if(!_userGenerator.IsValid())
        return false;
    else
        updateUser();

    _rc = _userGenerator.GetSkeletonCap().LoadCalibrationData(user,slot);
    return(_rc == XN_STATUS_OK);
}

bool ContextWrapper::saveCalibrationDataSkeleton(int user,const char* calibrationFile)
{
    if(!_userGenerator.IsValid())
        return false;
    else
        updateUser();

    if(_userGenerator.GetSkeletonCap().IsCalibrated(user) == 0)
        return false;

    _rc = _userGenerator.GetSkeletonCap().SaveCalibrationDataToFile(user,calibrationFile);
    return(_rc == XN_STATUS_OK);
}

bool ContextWrapper::loadCalibrationDataSkeleton(int user,const char* calibrationFile)
{
    if(!_userGenerator.IsValid())
        return false;
    else
        updateUser();

    _rc = _userGenerator.GetSkeletonCap().LoadCalibrationDataFromFile(user,calibrationFile);
    return(_rc == XN_STATUS_OK);
}

void ContextWrapper::setSmoothingSkeleton(float factor)
{
    if(!_userGenerator.IsValid())
        return;

    _rc = _userGenerator.GetSkeletonCap().SetSmoothing(factor);
}

void ContextWrapper::clearCalibrationDataSkeleton(int slot)
{
    if(!_userGenerator.IsValid())
        return;

    _rc = _userGenerator.GetSkeletonCap().ClearCalibrationData(slot);
}

bool ContextWrapper::isCalibrationDataSkeleton(int slot)
{
    if(!_userGenerator.IsValid())
        return false;

    return _userGenerator.GetSkeletonCap().IsCalibrationData(slot) > 0;
}
*/

void ContextWrapper::startTrackingSkeleton(int user)
{
    if(!_userTracker.isValid())
        return;

    _userTracker.startSkeletonTracking(user);
}

void ContextWrapper::stopTrackingSkeleton(int user)
{
    if(!_userTracker.isValid())
        return;

    _userTracker.stopSkeletonTracking(user);
}

/*
void ContextWrapper::startPoseDetection(const char* pose,int user)
{
    if(!_userGenerator.IsValid())
        return;

    _userGenerator.GetPoseDetectionCap().StartPoseDetection(pose, user);
}

void ContextWrapper::stopPoseDetection(int user)
{
    if(!_userGenerator.IsValid())
        return;

    _userGenerator.GetPoseDetectionCap().StopPoseDetection(user);
}

*/

float ContextWrapper::getJointPositionSkeleton(int user,int joint,float* jointPos)
{
    if(!_userTracker.isValid())
        return 0.0f;
    else
        updateUser();

    std::map<int,const nite::UserData*>::iterator findItr = _userIdTempList.find(user);
    if(findItr != _userIdTempList.end())
    {
        const nite::SkeletonJoint& skelJoint = (findItr->second)->getSkeleton().getJoint((nite::JointType)joint);
        nite::Point3f point = skelJoint.getPosition();
        jointPos[0] = point.x;
        jointPos[1] = point.y;
        jointPos[2] = point.z;

        calcUserCoordsys(jointPos);
        return skelJoint.getPositionConfidence();
    }
    return 0.0f;
}

float ContextWrapper::getJointOrientationSkeleton(int user,
                                                 int joint,
                                                 float* jointOrientation)   // 3x3 matrix
{
    if(!_userTracker.isValid())
        return 0.0f;
    else
        updateUser();

    std::map<int,const nite::UserData*>::iterator findItr = _userIdTempList.find(user);
    if(findItr != _userIdTempList.end())
    {
        const nite::SkeletonJoint& skelJoint = (findItr->second)->getSkeleton().getJoint((nite::JointType)joint);
        nite::Quaternion orientation = skelJoint.getOrientation();

        // recalc quaternion to matrix
        Eigen::Quaternion<float> quat(orientation.w,orientation.x,orientation.y,orientation.z);

        // copy the data to the joint
        Eigen::Matrix3f     mat = quat.toRotationMatrix().transpose();
        memcpy(jointOrientation,mat.data(),sizeof(float) * 9);

        calcUserCoordsys(jointOrientation);

        return skelJoint.getPositionConfidence();
    }
    return 0.0f;
}



void ContextWrapper::convertRealWorldToProjective(float x,float y,float z,float* xp,float* yp,float* zp)
{
    if(!_depthStream.isValid())
        return;

    float localWc[]={x,y,z};
    calcUserCoordsysBack(localWc);

    openni::CoordinateConverter::convertWorldToDepth(_depthStream,
                                                     localWc[0],localWc[1],localWc[2],
                                                     xp,yp,zp);
}

void ContextWrapper::convertRealWorldToProjective(const float* pRealWorld,float* pProjective)
{
    if(!_depthStream.isValid())
        return;

    float localWc[]={pRealWorld[0],pRealWorld[1],pRealWorld[2]};
    calcUserCoordsysBack(localWc);

    openni::CoordinateConverter::convertWorldToDepth(_depthStream,
                                                     localWc[0],localWc[1],localWc[2],
                                                     &(pProjective[0]),&(pProjective[1]),&(pProjective[2]));
}


void ContextWrapper::convertProjectiveToRealWorld(float x,float y,float z,float* xr,float* yr,float* zr)
{
    if(!_depthStream.isValid())
        return;

    openni::CoordinateConverter::convertDepthToWorld(_depthStream,
                                                    x,y,z,
                                                    xr,yr,zr);
    calcUserCoordsys(xr,yr,zr);
}

void ContextWrapper::convertProjectiveToRealWorld(const float* pProjective,float* pRealWorld)
{
    if(!_depthStream.isValid())
        return;

    openni::CoordinateConverter::convertDepthToWorld(_depthStream,
                                                    pProjective[0],pProjective[1],pProjective[2],
                                                    &(pRealWorld[0]),&(pRealWorld[1]),&(pRealWorld[2]));
    calcUserCoordsys(pRealWorld);
}



bool ContextWrapper::alternativeViewPointDepthToImage()
{
    // print out deprecated
    //logDeprecated(MsgNode_Deprecated,"alternativeViewPointDepthToImage","setImageRegistrationMode");

    if(!_device.isValid())
        return false;

    return (_device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR) == openni::STATUS_OK);
}

bool ContextWrapper::setDepthToColor(bool enable)
{
    if(!_device.isValid())
        return false;

    return (_device.setImageRegistrationMode(enable ? openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR : openni::IMAGE_REGISTRATION_OFF) == openni::STATUS_OK);

}

bool ContextWrapper::depthToColor()
{
    if(!_device.isValid())
        return false;

    return(_device.getImageRegistrationMode() == openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
}

bool ContextWrapper::setDepthColorSyncEnabled(bool enable)
{
    if(!_device.isValid())
        return false;

    return(_device.setDepthColorSyncEnabled(enable) == openni::STATUS_OK);
}


/*
void ContextWrapper::moveKinect(float angle)
{
    if(_kinectMotors.open() == false)
        return;

    _kinectMotors.move(angle);
}
*/

///////////////////////////////////////////////////////////////////////////////
// access methods
void ContextWrapper::setMirror(bool flag)
{
    _mirrorFlag = flag;
    if(_depthStream.isValid())
        _depthStream.setMirroringEnabled(flag);
    if(_imageStream.isValid())
        _imageStream.setMirroringEnabled(flag);
    if(_irStream.isValid())
        _irStream.setMirroringEnabled(flag);

    /*
    if(_userCoordsysFlag)
    {   // mirror coordsys
        _userCoordsysRetMat = _userCoordsysRetMat * Eigen::Scaling(-1.0f,1.0f,1.0f);
        _userCoordsysForwardMat = _userCoordsysRetMat.inverse();
    }
    */
}

bool ContextWrapper::mirror()
{
    if(!_depthStream.isValid())
        return false;

    return _depthStream.getMirroringEnabled();
}

///////////////////////////////////////////////////////////////////////////////
// threading

void ContextWrapper::run()
{
    _threadRun = true;
std::cout << "run" << std::endl;
    while(_threadRun)
    {
        updateSub();

        boost::this_thread::sleep(boost::posix_time::millisec(1));
        //std::cout <<"." << std::flush;
    }

}

///////////////////////////////////////////////////////////////////////////////
// calibration
// ----------------------------------------------------------------------------
// watch out for the storage order of the matrix !!!!!
//
// OpenNI column-major
//      http://www.opengl.org/archives/resources/faq/technical/transformations.htm
// OpenNI row-major
//      http://groups.google.com/group/openni-dev/browse_thread/thread/fa26f5c97fa8d259
// Eigen by default column-major
//      http://eigen.tuxfamily.org/api/TopicStorageOrders.html

void ContextWrapper::setUserCoordsys(float centerX,float centerY,float centerZ,
                                     float xDirX,float xDirY,float xDirZ,
                                     float zDirX,float zDirY,float zDirZ)
{
 // _calibrationMat.identity();
    _userCoordsysNullPoint << centerX,centerY,centerZ;

    _userCoordsysXAxis << xDirX,xDirY,xDirZ;
    _userCoordsysXAxis -= _userCoordsysNullPoint;
    _userCoordsysXAxis.normalize();

    _userCoordsysZAxis << zDirX,zDirY,zDirZ;
    _userCoordsysZAxis -= _userCoordsysNullPoint;
    _userCoordsysZAxis.normalize();

    // calculate the up dir
    _userCoordsysYAxis = _userCoordsysZAxis.cross(_userCoordsysXAxis);
    _userCoordsysYAxis.normalize();

    // calculate the clean zAxis
    _userCoordsysZAxis = _userCoordsysXAxis.cross(_userCoordsysYAxis);
    _userCoordsysZAxis.normalize();


//    // calculate the xform
//    Eigen::Vector3f nullPointTrans(_userCoordsysNullPoint - Eigen::Vector3f(0.0f,0.0f,0.0f));

//     // left hand coordsys matrix !!!!!!!!!!
//    _userCoordsysMat[0] = _userCoordsysXAxis.x();   _userCoordsysMat[1] = _userCoordsysYAxis.x();   _userCoordsysMat[2] =   _userCoordsysZAxis.x(); _userCoordsysMat[3] =   _userCoordsysNullPoint.x();
//    _userCoordsysMat[4] = _userCoordsysXAxis.y();   _userCoordsysMat[5] = _userCoordsysYAxis.y();   _userCoordsysMat[6] =   _userCoordsysZAxis.y(); _userCoordsysMat[7] =   _userCoordsysNullPoint.y();
//    _userCoordsysMat[8] = _userCoordsysXAxis.z();   _userCoordsysMat[9] = _userCoordsysYAxis.z();   _userCoordsysMat[10] =  _userCoordsysZAxis.z(); _userCoordsysMat[11] =  _userCoordsysNullPoint.z();
//    _userCoordsysMat[12] = 0;                       _userCoordsysMat[13] = 0;                       _userCoordsysMat[14] =  0;                      _userCoordsysMat[15] =  1;


//    // debug output
//    std::cout << "_userCoordsysXAxis.x():" << _userCoordsysXAxis.x() << "\t" << "_userCoordsysXAxis.y():" << _userCoordsysXAxis.y() << "\t" << "_userCoordsysXAxis.z():" << _userCoordsysXAxis.z() << std::endl;
//    std::cout << "_userCoordsysYAxis.x():" << _userCoordsysYAxis.x() << "\t" << "_userCoordsysYAxis.y():" << _userCoordsysYAxis.y() << "\t" << "_userCoordsysYAxis.z():" << _userCoordsysYAxis.z() << std::endl;
//    std::cout << "_userCoordsysZAxis.x():" << _userCoordsysZAxis.x() << "\t" << "_userCoordsysZAxis.y():" << _userCoordsysZAxis.y() <<"\t" <<  "_userCoordsysZAxis.z():" << _userCoordsysZAxis.z() << std::endl;


    // calculate the transformation matrix
    Eigen::Vector3f origNull(0,0,0);
    Eigen::Vector3f origXAxis(1,0,0);
    Eigen::Vector3f origYAxis(0,1,0);
    Eigen::Vector3f origZAxis(0,0,1);

    Eigen::Vector3f userDefNull(centerX,centerY,centerZ);
    Eigen::Vector3f userDefXAxis(userDefNull + _userCoordsysXAxis);
    Eigen::Vector3f userDefYAxis(userDefNull + _userCoordsysYAxis);
    Eigen::Vector3f userDefZAxis(userDefNull + _userCoordsysZAxis);

    Eigen::Matrix<float,3,4> start,end;
    start.col(0)=origNull;
    start.col(1)=origXAxis;
    start.col(2)=origYAxis;
    start.col(3)=origZAxis;

    end.col(0)=userDefNull;
    end.col(1)=userDefXAxis;
    end.col(2)=userDefYAxis;
    end.col(3)=userDefZAxis;

    _userCoordsysRetMat = Eigen::umeyama(start,end,true);
    _userCoordsysForwardMat = _userCoordsysRetMat.inverse();
    _userCoordsysFlag = true;

    //Eigen::Matrix4f tranform = _userCoordsysRetMat.matrix().transpose();
    Eigen::Matrix4f tranform = _userCoordsysRetMat.matrix();
    memcpy(_userCoordsysMat,tranform.data(),sizeof(float)*16);

/*
    // debug output
    Eigen::Matrix4f 	testMat;
    testMat = Eigen::umeyama(start,end,true);

    Eigen::Matrix4f testInvMat = testMat.inverse();

    std::cout << "----- umeyama" << std::endl;
    std::cout << testMat << std::endl;
    std::cout << "----- invert" << std::endl;
    std::cout << testInvMat << std::endl;

    Eigen::Transform<float,3,Eigen::Affine> 	xform(testInvMat);

    std::cout << "----- reverse test" << std::endl;
    Eigen::Vector3f nullTest = xform * userDefNull;
    std::cout << "userDefNull: " << userDefNull << std::endl;
    std::cout << "nullTest: " << nullTest << std::endl;
*/
}

void ContextWrapper::resetUserCoordsys()
{
	_userCoordsysFlag = false;
}

bool ContextWrapper::hasUserCoordsys() const
{
  return _userCoordsysFlag;  
}

float* ContextWrapper::getUserCoordsysTransMat()
{
  if(_userCoordsysFlag == false)
	return NULL;
  // return a float[16] , 4x4 transform mat
  return _userCoordsysMat;
}

void ContextWrapper::getUserCoordsysTransMat(float* mat)
{
    if(_userCoordsysFlag == false)
      return;

    memcpy(mat,_userCoordsysMat,sizeof(float)*16);
}

bool ContextWrapper::getOrigUserCoordsys(float* nullPointX,float* nullPointY,float* nullPointZ,
										 float* xAxisX,float* xAxisY,float* xAxisZ,
										 float* yAxisX,float* yAxisY,float* yAxisZ,
										 float* zAxisX,float* zAxisY,float* zAxisZ)
{
  if(_userCoordsysFlag == false)
	return false;

  *nullPointX = _userCoordsysNullPoint.x();
  *nullPointY = _userCoordsysNullPoint.y();
  *nullPointZ = _userCoordsysNullPoint.z();
  
  *xAxisX = _userCoordsysXAxis.x();
  *xAxisY = _userCoordsysXAxis.y();
  *xAxisZ = _userCoordsysXAxis.z();

  *yAxisX = _userCoordsysYAxis.x();
  *yAxisY = _userCoordsysYAxis.y();
  *yAxisZ = _userCoordsysYAxis.z();
  
  *zAxisX = _userCoordsysZAxis.x();
  *zAxisY = _userCoordsysZAxis.y();
  *zAxisZ = _userCoordsysZAxis.z();
  
  return true;
}


void ContextWrapper::calcUserCoordsys(float* point)
{
    if(!_userCoordsysFlag)
        return;

    Eigen::Vector3f vec = _userCoordsysForwardMat * Eigen::Vector3f(point);
    point[0] = vec.x();
    point[1] = vec.y();
    point[2] = vec.z();
}

void ContextWrapper::calcUserCoordsys(float* x,float* y,float* z)
{
    if(!_userCoordsysFlag)
        return;

    Eigen::Vector3f vec = _userCoordsysForwardMat * Eigen::Vector3f(*x,*y,*z);
    *x = vec.x();
    *y = vec.y();
    *z = vec.z();
}

void ContextWrapper::calcUserCoordsysMat(float* mat)
{
    if(!_userCoordsysFlag)
        return;

    // check the
    Eigen::Matrix3f matOrg(mat);
    Eigen::Matrix3f res = _userCoordsysForwardMat.rotation() * matOrg.transpose();

    res.transposeInPlace();
    memcpy(mat,res.data(),sizeof(float)*9);
}

void ContextWrapper::calcUserCoordsysBack(float* point)
{
    if(!_userCoordsysFlag)
        return;

    Eigen::Vector3f vec = _userCoordsysRetMat * Eigen::Vector3f(point);
    point[0] = vec.x();
    point[1] = vec.y();
    point[2] = vec.z();
}

void ContextWrapper::calcUserCoordsysBack(float* x,float* y,float* z)
{
    if(!_userCoordsysFlag)
        return;

    Eigen::Vector3f vec = _userCoordsysRetMat * Eigen::Vector3f(*x,*y,*z);
    *x = vec.x();
    *y = vec.y();
    *z = vec.z();
}


void ContextWrapper::calcUserCoordsysBackMat(float* mat)
{
    if(!_userCoordsysFlag)
        return;

    Eigen::Matrix3f matOrg(mat);
    Eigen::Matrix3f res = _userCoordsysRetMat.rotation() * matOrg.transpose();

    res.transposeInPlace();
    memcpy(mat,res.data(),sizeof(float)*9);
}


void ContextWrapper::getUserCoordsys(float mat[])
{
    if(!_userCoordsysFlag)
        return;

    Eigen::Matrix4f tranform = _userCoordsysForwardMat.matrix().transpose();
    memcpy(mat,tranform.data(),sizeof(float)*16);
}

void ContextWrapper::getUserCoordsysBack(float mat[])
{
    if(!_userCoordsysFlag)
        return;

    Eigen::Matrix4f tranform = _userCoordsysRetMat.matrix().transpose();
    /*
    mat[0] = _userCoordsysRetMat.data()[0]; mat[1] = _userCoordsysRetMat.data()[1]; mat[2] =   _userCoordsysRetMat.data()[2];   mat[3] =   _userCoordsysRetMat.data()[12];
    mat[4] = _userCoordsysRetMat.data()[4]; mat[5] = _userCoordsysRetMat.data()[5]; mat[6] =   _userCoordsysRetMat.data()[6];   mat[7] =   _userCoordsysRetMat.data()[13];
    mat[8] = _userCoordsysRetMat.data()[8]; mat[9] = _userCoordsysRetMat.data()[9]; mat[10] =  _userCoordsysRetMat.data()[10];  mat[11] =  _userCoordsysRetMat.data()[14];
    mat[12] = _userCoordsysRetMat.data()[3];mat[13] = _userCoordsysRetMat.data()[8];mat[14] =  _userCoordsysRetMat.data()[11];   mat[15] =  _userCoordsysRetMat.data()[15];
    */

    memcpy(mat,tranform.data(),sizeof(float)*16);
}

///////////////////////////////////////////////////////////////////////////////
// helper function

// this code is based on the paper from Tomas Möller and Ben Trumbore
// 'Fast, minimum storage ray-triangle intersection.'
// http://www.graphics.cornell.edu/pubs/1997/MT97.html

bool rayTriangleIntersection(const Eigen::Vector3f& p,
                             const Eigen::Vector3f& d,
                             const Eigen::Vector3f& v0,
                             const Eigen::Vector3f& v1,
                             const Eigen::Vector3f& v2,
                             Eigen::Vector3f* hit)
{
    float a,f,u,v;
    Eigen::Vector3f e1 = v1 - v0;
    Eigen::Vector3f e2 = v2 - v0;

    Eigen::Vector3f h = d.cross(e2);
    a = e1.dot(h);

    if (a > -0.00001f && a < 0.00001f)
        return false;

    f = 1.0f / a;
    Eigen::Vector3f s = p - v0;
    u = f * s.dot(h);

    if (u < 0.0f || u > 1.0f)
        return false;

    Eigen::Vector3f q = s.cross(e1);
    v = f * d.dot(q);

    if (v < 0.0f || u + v > 1.0f)
        return false;

    float t = f * e2.dot(q);

    if (t > 0.00001f) // ray intersection
    {
        *hit = p + (d * t);
        return true;
    }
    else
        return false;

}

// http://www.openprocessing.org/sketch/45539

int raySphereIntersection(const Eigen::Vector3f& rayP,
                          const Eigen::Vector3f& dir,
                          const Eigen::Vector3f& sphereCenter,float sphereRadius,
                          Eigen::Vector3f* hit1, Eigen::Vector3f* hit2)
{
  Eigen::Vector3f e = dir.normalized();
  Eigen::Vector3f h = sphereCenter - rayP;
  float lf = e.dot(h);                      // lf=e.h
  float s = pow(sphereRadius,2) - h.dot(h) + pow(lf,2);   // s=r^2-h^2+lf^2
  if(s < 0.0)
      return 0;                    // no intersection points ?
  s = sqrt(s);                              // s=sqrt(r^2-h^2+lf^2)

  int result = 0;
  if(lf < s)                               // S1 behind A ?
  {
      if (lf+s >= 0)                          // S2 before A ?}
      {
        s = -s;                               // swap S1 <-> S2}
        result = 1;                           // one intersection point
      }
  }
  else
      result = 2;                          // 2 intersection points

  *hit1 = e * (lf-s) + rayP;
  *hit2 = e * (lf+s) + rayP;

  return result;
}

/*
// http://www.lighthouse3d.com/tutorials/maths/ray-sphere-intersection/
int raySphereIntersection(const Eigen::Vector3f& origin,
                          const Eigen::Vector3f& dir,
                          const Eigen::Vector3f& sphereCenter,
                          float sphereRadius,
                          Eigen::Vector3f* hit1,
                          Eigen::Vector3f* hit2,)
{
    Eigen::Vector3f vpc = sphereCenter - rayP;

    if ((vpc . dir) < 0) // when the sphere is behind the origin p
                        // note that this case may be dismissed if it is
                        // considered that p is outside the sphere
            if (fabs(vpc) > r)

                       // there is no intersection

        else if (|vpc| == r)

            intersection = p

        else // occurs when p is inside the sphere

            pc = projection of c on the line
                    // distance from pc to i1
            dist = sqrt(radius^2 - |pc - c|^2)
            di1 = dist - |pc - p|
            intersection = p + d * di1

    else // center of sphere projects on the ray

        pc = projection of c on the line
        if (| c - pc | > r)

            // there is no intersection

        else
                    // distance from pc to i1
            dist = sqrt(radius^2 - |pc - c|^2)

                if (|vpc| > r) // origin is outside sphere

                di1 = |pc - p| - dist

            else // origin is inside sphere

                di1 = |pc - p| + dist

            intersection = p + d * di1


    return false;
}
*/

bool ContextWrapper::rayTriangleIntersection(float p[],
                                             float dir[],
                                             float vec0[],
                                             float vec1[],
                                             float vec2[],
                                             float hit[])
{
    Eigen::Vector3f hitVec;

    if(::rayTriangleIntersection(Eigen::Vector3f(p),
                                 Eigen::Vector3f(dir),
                                 Eigen::Vector3f(vec0),
                                 Eigen::Vector3f(vec1),
                                 Eigen::Vector3f(vec2),
                                 &hitVec))
    {
        hit[0] = hitVec.x();
        hit[1] = hitVec.y();
        hit[2] = hitVec.z();
        //memcpy(hit,hitVec.data(),sizeof(float) * 3);
        return true;
    }

    return false;
}


int ContextWrapper::raySphereIntersection(float p[],
                                          float dir[],
                                          float sphereCenter[],
                                          float sphereRadius,
                                          float hit1[],float hit2[])
{
    Eigen::Vector3f hitVec1;
    Eigen::Vector3f hitVec2;

    int ret = ::raySphereIntersection(Eigen::Vector3f(p),
                                      Eigen::Vector3f(dir),
                                      Eigen::Vector3f(sphereCenter),sphereRadius,
                                      &hitVec1,&hitVec2);

    if(ret > 0)
    {
        hit1[0] = hitVec1.x();
        hit1[1] = hitVec1.y();
        hit1[2] = hitVec1.z();

        if(ret > 1)
        {
            hit2[0] = hitVec2.x();
            hit2[1] = hitVec2.y();
            hit2[2] = hitVec2.z();
        }
    }

    return ret;
}
