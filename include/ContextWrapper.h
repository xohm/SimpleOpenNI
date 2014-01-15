/* ----------------------------------------------------------------------------
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

#ifndef _CONTEXWRAPPER_H_
#define _CONTEXWRAPPER_H_

#ifdef WIN32
	#pragma warning( disable : 4251 )	// disable warnings from NITE
	#pragma warning( disable : 4275 )
#endif

#include <vector>
#include <iostream>

// boost
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/cstdint.hpp>

// eigen
#include <Eigen/Geometry>

// OpenNI
#include <OpenNI.h>
#include <NiTE.h>

// #include "KinectMotor.h"

///////////////////////////////////////////////////////////////////////////////
// defines

#define		Node_None		(0)
#define		Node_Depth		(1 << 0)
#define		Node_Image		(1 << 1)
#define		Node_Ir			(1 << 2)
#define		Node_Scene		(1 << 3)
#define		Node_User		(1 << 4)
#define		Node_Hand		(1 << 5)
#define		Node_Gesture	(1 << 6)
#define		Node_Recorder	(1 << 7)
#define		Node_Player		(1 << 8)

#define		DepthImgMode_Default	0
#define		DepthImgMode_RgbFade	1

#define		RunMode_Default			0
#define		RunMode_SingleThreaded	RunMode_Default
#define		RunMode_MultiThreaded	(RunMode_SingleThreaded + 1)


#define		MAX_DEPTH		10000	// 10m
#define		STRING_BUFFER	255

/*

#define SOPENNI_CB_NAME(FuncName)     FuncName##Cb
#define SOPENNI_CB_STATIC(FuncName,...)\
    public: \
    static void XN_CALLBACK_TYPE FuncName##Cb(__VA_ARGS__);
#define SOPENNI_CB_MEMBER(FuncName,...)\
    protected:\
    void on##FuncName##Cb(__VA_ARGS__);
*/
#define SOPENNI_CB_VIRTUAL(FuncName,...)\
    protected:\
    virtual void on##FuncName##Cb(__VA_ARGS__){;}

#define SOPENNI_CB_CALL(FuncName,...)\
    on##FuncName##Cb(__VA_ARGS__);

// added this typedef because of problems with virtal methods and the %apply
typedef const float* constPFloat;



namespace sOpenNI{

class Vec3f
{
public:
    Vec3f(float x,float y,float z)
    {
        vec[0] = x;
        vec[1] = y;
        vec[2] = z;

    }

    Vec3f(const Vec3f& copy)
    {
        vec[0] = copy.vec[0];
        vec[1] = copy.vec[1];
        vec[2] = copy.vec[2];
    }

    float x() const { return vec[0]; }
    float y() const { return vec[1]; }
    float z() const { return vec[2]; }

    const float* get() const { return vec; }

protected:
    float vec[3];
};


class ContextWrapper
{
public:

    ContextWrapper();
    virtual ~ContextWrapper();

    int version();

    static void chdir(const char* dir);
    static std::string getcwd();

    //////////////////////////////////////////////////////////////////////////////
    // init methods
    static bool initContext();
    bool init(int runMode=RunMode_SingleThreaded);
    bool init(int deviceIndex,int runMode);
    bool init(const char* record,int runMode=RunMode_SingleThreaded);

    bool isInit(){	return _initFlag; }

    void close();

    virtual void update();
    static void updateAll();

    int nodes();

    //////////////////////////////////////////////////////////////////////////////
    // multiple devices
    static int deviceCount();
    static int deviceNames(std::vector<std::string>* nodeNames);
    int deviceIndex() const { return _deviceIndex; }

    //////////////////////////////////////////////////////////////////////////////
    // depth methods
    virtual bool enableDepth();
    virtual bool enableDepth(int width,int height,int fps);

 //   xn::DepthGenerator& getDepthGenerator() { return _depth; }


    int depthWidth();
    int depthHeight();

    int     depthImage(int* map);		// argb 4-Bytes / alpha is not used

    void    setDepthImageColor(int r,int g,int b);
    void    setDepthImageColorMode(int mode);
    int     depthImageColorMode();

    int     depthMapSize();
    int     depthMap(int* map);					// in milimeters
    int     depthMapRealWorld(float* map);
    float*  depthMapRealWorldA();

    int depthHistSize();
    int depthHistMap(float* histMap);

    float hFieldOfView();
    float vFieldOfView();
    //////////////////////////////////////////////////////////////////////////////
    // cam image
    virtual bool enableRGB();
    virtual bool enableRGB(int width,int height,int fps);

    int rgbWidth();
    int rgbHeight();

    int	rgbImage(int* map);			// argb 4-Bytes / alpha is not used

    //////////////////////////////////////////////////////////////////////////////
    // ir image
    virtual bool enableIR();
    virtual bool enableIR(int width,int height,int fps);

    int irWidth();
    int irHeight();

    int irMap(int* map);			// 16-bit value
    int irImage(int* map);			// argb 4-Bytes / alpha is not used

/*
    //////////////////////////////////////////////////////////////////////////////
    // scene analyzer
    virtual bool enableScene();
    virtual bool enableScene(int width,int height,int fps);

    xn::SceneAnalyzer& getSceneAnalyzer() { return _sceneAnalyzer; }

    int sceneWidth();
    int sceneHeight();

    int sceneMap(int* map);
    int sceneImage(int* map);		// 16-bit value, with the labels, size of the depth map
    void getSceneFloor(XnVector3D* point,
                       XnVector3D* normal);
    void getSceneFloorCoordsys(float* matrix);		// 4*4 matrix, rotation + translation to the nearest point
    // on the floor plane to the camera as null point

    //////////////////////////////////////////////////////////////////////////////
    // users
    */
    virtual bool enableUser();

    int	userWidth();
    int	userHeight();

    bool	getCoM(int user, float*  com);
    bool	getBoundingBox(int user, float*  boudingbox);

    int     getNumberOfUsers();
    int     getUsers(std::vector<int>* userList);

    int     userMap(int* map);
    int     userImage(int* map);

    /*
    int     getUserPixels(int user,int* userSceneData);

    bool	getUserPostition(int user, XnBoundingBox3D*  pPosition );

    bool	isCalibratedSkeleton(int user);
    bool	isCalibratingSkeleton(int user);
    void	requestCalibrationSkeleton(int user, bool force);
    void	abortCalibrationSkeleton(int user);

    bool	saveCalibrationDataSkeleton(int user,int slot);
    bool	loadCalibrationDataSkeleton(int user,int slot);
    void	clearCalibrationDataSkeleton(int slot);
    bool	isCalibrationDataSkeleton(int slot);

    bool	saveCalibrationDataSkeleton(int user,const char* calibrationFile);
    bool	loadCalibrationDataSkeleton(int user,const char* calibrationFile);

    void	setSmoothingSkeleton(float factor);
*/
    bool	isTrackingSkeleton(int user);
    void	startTrackingSkeleton(int user);
    void	stopTrackingSkeleton(int user);
/*
    void	startPoseDetection(const char* pose,int user);
    void	stopPoseDetection(int user);
*/

    float	getJointPositionSkeleton(int user,int joint,float* jointPos);
    // 3*3 rotation matrix
    float	getJointOrientationSkeleton(int user,
                                        int joint,
                                        float* jointOrientation);

    //////////////////////////////////////////////////////////////////////////////
    // hands
    virtual bool enableHand();

    int 	startTrackingHand(float*pos);
    void	stopTrackingHand(int handId);
    void	stopTrackingAllHand();
    void	setSmoothingHand(float smoothingFactor);
    float	getSmoothingHand();

    void startGesture(int gesture);
    void endGesture(int gesture);
/*
    //////////////////////////////////////////////////////////////////////////////
    // audio
    //bool enableAudio();
*/
    //////////////////////////////////////////////////////////////////////////////
    // recorder
    virtual bool enableRecorder(const char* filePath);
    bool addNodeToRecording(int nodeType,bool lossy = false);

    //////////////////////////////////////////////////////////////////////////////
    // player
    virtual bool openFileRecording(const char* filePath);

    void playbackPlay(bool play);
    bool isPlaybackPlay() const;

    void setPlaybackSpeedPlayer(float speed);
    float playbackSpeedPlayer();

    void setRepeatPlayer(bool loop);
    bool repeatPlayer();

    int curFramePlayer();
    int framesPlayer();
    void seekPlayer(int offset);

    bool isEndPlayer();

    //////////////////////////////////////////////////////////////////////////////
    // access methods
    void setMirror(bool flag);
    bool mirror();

    //////////////////////////////////////////////////////////////////////////////
    // converter methods
    void convertRealWorldToProjective(float x,float y,float z,float* xp,float* yp,float* zp);
    void convertRealWorldToProjective(const float* pRealWorld,float* pProjective);

    void convertProjectiveToRealWorld(float x,float y,float z,float* xr,float* yr,float* zr);
    void convertProjectiveToRealWorld(const float* pProjective,float* pRealWorld);

    bool alternativeViewPointDepthToImage();
    bool setDepthToColor(bool enable);
    bool depthToColor();

    bool setDepthColorSyncEnabled(bool enable);

    ///////////////////////////////////////////////////////////////////////////
    // kinect motor
//    void moveKinect(float angle);

    ///////////////////////////////////////////////////////////////////////////
    // calibration
    void setUserCoordsys(float centerX,float centerY,float centerZ,
						float xDirX,float xDirY,float xDirZ,
						float zDirX,float zDirY,float zDirZ);
    bool setUserCoordsys(float mat[]);

    void resetUserCoordsys();
    bool hasUserCoordsys() const;
    float* getUserCoordsysTransMat();	// returns the 4x4 matrix
    void getUserCoordsysTransMat(float* mat);	// needs a 4x4 float array

    bool getOrigUserCoordsys(float* nullPointX,float* nullPointY,float* nullPointZ,
                             float* xAxisX,float* xAxisY,float* xAxisZ,
                             float* yAxisX,float* yAxisY,float* yAxisZ,
                             float* zAxisX,float* zAxisY,float* zAxisZ);

    void calcUserCoordsys(float* point);
    void calcUserCoordsys(float* x,float* y,float* z);
    void calcUserCoordsysMat(float* mat);

    void calcUserCoordsysBack(float* point);
    void calcUserCoordsysBack(float* x,float* y,float* z);
    void calcUserCoordsysBackMat(float* mat);


    void getUserCoordsys(float mat[]);	// needs a 4x4 float array
    void getUserCoordsysBack(float mat[]);	// needs a 4x4 float array

    ///////////////////////////////////////////////////////////////////////////
    // geometry helper functions

    static bool rayTriangleIntersection(float p[],
                                        float dir[],
                                        float vec0[],
                                        float vec1[],
                                        float vec2[],
                                        float hit[]);

    static int raySphereIntersection(float p[],
                                     float dir[],
                                     float sphereCenter[],
                                     float sphereRadius,
                                     float hit1[],float hit2[]);

    ///////////////////////////////////////////////////////////////////////////
    // callbacks

    // user
    SOPENNI_CB_VIRTUAL(NewUser,
                       int user)
    SOPENNI_CB_VIRTUAL(VisibleUser,
                       int user)
    SOPENNI_CB_VIRTUAL(OutOfSceneUser,
                       int user)
    SOPENNI_CB_VIRTUAL(LostUser,
                       int user)

    // hand
    SOPENNI_CB_VIRTUAL(NewHand,
                       int handId,const Vec3f& pos3d)
    SOPENNI_CB_VIRTUAL(TrackedHand,
                       int handId,const Vec3f& pos3d)
    SOPENNI_CB_VIRTUAL(LostHand,
                       int handId)

    // gesture
    SOPENNI_CB_VIRTUAL(NewGesture,
                       int gestureType)
    SOPENNI_CB_VIRTUAL(InProgressGesture,
                       int gestureType)
    SOPENNI_CB_VIRTUAL(AbortedGesture,
                       int gestureType)
    SOPENNI_CB_VIRTUAL(CompletedGesture,
                       int gestureType,const Vec3f& pos3d)


protected:

    enum LogOutMsg{
        MsgNode_End         = 0,
        MsgNode_Info        = 1,
        MsgNode_Error       = 2,
        MsgNode_Deprecated  = 3,
    };

    static void logOut(int msgType,const char* msg,...);	// must end with null
    static void logDeprecated(int msgType,const char* oldFunc,const char* newFunc);

    static std::vector<class ContextWrapper*> _classList;

    /*
    static KinectMotors _kinectMotors;
*/

    //////////////////////////////////////////////////////////////////////////////
    // create methods
    bool createDepth(bool force = true);
    bool createRgb(bool force = true);
    bool createIr(bool force = true);
    bool createScene(bool force = true);
    bool createUser(bool force = true);
    bool createGesture(bool force = true);
    bool createHand(bool force = true);

    void calcHistogram();
    void createDepthImage();
    void calcDepthImageRealWorld();
    void createIrImage();
    void calcUserData();

    bool updateDepthData();
    bool updateDepthImageData();
    bool updateDepthRealWorldData();

    void updateRgbData();

    void updateIrData();

    void updateSceneData();
    void updateSceneImageData();

    void updateUser();
    void updateHand();
    void updateGesture();

    void genFirstTime();
    void doNiteUserCb(const nite::UserData& userData);
    void doNiteHandCb(const nite::HandData& handData);
    void doNiteGestureCb(const nite::GestureData& handData);

    const nite::UserData* getUserData(int userId)
    {
        std::map<int,const nite::UserData*>::iterator findItr = _userIdTempList.find(userId);
        if(findItr != _userIdTempList.end())
            return (findItr->second);
        else
            return NULL;
    }
    bool copyRgbImage(int* map);

    bool                    _initFlag;
    bool                    _generatingFlag;
    bool                    _firstTimeUpdate;
    openni::Status          _rc;

    static bool             _globalContextFlag;


    // device
    openni::Device          _device;
    /*
        std::string             _deviceCreationInfo;
    */
    int                     _deviceIndex;
    static int              _deviceCount;
    int                     _nodes;

    // all streams
    openni::VideoStream**    _streams;
    openni::VideoFrameRef**  _streamsFrameRef;
    int                      _streamCount;
    bool                     _mirrorFlag;

    // depth
    openni::VideoStream         _depthStream;
    openni::VideoMode           _depthVideoMode;
    openni::VideoMode           _depthMapOutputMode;
    const openni::SensorInfo*   _depthSensorInfo;
    openni::VideoFrameRef       _depthFrame;
    float                       _pDepthHist[MAX_DEPTH];
    float                       _pDepthGamma[MAX_DEPTH];
    int                         _depthBufSize;
    int*                        _depthMapBuffer;
    float*                      _depthMapRealWorld;
    int*                        _pDepthImage;
    float                       _depthImageColor[3];
    int                         _depthImageColorMode;

    // cam image
    openni::VideoStream         _imageStream;
    openni::VideoMode           _imageVideoMode;
    openni::VideoMode           _imageMapOutputMode;
    const openni::SensorInfo*   _imageSensorInfo;
    openni::VideoFrameRef       _imageFrame;
    int                         _imgBufSize;
    int*                        _imgBuffer;

    // ir
    openni::VideoStream         _irStream;
    openni::VideoMode           _irVideoMode;
    openni::VideoMode           _irMapOutputMode;
    const openni::SensorInfo*   _irSensorInfo;
    openni::VideoFrameRef       _irFrame;
    int*                        _pIrImage;
    int                         _irBufSize;

    /*
    // scene
    xn::DepthGenerator	_sceneDepth;
    xn::SceneAnalyzer	_sceneAnalyzer;
    xn::SceneMetaData	_sceneMD;
    XnMapOutputMode		_sceneMapOutputMode;
    XnRGB24Pixel*		_pSceneImage;
    int                 _sceneBufSize;
    */

    // user
    bool                                _niteFlag;
    nite::UserTracker                   _userTracker;
    nite::UserTrackerFrameRef           _userFrameRef;
    openni::VideoMode                   _userVideoMode;
    std::map<int,const nite::UserData*> _userIdTempList;
    std::map<int,int>                   _userLastState;

    int*                                _pUserImage;
    int                                 _userBufSize;

    // hand / gesture
    nite::HandTracker                   _handTracker;
    nite::HandTrackerFrameRef           _handRefFrame;
    std::map<int,const nite::HandData*> _handIdTempList;
    std::map<int,int>                   _handLastState;

    std::map<int,const nite::GestureData*>  _gestureIdTempList;
    std::map<int,int>                       _gestureLastState;


    // recorder / player
    openni::Recorder        	_recorder;
    openni::PlaybackControl*    _player;
    bool                        _playerRepeat;
    bool                        _playerPlay;
    int                         _playerPlayFrame;

    ///////////////////////////////////////////////////////////////////////////
    // update flags, performance

    // timestamps
    unsigned long 	_depthMapTimeStamp;
    unsigned long 	_depthImageTimeStamp;
    unsigned long 	_depthRealWorldTimeStamp;

    unsigned long 	_imageTimeStamp;

    unsigned long 	_irTimeStamp;

    unsigned long 	_sceneTimeStamp;
    unsigned long	_sceneImageTimeStamp;

    unsigned long 	_userTimeStamp;
    unsigned long 	_gestureTimeStamp;
    unsigned long 	_handTimeStamp;

    unsigned long 	_updateTimeStamp;
    unsigned long 	_updateSubTimeStamp;

    ///////////////////////////////////////////////////////////////////////////
    // calibration
	bool										_userCoordsysFlag;
    Eigen::Transform<float,3,Eigen::Affine> 	_userCoordsysForwardMat;
    Eigen::Transform<float,3,Eigen::Affine> 	_userCoordsysRetMat;
    float                                       _userCoordsysMat[16];
	Eigen::Vector3f								_userCoordsysNullPoint;
	Eigen::Vector3f								_userCoordsysXAxis;
	Eigen::Vector3f								_userCoordsysYAxis;
	Eigen::Vector3f								_userCoordsysZAxis;
	
public:
    inline unsigned long depthMapTimeStamp(){ return _depthMapTimeStamp; }
    inline unsigned long depthImageTimeStamp(){ return _depthImageTimeStamp; }
    inline unsigned long depthRealWorldTimeStamp(){ return _depthRealWorldTimeStamp; }
    inline unsigned long imageTimeStamp(){ return _imageTimeStamp; }
    inline unsigned long irTimeStamp(){ return _irTimeStamp; }
    inline unsigned long sceneTimeStamp(){ return _sceneTimeStamp; }
    inline unsigned long userTimeStamp(){ return _userTimeStamp; }
    inline unsigned long handTimeStamp(){ return _handTimeStamp; }

    inline unsigned long updateTimeStamp(){ return _updateTimeStamp; }
    inline unsigned long updateSubTimeStamp(){ return _updateSubTimeStamp; }

    inline bool newThreadData()
    {
        return(_updateTimeStamp != _updateSubTimeStamp);
    }

    inline bool newDepthMapThreadData()
    {
        return(_depthMapTimeStamp != _updateTimeStamp) && newThreadData();
    }

    inline bool newDepthImageThreadData()
    {
        return(_depthImageTimeStamp != _updateTimeStamp) && newThreadData();
    }

    inline bool newDepthRealWorldThreadData()
    {
        return(_depthRealWorldTimeStamp != _updateTimeStamp) && newThreadData();
    }

    ///////////////////////////////////////////////////////////////////////////
    // threading

private:

    void updateSub();
    bool updateOpenNI(bool force = false);
    void run();

    boost::shared_ptr<boost::thread>	_thread;
    boost::mutex                        _mainMutex;
    bool                                _threadRun;
    int									_threadMode;

    boost::mutex                        _dataMutex;

    boost::mutex                        _depthMutex;
    boost::mutex                        _imgMutex;
    boost::mutex                        _irMutex;



};


}; // namespace sOpenNI




#endif // _CONTEXWRAPPER_H_
