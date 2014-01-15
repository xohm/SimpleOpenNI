# -----------------------------------------------------------------------------
# SimpleOpenNIMain
# -----------------------------------------------------------------------------
# Processing Wrapper for the OpenNI/Kinect library 2.0
# prog:  Max Rheiner / Interaction Design / Zhdk / http://iad.zhdk.ch/
# -----------------------------------------------------------------------------

%module(directors="1") SimpleOpenNI

%{
#include "ContextWrapper.h"
/*
#include "NITE_Helpers.h"
*/
#include <string>
#include <vector>

%}

%include "arrays_java.i"
%include "cpointer.i"
%include "typemaps.i"
%include "carrays.i"

%apply int[] { int * };
%apply float[] { float * };


# ----------------------------------------------------------------------------
# OpenNI

#%include "SimpleOpenNI.i"


# ----------------------------------------------------------------------------
# stl

%include "std_vector.i"
%include "std_string.i"
 
%template(IntVector)	std::vector<int>;
/*
%template(Vector3D)		std::vector<XnVector3D>;
%template(Point3D)		std::vector<XnPoint3D>;
*/

%template(StrVector)	std::vector<std::string>;
 
 
# ----------------------------------------------------------------------------
# ContextWrapper

//typedef const float* constPFloat;

namespace sOpenNI{


class Vec3f
{
public:
    Vec3f(float x,float y,float z);
    Vec3f(const Vec3f& copy);

    float x() const;
    float y() const;
    float z() const;

    //onst float* get();
};

        /*
# typedef XnPoint3D*	XnPoint3DArrayX;
# JAVA_ARRAYSOFCLASSES(XnPoint3DArrayX)
# %apply XnPoint3D[] {XnPoint3DArrayX};
# */

# %constant int USERS_ALL		= 0;

%constant int SKEL_HEAD			= nite::JOINT_HEAD;
%constant int SKEL_NECK			= nite::JOINT_NECK;
%constant int SKEL_TORSO		= nite::JOINT_TORSO;

%constant int SKEL_LEFT_SHOULDER	= nite::JOINT_LEFT_SHOULDER;
%constant int SKEL_LEFT_ELBOW		= nite::JOINT_LEFT_ELBOW;
%constant int SKEL_LEFT_HAND		= nite::JOINT_LEFT_HAND;
%constant int SKEL_LEFT_FINGERTIP	= nite::JOINT_HEAD;

%constant int SKEL_RIGHT_SHOULDER	= nite::JOINT_RIGHT_SHOULDER;
%constant int SKEL_RIGHT_ELBOW		= nite::JOINT_RIGHT_ELBOW;
%constant int SKEL_RIGHT_HAND		= nite::JOINT_RIGHT_HAND;
%constant int SKEL_RIGHT_FINGERTIP	= nite::JOINT_HEAD;

%constant int SKEL_LEFT_HIP		= nite::JOINT_LEFT_HIP;
%constant int SKEL_LEFT_KNEE		= nite::JOINT_LEFT_KNEE;
%constant int SKEL_LEFT_FOOT		= nite::JOINT_LEFT_FOOT;

%constant int SKEL_RIGHT_HIP		= nite::JOINT_RIGHT_HIP;
%constant int SKEL_RIGHT_KNEE		= nite::JOINT_RIGHT_KNEE;
%constant int SKEL_RIGHT_FOOT		= nite::JOINT_RIGHT_FOOT;

%constant int GESTURE_WAVE		= nite::GESTURE_WAVE;
%constant int GESTURE_CLICK		= nite::GESTURE_CLICK;
%constant int GESTURE_HAND_RAISE	= nite::GESTURE_HAND_RAISE;

%constant int NODE_NONE			= Node_None;		
%constant int NODE_DEPTH		= Node_Depth;		
%constant int NODE_IMAGE		= Node_Image;		
%constant int NODE_IR			= Node_Ir;		
%constant int NODE_SCENE		= Node_Scene;		
%constant int NODE_USER			= Node_User;		
%constant int NODE_HANDS		= Node_Hand;
%constant int NODE_GESTURE		= Node_Gesture;		
%constant int NODE_RECORDER		= Node_Recorder;		
%constant int NODE_PLAYER		= Node_Player;		

# %constant int CODEC_NONE		= XN_CODEC_NULL;
# %constant int CODEC_UNCOMPRESSED	= XN_CODEC_UNCOMPRESSED;		
# %constant int CODEC_JPEG		= XN_CODEC_JPEG;
# %constant int CODEC_16Z		= XN_CODEC_16Z;
# %constant int CODEC_16Z_EMB_TABLES	= XN_CODEC_16Z_EMB_TABLES;		
# %constant int CODEC_CODEC_8Z		= XN_CODEC_8Z;		
# 
# %constant int RECORD_MEDIUM_FILE	= XN_CODEC_NULL;
# 
# %constant int PLAYER_SEEK_SET           = XN_PLAYER_SEEK_SET;
# %constant int PLAYER_SEEK_CUR           = XN_PLAYER_SEEK_CUR;
# %constant int PLAYER_SEEK_END           = XN_PLAYER_SEEK_END;


%constant int IMG_MODE_DEFAULT          = DepthImgMode_Default;
%constant int IMG_MODE_RGB_FADE         = DepthImgMode_RgbFade;

%constant int RUN_MODE_DEFAULT		= RunMode_Default;
%constant int RUN_MODE_SINGLE_THREADED	= RunMode_SingleThreaded;		
%constant int RUN_MODE_MULTI_THREADED	= RunMode_MultiThreaded;		

%feature("director") ContextWrapper;
class ContextWrapper
{
public:
 
    ContextWrapper();
    virtual ~ContextWrapper();

    int version();

    static void chdir(const char* dir);
    static std::string getcwd();

    static bool initContext();
    bool init(int runMode=RunMode_MultiThreaded);
    bool init(const char* record,int runMode=RunMode_SingleThreaded);
    bool init(int deviceIndex,int runMode);

    int nodes();

    static int deviceCount();
    static int deviceNames(std::vector<std::string>* nodeNames);
    int deviceIndex() const;

    bool isInit();
    void close();

    virtual bool enableDepth();
    virtual bool enableDepth(int width,int height,int fps);

    virtual bool enableRGB();
    virtual bool enableRGB(int width,int height,int fps);

    virtual bool enableIR();
    virtual bool enableIR(int width,int height,int fps);

# 	virtual bool enableScene();
# 	virtual bool enableScene(int width,int height,int fps);

    virtual bool enableUser();

    virtual bool enableHand();

#         virtual bool enableGesture();

    virtual void update();
    static void updateAll();

    int     depthWidth();
    int     depthHeight();
    int     depthImage(int* map);

    void    setDepthImageColor(int r,int g,int b);
    void    setDepthImageColorMode(int mode);
    int     depthImageColorMode();

    int     depthMapSize();
    int     depthMap(int* map);

    int     depthMapRealWorld(float* map);
    //float*  depthMapRealWorldA();

    float hFieldOfView();
    float vFieldOfView();

    int rgbWidth();
    int rgbHeight();
    int rgbImage(int* map);

    int irWidth();
    int irHeight();
    int irMap(int* map);
    int irImage(int* map);

# 	int sceneWidth();
# 	int sceneHeight();
# 	int sceneMap(int* map);			
# 	int sceneImage(int* map);
# 	void getSceneFloor(XnVector3D* point,
# 					   XnVector3D* normal);

    int	userWidth();
    int	userHeight();

    bool	getCoM(int user, float*  com);
    bool	getBoundingBox(int user, float*  boudingbox);


    int	getNumberOfUsers();
    int	getUsers(std::vector<int>* userList);

    int userMap(int* map);
    int userImage(int* map);

#         int	getUserPixels(int user,int* userSceneData);
# 
# 	bool	getUserPostition(int user, XnBoundingBox3D*  pPosition );
# 
# 	
# 	bool	isCalibratedSkeleton(int user);
# 	bool	isCalibratingSkeleton(int user);
# 	void	requestCalibrationSkeleton(int user, bool force);
# 	void	abortCalibrationSkeleton(int user);
# 	
# 	bool	saveCalibrationDataSkeleton(int user,int slot);
# 	bool	loadCalibrationDataSkeleton(int user,int slot);
# 	void	clearCalibrationDataSkeleton(int slot);
# 	bool	isCalibrationDataSkeleton(int slot);
# 
#         bool	saveCalibrationDataSkeleton(int user,const char* calibrationFile);
#         bool	loadCalibrationDataSkeleton(int user,const char* calibrationFile);
# 
# 	void	setSmoothingSkeleton(float factor);
# 	
    bool	isTrackingSkeleton(int user);
    void	startTrackingSkeleton(int user);
    void	stopTrackingSkeleton(int user);
# 
# 	void	startPoseDetection(const char* pose,int user);
# 	void	stopPoseDetection(int user);

    float	getJointPositionSkeleton(int user,int joint,float* jointPos);
    float	getJointOrientationSkeleton(int user,
                                        int joint,
                                        float* jointOrientation);


    int     	startTrackingHand(float*pos);
    void	stopTrackingHand(int handId);
    void	stopTrackingAllHand();
    void	setSmoothingHand(float smoothingFactor);
    float	getSmoothingHand();

    void        startGesture(int gesture);
    void        endGesture(int gesture);

    virtual bool enableRecorder(const char* filePath);
    bool addNodeToRecording(int nodeType,bool lossy);

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


    void setMirror(bool flag);
    bool mirror();

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
    //void moveKinect(float angle);

    ///////////////////////////////////////////////////////////////////////////
    // calibration
    void setUserCoordsys(float centerX,float centerY,float centerZ,
                         float xDirX,float xDirY,float xDirZ,
                         float zDirX,float zDirY,float zDirZ);
    void resetUserCoordsys();
    bool hasUserCoordsys() const;

    //float* getUserCoordsysTransMat();	// returns the 4x4 matrix
    void getUserCoordsysTransMat(float* mat);// needs a 4x4 float array

    void calcUserCoordsys(float* point);
    void calcUserCoordsysMat(float* mat);

    void calcUserCoordsysBack(float* point);
    void calcUserCoordsysBackMat(float* mat);

    void getUserCoordsys(float mat[]);	// needs a 4x4 float array
    void getUserCoordsysBack(float mat[]);	// needs a 4x4 float array

# /*
#         bool getOrigUserCoordsys(float* nullPointX,float* nullPointY,float* nullPointZ,
#                                  float* xAxisX,float* xAxisY,float* xAxisZ,
#                                  float* yAxisX,float* yAxisY,float* yAxisZ,
#                                  float* zAxisX,float* zAxisY,float* zAxisZ);
# */
#         ///////////////////////////////////////////////////////////////////////////
#         // geometry helper functions
# 
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
# 
#         ///////////////////////////////////////////////////////////////////////////
# 	// XnVSessionMananger
# 	XnVSessionManager* createSessionManager(const XnChar* strUseAsFocus, const XnChar* strUseAsQuickRefocus,
# 											 xn::HandsGenerator*	pTracker = NULL, 
# 											 xn::GestureGenerator*	pFocusGenerator = NULL,
# 											 xn::GestureGenerator*	pQuickRefocusGenerator = NULL);
# 
# 	void update(XnVSessionManager* sessionManager);
# 
# 
# 
# 	///////////////////////////////////////////////////////////////////////////
# 	// time stamps
# 
    unsigned long depthMapTimeStamp();
    unsigned long depthImageTimeStamp();
    unsigned long depthRealWorldTimeStamp();
    unsigned long imageTimeStamp();
    unsigned long irTimeStamp();
    unsigned long sceneTimeStamp();
    unsigned long userTimeStamp();
    unsigned long handTimeStamp();

    unsigned long updateTimeStamp();
    unsigned long updateSubTimeStamp();

protected:

    virtual void onNewUserCb(int userId);
    virtual void onLostUserCb(int userId);
    virtual void onVisibleUserCb(int userId);
    virtual void onOutOfSceneUserCb(int userId);

    virtual void onNewHandCb(int handId,const Vec3f& pos3d);
    virtual void onTrackedHandCb(int handId,const Vec3f& pos3d);
    virtual void onLostHandCb(int handId);

    virtual void onNewGestureCb(int gestureType);
    virtual void onInProgressGestureCb(int gestureType);
    virtual void onAbortedGestureCb(int gestureType);
    virtual void onCompletedGestureCb(int gestureType,const Vec3f& pos3d);

#         virtual void onStartCalibrationCb(unsigned int userId);
# 	virtual void onEndCalibrationCb(unsigned int userId,bool successFlag);
# 
# 	virtual void onStartPoseCb(const char* strPose, unsigned int user);
# 	virtual void onEndPoseCb(const char* strPose, unsigned int user);
# 
# 	virtual void onCreateHandsCb(unsigned int nId, const XnPoint3D* pPosition, float fTime);
# 	virtual void onUpdateHandsCb(unsigned int nId, const XnPoint3D* pPosition, float fTime);
# 	virtual void onDestroyHandsCb(unsigned int nId, float fTime);
# 
# 	virtual void onRecognizeGestureCb(const char* strGesture, const XnPoint3D* pIdPosition,const XnPoint3D* pEndPosition);
# 	virtual void onProgressGestureCb(const char* strGesture, const XnPoint3D* pPosition,float fProgress);
# 
# 	virtual void onStartSessionCb(const XnPoint3D& ptPosition);
# 	virtual void onEndSessionCb();
# 	virtual void onFocusSessionCb(const XnChar* strFocus, const XnPoint3D& ptPosition, XnFloat fProgress);
# 
};

};

# 
# 
# 
# 
# # ----------------------------------------------------------------------------
# # NITE
# 
# #%include "SimpleNite.i"
