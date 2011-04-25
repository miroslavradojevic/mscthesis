#ifndef _NAV_H_
#define _NAV_H_
// ================================================================== includes
#include <vector>
// ModuleCore include should be above ROS and OceanSHELL includes.
#include <osl_core/ModuleCore.h>
#include <osl_core/GlobalCoordConverter.h>
#include <osl_core/TimerMillis.h>
#include <osl_core/Chrono.h>

#include <OceanLIB/Processing/olibKalmanFilter.h>
#include <OceanLIB/Processing/olibKalmanMeasure.h>
#include <OceanLIB/Abstraction/olibVector.h>
#include <OceanLIB/Abstraction/olibMatrix.h>

#include <auv_msgs/NavSts.h>
#include <auv_msgs/GPS.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "filt.h"

		// global variables used to record the data
		FILE *pInputMessages;
        FILE *pPredictedState;
        FILE *pCorrectedState;
        FILE *pPredictedP;
        FILE *pCorrectedP;


class navigationRos: public osl_core::ModuleCore
// class that uses nav_sts and lbl_sts ROS messages for navigation
// for algorithm testing purposes
{
public:
	navigationRos();// left empty by now
	~navigationRos();
	bool LBLjustHappened;
private:
	// overriding existing functions from ModuleCore
	bool init(int argc, char * argv[]);
	void doWork();
	virtual void cleanup();
	//*****
	osl_core::TimerMillis m_timer;
	
	ros::Subscriber  m_rosInputNavSub;   // subscribing to an input ros message (can be emulated by playing bag files)
	ros::Subscriber  m_rosInputLblNavSub;// subscribing to an input lbl message 
	auv_msgs::NavSts m_rosInputNavMsg;   // variable to store an input message from lbl or nav

	ros::Publisher   m_rosFiltNavPub;  // publisher: output ros message with filtered nav
	auv_msgs::NavSts m_rosFiltNavMsg;  // store output filtering message

	extendedKalmanFilter5DOF EKF5DOF; // ext kalman filter instance (derived from OceanLIB -> olibKalmanFilter library, defined in filt.h)
	                                  // extended kalman implementation  
	                                  // fixed: 5dof, 11 states, 5 noise, constant speed model
	//uint32 initial_measure_size;
	//olibKalmanMeasure measurementEKF5DOF(1, *EKF5DOF); // class for measurement, derived from library, goes together with Kalman filter

    	float pitch, yaw;
    	float pitchRate, yawRate; 
    	float surgeVelocity, swayVelocity, heaveVelocity;
    	float north, east, altitude, depth;
	
    	//bool firstDvlMsgReceived, firstGyroMsgReceived, firstTcmMsgReceived, firstDepthMsgReceived;
    	bool firstMsgReceived;
    	bool gotGpsFix;
    	bool fakeGPS;
    	int whoWasIt;
    	void handleROSnavMsg(const auv_msgs::NavStsConstPtr & msg);
    	void handleROSlblMsg(const auv_msgs::NavStsConstPtr & msg);
    	void filtering(const auv_msgs::NavStsConstPtr & msg, extendedKalmanFilter5DOF & filter);
    	void sendROSNavSts();
};
#endif
