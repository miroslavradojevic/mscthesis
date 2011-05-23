#ifndef _NAV_ROS_H_
#define _NAV_ROS_H_
// ================================================================== includes
#include <vector>
#include <sstream>
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

		// global variables used to record the data for plotting
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
private:
	// overriding existing functions from ModuleCore
	bool init(int argc, char * argv[]);
	void doWork();
	virtual void cleanup();
	
	int COUNT_FILT;
	int sendMsgPeriod;
	int ekfPeriod;
    	osl_core::TimerMillis m_timer;
    	// timer for Kalman filter (used to count the defined interval and threshold kalman execution in case it is defined to run each dT)
    	osl_core::TimerMillis m_timer_ekf_TRIGGER; 
    	// timer used to measure elapsed time between Kalman executions
    	osl_core::Chrono *m_timer_ekf_MES;  
    	filterMode ekfMode;
    	extendedKalmanFilter5DOF EKF5DOF; // ext kalman filter instance (derived from OceanLIB -> olibKalmanFilter library, defined in filt.h)
    	sensorType MeasurementSensor;
	
	///////////////////////////         ROS         ////////////////////////////////////////
	ros::Subscriber  m_rosInputNavSub;   // subscribing to an input ros message (can be emulated by playing bag files)
	ros::Subscriber  m_rosInputLblNavSub;// subscribing to an input lbl message 

	auv_msgs::NavSts m_rosInputNavMsg;   // variable to store an input message from lbl or nav

	ros::Publisher   m_rosFiltNavPub;  // publisher: output ros message with filtered nav
	
	auv_msgs::NavSts m_rosFiltNavMsg;  // store output filtering message

    	void handleROSnavMsg(const auv_msgs::NavStsConstPtr & msg);
    	void handleROSlblMsg(const nav_msgs::OdometryConstPtr & msg);
    	void sendROSNavSts();	
	////////////////////////////////////////////////////////////////////////////////////////
	
	osl_core::GlobalCoordConverter globalCoordConverter; // for conversion
    	osl_core::T_LLD LLD;
    	osl_core::T_NED NED;
    	osl_core::T_NED originNED;	
	
	    // navigation variables //
	float roll, pitch, yaw;
	float pitchRate, rollRate, yawRate;
	float surgeVelocity, swayVelocity, heaveVelocity;
	float north, east, northNAV, northLBL,  eastNAV, eastLBL,  altitude, depth;
	float northFiltered, eastFiltered, depthFiltered, altitudeFiltered;
	float surgeVelocityFiltered, swayVelocityFiltered, heaveVelocityFiltered;
	float pitchFiltered, yawFiltered, pitchRateFiltered, yawRateFiltered;
	
	olibVector<double64> ObservationVec; // Observation joins together Measurements from different devices
	olibVector<stateID>  ObservationStates; // which states are contained in observation
	
	olibVector<double64> MeasurementVec; // Measurement comes from particular device
	olibVector<stateID>  MeasurementStates;
	void addMeasurementToObservation(olibVector<double64> & ObservationVec, olibVector<stateID> & ObservationStates, olibVector<double64> & MeasurementVec, olibVector<stateID> & MeasurementStates);
	void storeFilteringResults(olibVector<double64> filteredStateVec);

};
#endif
