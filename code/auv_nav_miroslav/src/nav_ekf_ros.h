#ifndef _NAV_EKF_ROS_H_
#define _NAV_EKF_ROS_H_
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

	//create log file name from time
	FILE *pLogInput;
	FILE *pLogOutput;
	FILE *pLogObser;	
	FILE *pLogGps;
	FILE *pLogParams;
	FILE *pOldNav;

class navigationRos: public osl_core::ModuleCore
// class that uses nav_sts and lbl_sts ROS messages for navigation
// for algorithm testing purposes, possible to use, however, old nav
// needs to be started in order to have the input
{
public:
	navigationRos();// left empty by now
	~navigationRos();
private:
	//// overriding existing functions from ModuleCore
	bool init(int argc, char * argv[]);
	void doWork();
	virtual void cleanup();
	////
	// navigation filter params
	bool gotHeading, gotGpsFix, fakeGPS, firstDepthMsgReceived,gotLBL;
	bool useGPSForFiltering;
	int sendMsgPeriod;
	int ekfPeriod;
	filterMode ekfMode;
	// timers to measure filter trigger and elapsed time
    	osl_core::TimerMillis m_timer;
    	// timer for Kalman filter (used to count the defined interval and threshold kalman execution in case it is defined to run each dT)
    	osl_core::TimerMillis m_timer_ekf_TRIGGER; 
    	// timer used to measure elapsed time between Kalman executions
    	osl_core::Chrono *m_timer_ekf_MES;  
    	// ekf filter instance for navigation module
    	extendedKalmanFilter5DOF EKF5DOF; // ext kalman filter instance (derived from OceanLIB -> olibKalmanFilter library, defined in filt.h)
    	
    	sensorType MeasurementSensor; // which sensor was used for particular measurement
	
	///////////////////////////         ROS         ////////////////////////////////////////
	// essential reason for naming application ros - it takes ros messages
	ros::Subscriber  m_rosInputNavSub;   // subscribing to an input ros message (can be emulated by playing bag files) 
	ros::Subscriber  m_rosInputLblNavSub;// subscribing to an input lbl message 
	ros::Subscriber  m_rosInputGpsSub;//gps
	auv_msgs::NavSts m_rosInputNavMsg;   // variable to store an input message from lbl or nav

	ros::Publisher   m_rosFiltNavPub;  // publisher: output ros message with filtered nav
	
	auv_msgs::NavSts m_rosFiltNavMsg;  // store output filtering message
	// rviz visualisation variables
	std::string tfBaseLinkFrame_;
	std::string tfMapFrame_;
	nav_msgs::Odometry m_rosFiltOdometryMsg; // odometry message of filtered values - to show in rviz
	ros::Publisher m_rosFiltOdometryPub;  // publisher: odometry for visualising filtered values in rviz
	// GPS tools lat, lon...
	osl_core::GlobalCoordConverter globalCoordConverter;
    	osl_core::T_LLD LLD;
    	float originNORTH, originEAST;
    	

    	void handleROSnavMsg(const auv_msgs::NavStsConstPtr & msg);
    	void handleROSlblMsg(const nav_msgs::OdometryConstPtr & msg);
        void handleGpsMsg(const auv_msgs::GPSConstPtr & msg);    	
    	void sendROSNavSts();
    	void sendROSFiltOdometry();	
	////////////////////////////////////////////////////////////////////////////////////////
	
	
	// just in case left... should be deleted
	//osl_core::GlobalCoordConverter globalCoordConverter; // for conversion
    	//osl_core::T_LLD LLD;
    	//osl_core::T_NED NED;
    	//osl_core::T_NED originNED;	
		
	// measured values //
	float roll, pitch, yaw;
	float pitchRate, rollRate, yawRate;
	float surgeVelocity, swayVelocity, heaveVelocity;
	float north, east, altitude, depth;
	// NAV- from existing nav_sts, LBL - from lbl_sts
	float northNAV, northLBL, northGPS,  eastNAV, eastLBL, eastGPS; // auxilliary
	// values after filtering //
	float northFiltered, eastFiltered, depthFiltered, altitudeFiltered;
	float surgeVelocityFiltered, swayVelocityFiltered, heaveVelocityFiltered;
	float pitchFiltered, yawFiltered, pitchRateFiltered, yawRateFiltered;
	
	olibVector<double64> ObservationVec; // ObservationVec joins together Measurements from different devices that occured within some interval
	olibVector<stateID>  ObservationStates; // which states are contained in ObservationVec
	
	olibVector<double64> MeasurementVec; // Measurement comes from particular device, deifned in advance depending on the device
	olibVector<stateID>  MeasurementStates; // states contained in MeasurementVec
	void addMeasurementToObservation(olibVector<double64> & ObservationVec, olibVector<stateID> & ObservationStates, olibVector<double64> & MeasurementVec, olibVector<stateID> & MeasurementStates);
	void storeFilteringResults(olibVector<double64> filteredStateVec, double64 elapsedPeriod);
        void storeLogFile(double64 interval, 
                  olibVector<double64> previousState, 
                  olibMatrix<double64> previousP, 
                  olibVector<double64> Obser, 
                  olibVector<stateID> ObserStates, 
                  olibVector<double64> filteredState, 
                  olibMatrix<double64> filteredP);
};
#endif
