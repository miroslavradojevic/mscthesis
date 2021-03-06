/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Heriot-Watt University, UK.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Heriot-Watt University nor the names of
*     its contributors may be used to endorse or promote products
*     derived from this software without specific prior written
*     permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*  Author: Joel Cartwright, Nicolas Valeyrie, Pedro Patron
*
*********************************************************************/

#ifndef _NAV_EKF_H_
#define _NAV_EKF_H_

// ================================================================== includes

#include <vector>

// ModuleCore include should be above ROS and OceanSHELL includes.
#include <osl_core/ModuleCore.h>
#include <osl_core/GlobalCoordConverter.h>
#include <osl_core/TimerMillis.h>
#include <osl_core/Chrono.h>

#ifdef USING_ROS
#include <auv_msgs/NavSts.h>
#include <auv_msgs/GPS.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#endif // USING_ROS

#ifdef USING_OSH
#include <OceanALI/Message/aliMsg_NavSts.h>
#include <OceanALI/Message/aliMsg_GyroSts.h>
#include <OceanALI/Message/aliMsg_DepthSts.h>
#include <OceanALI/Message/aliMsg_GPSSts.h>
#include <OceanALI/Message/aliMsg_TimeDelay.h>

#include <DVLMsg.h>
#include <Tcm2Msg.h>
#endif // USING_OSH

#include "filt.h" // ekf filter class definition is there
	//*** create log files (later to be expelled)
	FILE *pLogInput;
	FILE *pLogOutput;
	FILE *pLogObser;	
	FILE *pLogGps;
	FILE *pLogParams;
	FILE *pOldNav;
	// file used to record the data in .csv log files
	char * removeWhiteSpaces(char * ); // to remove white spaces when creating a time based log file name 

#define navMaxNumTransponders 4

float f_wrapAngleDeg (float);
float f_wrapAngleRad (float);

class LBLOutlier
{
public:
	LBLOutlier() {
		memset(&offsetNED, 0, sizeof(offsetNED));
		offsetDistance = 0;
	}

	LBLOutlier(osl_core::T_NED offsetNED, float offsetDistance) {
		this->offsetNED = offsetNED;
		this->offsetDistance = offsetDistance;
	}

	osl_core::T_NED offsetNED;
	float offsetDistance;

};

bool operator<(LBLOutlier a, LBLOutlier b) {
	return (a.offsetDistance < b.offsetDistance);
}

class navigation: public osl_core::ModuleCore
{
public:
	
	navigation();
//	~navigation();

private:
	osl_core::TimerMillis m_timer;
	osl_core::TimerMillis m_timer_dvl;	// timer for dvl //
	// timer for Kalman filter (used to count the defined interval and threshold kalman execution in case it is defined to run each dT)
    	osl_core::TimerMillis m_timer_ekf_TRIGGER;
	// timer used to measure elapsed time between Kalman executions
    	osl_core::Chrono *m_timer_ekf_MES;
    	
	// messages //
#ifdef USING_OSH
    	aliMsg_NavSts navMsg;
    	TCM2Msg tcmMsg, oldTcmMsg;
    	aliMsg_GyroSts gyroMsg, oldGyroMsg;
    	DVLMsg dvlMsg, oldDvlMsg;
    	aliMsg_DepthSts depthMsg, oldDepthMsg;
    	aliMsg_GPSSts gpsMsg;
    	aliMsg_TimeDelay lblMsg;
#endif // ifdef USING_OSH

#ifdef USING_ROS
	
	ros::Publisher m_rosNavPub;
	ros::Publisher m_rosOdometryPub;
	ros::Publisher m_rosGPSOffsetPub;
	ros::Publisher m_rosGPSPub;
	ros::Publisher m_rosLBLPub;
	ros::Publisher m_rosLBLOffsetPub;
	ros::Publisher m_rosFiltNavPub;  // publisher: output ros message with filtered nav
	ros::Publisher m_rosFiltOdometryPub;  // publisher: odometry for visualising filtered values in rviz

	auv_msgs::NavSts m_rosNavMsg;
	nav_msgs::Odometry m_rosOdometryMsg;
	nav_msgs::Odometry m_rosGPSOffsetMsg;
	nav_msgs::Odometry m_rosLBLOffsetMsg;
	auv_msgs::GPS m_rosGPSMsg;
	auv_msgs::NavSts m_rosLBLMsg;
	auv_msgs::NavSts m_rosFiltNavMsg;        // store output filtering message
	nav_msgs::Odometry m_rosFiltOdometryMsg; // odometry message of filtered values - to show in rviz

	//ros::Subscriber  m_rosInputNavSub;  // to compare with the old nav
	
    	std::string tfMapFrame_;
    	std::string tfOdomFrame_;
    	std::string tfBaseLinkFrame_;
    	std::string tfGPSFrame_;
    	std::string tfLBLFrame_;

#endif // ifdef USING_ROS

    	// navigation variables (measurement & dead reckoning) //
    	float roll, pitch, yaw; 
    	float pitchRate, rollRate, yawRate;
   	float surgeVelocity, swayVelocity, heaveVelocity;
    	float northVelocity, eastVelocity, depthVelocity;
    	float north, east, altitude, depth;
	    
	// values after filtering //
	float northFiltered, eastFiltered, depthFiltered, altitudeFiltered;
	float surgeVelocityFiltered, swayVelocityFiltered, heaveVelocityFiltered;
	float pitchFiltered, yawFiltered, pitchRateFiltered, yawRateFiltered;    

	
    float initialHeading;  /* initial heading provides by the compass */
    float DVL_position_offset;
    int sendMsgPeriod;
    int delayDvlLock;

    	// ekf filter instance for navigation module
    	extendedKalmanFilter5DOF EKF5DOF; // ext kalman filter instance (derived from OceanLIB -> olibKalmanFilter library, defined in filt.h)
	T_EKF5DOF_PARAMS EKF5DOF_params; // ekf parameters 
	olibVector<double64> ObservationVec; // ObservationVec joins together Measurements from different devices that occured within some interval
	olibVector<stateID>  ObservationStates; // which states are contained in ObservationVec
	
	olibVector<double64> MeasurementVec; // Measurement comes from particular device, deifned in advance depending on the device
	olibVector<stateID>  MeasurementStates; // states contained in MeasurementVec
	sensorType MeasurementSensor; // which sensor was used for particular measurement

	int ekfPeriod; // ms
	filterMode ekfMode;

    osl_core::GlobalCoordConverter globalCoordConverter;
    bool fixedOrigin;
    osl_core::T_LLD originLLD;	/* lat and lon origin for the local NED frame */

    float fogOffset;
    float transverseVelocity;		/* velocity returned by the dvl */
    float longitudinalVelocity;		/* velocity returned by the dvl */

    /* flags */
    bool gotHeading;
    bool gotPosition;
    bool fogOffsetSet;
    bool firstDvlMsgReceived, firstGyroMsgReceived, firstTcmMsgReceived, firstDepthMsgReceived;
    bool gotGpsFix;
    bool fakeGPS;
    osl_core::T_LLD fakeGPSLLD;
    bool lostDvlLock;

    //bool useGPSForFiltering;

    uint8 navStatus;	/* status of navigation module set in the nav message */
    uint8 navMode;		/* navigation mode */

    // global origin from gps //
    double latitude;
    double latitudeFiltered;
    double longitude;
    double longitudeFiltered;

    /* global position from LBL system, it supports up to 4 transponder locations */
    bool useLBL;	// whether or not we listen to LBL messages
    float speedOfSound;
    bool applyLBLSolution;
    bool stableLBLSolution;
    float maxLBLStableDistance;
    int maxLBLOutliersFromStable;
    std::vector<LBLOutlier> LBLOutlierHistory;

    bool			transponderNEDValid;
    bool			transponderEnable[navMaxNumTransponders];
    bool			transponderActive[navMaxNumTransponders];
    osl_core::T_LLD transponderLLD[navMaxNumTransponders];
    osl_core::T_NED transponderNED[navMaxNumTransponders];
    double          transponderRange[navMaxNumTransponders];
 //   osl_core::T_LLD LBLcandidate[maxNumTransponders][maxNumTransponders][2];
    osl_core::T_LLD LBLbasedLLD;
    osl_core::T_NED LBLbasedNED;
    double			LBLdistance;
    osl_core::Chrono *LBLfixAge;
    bool calculateTransponderNED();
    bool calculateLBLbasedLLD(osl_core::T_LLD & solutionLLD, osl_core::T_NED & offset2current, double & distance2current);
    bool calculateLBLbasedLLDfromTransponderPair(
    		const unsigned int & first, const unsigned int & second, osl_core::T_LLD & solutionLLD, osl_core::T_NED & offset2current, double & distance2current);

    bool compensateDVLPitchRoll;

    osl_core::TimerMillis diagTimer;
    timeout_t diagPeriod;

     // These override virtual functions in NessieCore
    /**
     * Custom module initialisation.
     */
    bool init(int argc, char * argv[]);

    void jumpToGlobalPosition(double newLat, double newLon);

    void jumpPositionByOffset(float northOffset, float eastOffset);

#ifdef USING_OSH
    /**
     * Message handling for the module.
     */
    void handleMsg(oshMsg_Generic &msg);

    void handleTimeDelayMsg(aliMsg_TimeDelay lblMsg);
#endif // USING_OSH
	
     	//
     	// Function for non-message handling work performed by the module.
     	// Return false normally, or true if there is "more work to do".
     	//
    	void doWork();

	void updateGlobalPosition();
	void updateGlobalFilteredPosition();// same just for filtered values

	void sendROSNavSts(); // nav 
	void sendROSFiltNavSts(); // filtered nav
	//void handleOldNavMsg(const auv_msgs::NavStsConstPtr & msg); // for comparison	

	void sendROSOdometry();  // odometry for rviz
	void sendROSFiltOdometry(); // filt odometry for rviz

	void sendALINavSts();

	//void sendSeetrackNavMsg();
	// functions used in filtering
	void addMeasurementToObservation(olibVector<double64> & ObservationVec, olibVector<stateID> & ObservationStates, olibVector<double64> & MeasurementVec, olibVector<stateID> & MeasurementStates);
	void storeFilteringResults(olibVector<double64> filteredStateVec);
	void storeLogFile(double64 interval, olibVector<double64> previousState, olibMatrix<double64> previousP, olibVector<double64> Obser, olibVector<stateID> ObserStates, olibVector<double64> filteredState, olibMatrix<double64> filteredP);
	
	//////////////////////////////
    	//
     	// Custom module cleanup.
     	//
    	virtual void cleanup();
};

#endif
