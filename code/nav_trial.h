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

#ifndef _NAV_TRIAL_H_
#define _NAV_TRIAL_H_

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

#include "filt.h"

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
	osl_core::TimerMillis m_timer_dvl;	/* timer for dvl */

    /* messages */
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
	ros::Publisher m_rosFiltNavPub;
	ros::Publisher m_rosOdometryPub;
	ros::Publisher m_rosGPSOffsetPub;
	ros::Publisher m_rosGPSPub;
	ros::Publisher m_rosLBLPub;
	ros::Publisher m_rosLBLOffsetPub;

	auv_msgs::NavSts m_rosNavMsg;
	auv_msgs::NavSts m_rosFiltNavMsg;
	nav_msgs::Odometry m_rosOdometryMsg;
	nav_msgs::Odometry m_rosGPSOffsetMsg;
	nav_msgs::Odometry m_rosLBLOffsetMsg;
	auv_msgs::GPS m_rosGPSMsg;
	auv_msgs::NavSts m_rosLBLMsg;

    std::string tfMapFrame_;
    std::string tfOdomFrame_;
    std::string tfBaseLinkFrame_;
    std::string tfGPSFrame_;
    std::string tfLBLFrame_;

#endif // ifdef USING_ROS

    /* navigation variables */
    float roll, pitch, yaw, yawTcm;
    float pitchRate, rollRate, yawRate, yawTcmRate;
    float surgeVelocity, swayVelocity, heaveVelocity;
    float northVelocity, eastVelocity, depthVelocity;
    float north, east, altitude, depth;

    float initialHeading;  /* initial heading provides by the compass */
    float DVL_position_offset;
    int sendMsgPeriod;
    int delayDvlLock;
    

    ////////// filtering variables (taken from filt.h) /////////////////
    	extendedKalmanFilter5DOF EKF5DOF; // ext kalman filter instance (derived from OceanLIB -> olibKalmanFilter library, defined in filt.h)
    	osl_core::TimerMillis m_timer_ekf; // timer for Kalman filter
	
	olibVector<double64> ObservationVec;
	olibVector<stateID>  ObservationStates;
	
	olibVector<double64> MeasurementVec; 
	olibVector<stateID>  MeasurementStates;
	sensorType MeasurementSensor;

	void addMeasurementToObservation(olibVector<double64> & ObservationVec, olibVector<stateID> & ObservationStates, olibVector<double64> & MeasurementVec, olibVector<stateID> & MeasurementStates);
	
	int ekfPeriod;
	
	// output results
	float northFiltered, eastFiltered, depthFiltered, altitudeFiltered;
	float surgeVelocityFiltered, swayVelocityFiltered, heaveVelocityFiltered;
	float pitchFiltered, yawFiltered, pitchRateFiltered, yawRateFiltered;
	void storeFilteringResults(olibVector<double64> filteredStateVec);
    ///////////////////////////////////////////////////////////////////

    osl_core::GlobalCoordConverter globalCoordConverter;
    bool fixedOrigin;
    osl_core::T_LLD originLLD;	/* lat and lon origin for the local NED frame */

    float fogOffset;
    float transverseVelocity;		/* velocity returned by the dvl */
    float longitudinalVelocity;		/* velocity returned by the dvl */

    /* flags */
    bool gotHeading;
    bool fogOffsetSet;
    bool firstDvlMsgReceived, firstGyroMsgReceived, firstTcmMsgReceived, firstDepthMsgReceived;
    bool gotGpsFix;
    bool fakeGPS;
    osl_core::T_LLD fakeGPSLLD;
    bool lostDvlLock;

    uint8 navStatus;	/* status of navigation module set in the nav message */
    uint8 navMode;		/* navigation mode */

    /* global origin from gps */
    double latitude;
    double longitude;

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

    /**
     * Function for non-message handling work performed by the module.
     * Return false normally, or true if there is "more work to do".
     */
    void doWork();

	void updateGlobalPosition();

	void sendROSNavSts();

	void sendROSOdometry();

	void sendALINavSts();

	void sendSeetrackNavMsg();

    /**
     * Custom module cleanup.
     */
    virtual void cleanup();
};

#endif
