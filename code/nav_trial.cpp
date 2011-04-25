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

// Define DEBUG_OUTPUT to enable PRINTLN_DEBUG output when not using ROS.
// ROS debug level output is toggled at runtime using rxconsole.
//#define DEBUG_OUTPUT

#include <algorithm>

using namespace std;

#include <osl_core/GeneralMath.h>

#include "nav_trial.h"

float f_wrapAngleDeg(float angle1) {
	float angle2 = angle1;
	if (angle2 < 0)
		do {
			angle2 += 360;
		} while (angle2 < 0);
	if (angle2 >= 360)
		do {
			angle2 -= 360;
		} while (angle2 >= 360);

	return angle2;
}

float f_wrapAngleRad(float angle1) {
	float angle2 = angle1;
	if (angle2 < 0)
		do {
			angle2 += 2 * M_PI;
		} while (angle2 < 0);
	if (angle2 >= 2 * M_PI)
		do {
			angle2 -= 2 * M_PI;
		} while (angle2 >= 2 * M_PI);

	return angle2;
}
// constructor
navigation::navigation()
: ObservationVec(1,0),
  ObservationStates(1,NULL_STATE),
  MeasurementVec(1,0),
  MeasurementStates(1,NULL_STATE)
{

}


bool navigation::init(int argc, char * argv[]) {
	//PRINTLN_INFO("init entered");

	/* read INI file */
	if (!getINIFloat(DVL_position_offset, "dvlSurgeOffset", "NAVIGATION"))
		return false;

	if (!getINIBool(compensateDVLPitchRoll, "compensateDVLPitchRoll",
			"NAVIGATION")) return false;
	if (compensateDVLPitchRoll)
	{
		cout << "Compensation for DVL pitch and roll enabled." << endl;
	}
	else
	{
		cout << "Compensation for DVL pitch and roll DISABLED." << endl;
	}

	if (!getINIBool(fakeGPS, "fakeGPS", "NAVIGATION"))
		return false;
	// TODO: floats for origin lat and lon are not really precise enough - they should be doubles.
	float fakeGPSLat;
	float fakeGPSLon;
	if (!getINIFloat(fakeGPSLat, "fakeGPSLat", "NAVIGATION"))
		return false;
	if (!getINIFloat(fakeGPSLon, "fakeGPSLon", "NAVIGATION"))
		return false;

	if (!getINIBool(fixedOrigin, "fixedOrigin", "NAVIGATION"))
		return false;
	// TODO: floats for origin lat and lon are not really precise enough - they should be doubles.
	float fixedOriginLat;
	float fixedOriginLon;
	if (!getINIFloat(fixedOriginLat, "fixedOriginLat", "NAVIGATION"))
		return false;
	if (!getINIFloat(fixedOriginLon, "fixedOriginLon", "NAVIGATION"))
		return false;

	/* set flags */
	gotHeading = false;
	gotGpsFix = false;
	fogOffsetSet = false;
	firstDvlMsgReceived = false;
	firstGyroMsgReceived = false;
	firstTcmMsgReceived = false;
	firstDepthMsgReceived = false;
	lostDvlLock = false;
	transponderNEDValid = false;
    stableLBLSolution = false;
    LBLfixAge = new osl_core::Chrono();

	/* initialise scalar variables to zero */
	roll = pitch = yaw = 0.0;
	pitchRate = rollRate = yawRate = 0.0;
	surgeVelocity = swayVelocity = heaveVelocity = 0.0;
	northVelocity = eastVelocity = depthVelocity = 0.0;
	north = east = altitude = depth = 0.0;
	initialHeading = 0.0;
	fogOffset = 0.0;

	// read LBL parameters from iniFile
	if (!getINIBool(useLBL, "useLBL", "NAVIGATION"))
		return false;
	// read speed of sound
	if (!getINIFloat(speedOfSound, "speedOfSound", "NAVIGATION"))
		return false;
	if (!getINIBool(applyLBLSolution, "applyLBLSolution", "NAVIGATION"))
		return false;

    if (!getINIFloat(maxLBLStableDistance, "maxLBLStableDistance", "NAVIGATION"))
		return false;
    if (!getINIInt(maxLBLOutliersFromStable, "maxLBLOutliersFromStable", "NAVIGATION"))
		return false;

    cout << "useLBL                   : " << useLBL << endl;
    if (useLBL) {
		cout << "applyLBLSolution         : " << applyLBLSolution << endl;
		cout << "maxLBLStableDistance     : " << maxLBLStableDistance << endl;
		cout << "maxLBLOutliersFromStable : " << maxLBLOutliersFromStable << endl;
		if (maxLBLOutliersFromStable % 2 != 1) {
			cerr << "maxLBLOutliersFromStable cannot be an even number!" << endl;
			return false;
		}
    }
	// DT4A
	if (!getINIBool(transponderEnable[0], "DT4AEnable", "NAVIGATION"))
		return false;
	float temp;
	if (!getINIFloat(temp, "DT4ALat", "NAVIGATION"))
		return false;
	transponderLLD[0].lat = temp;
	if (!getINIFloat(temp, "DT4ALon", "NAVIGATION"))
		return false;
	transponderLLD[0].lon = temp;
	if (!getINIFloat(temp, "DT4ADepth", "NAVIGATION"))
		return false;
	transponderLLD[0].depth = temp;
	transponderActive[0] = false;
	// DT4B
	if (!getINIBool(transponderEnable[1], "DT4BEnable", "NAVIGATION"))
		return false;
	if (!getINIFloat(temp, "DT4BLat", "NAVIGATION"))
		return false;
	transponderLLD[1].lat = temp;
	if (!getINIFloat(temp, "DT4BLon", "NAVIGATION"))
		return false;
	transponderLLD[1].lon = temp;
	if (!getINIFloat(temp, "DT4BDepth", "NAVIGATION"))
		return false;
	transponderLLD[1].depth = temp;
	transponderActive[1] = false;
	// DT4C
	if (!getINIBool(transponderEnable[2], "DT4CEnable", "NAVIGATION"))
		return false;
	if (!getINIFloat(temp, "DT4CLat", "NAVIGATION"))
		return false;
	transponderLLD[2].lat = temp;
	if (!getINIFloat(temp, "DT4CLon", "NAVIGATION"))
		return false;
	transponderLLD[2].lon = temp;
	if (!getINIFloat(temp, "DT4CDepth", "NAVIGATION"))
		return false;
	transponderLLD[2].depth = temp;
	transponderActive[2] = false;
	// DT4D
	if (!getINIBool(transponderEnable[3], "DT4DEnable", "NAVIGATION"))
		return false;
	if (!getINIFloat(temp, "DT4DLat", "NAVIGATION"))
		return false;
	transponderLLD[3].lat = temp;
	if (!getINIFloat(temp, "DT4DLon", "NAVIGATION"))
		return false;
	transponderLLD[3].lon = temp;
	if (!getINIFloat(temp, "DT4DDepth", "NAVIGATION"))
		return false;
	transponderLLD[3].depth = temp;
	transponderActive[3] = false;

	// -------------------------------------------------------
	// Possible settings for fakeGPS and fixedOrigin:
	//
	// 1. fakeGPS false, fixedOrigin false:
	//      The first suitable GPS fix is used as the origin. No nav is sent before this.
	//		When the origin is obtained, the north and east position will start at zero.
	// 2. fakeGPS false, fixedOrigin true:
	//      The first suitable GPS fix is used to localise the vehicle.
	//      Once this is obtained, the fixedOrigin latitude and longitude are used to
	//      map the current GPS latitude and longitude into a north, east offset which
	//      determines the current vehicle position. No nav is sent before the GPS fix.
	// 3. fakeGPS true, fixedOrigin false
	//      GPS is ignored. The provided fakeGPSLat and fakeGPSLon are used as both the
	//      origin and initial vehicle position. North and east position will start at zero.
	//      Nav will be sent immediately.
	// 4. fakeGPS true, fixedOrigin true
	//      GPS is ignored. The provided fakeGPSLat and fakeGPSLon will be mapped into the
	//      frame of the fixedOrigin latitude and longitude, as if they were received as
	//      the first GPS fix (like case 1). Nav will be sent immediately.

	if (fixedOrigin)
	{
		originLLD.lat = fixedOriginLat;
		originLLD.lon = fixedOriginLon;
		originLLD.depth = 0.0;
		cout << "Fixed origin from ini file" << endl;
		cout << "Fixed origin ( Lat:" << originLLD.lat << ", Lon:" << originLLD.lon
				<< " ) " << endl;
		globalCoordConverter.setLLDReference(originLLD);
	}
	else {
		cout << "No fixed origin defined - origin will be determined first GPS fix." << endl;
	}

	if (fakeGPS)
	{
		fakeGPSLLD.lat = fakeGPSLat;
		fakeGPSLLD.lon = fakeGPSLon;
		fakeGPSLLD.depth = 0.0;

		cout << "Using fake initial GPS position from ini file, real GPS ignored." << endl;
		cout << "Fake initial GPS ( Lat:" << fakeGPSLLD.lat << ", Lon:" << fakeGPSLLD.lon
				<< " ) " << endl;

		gotGpsFix = true;

		if (fixedOrigin) {
			jumpToGlobalPosition(fakeGPSLat, fakeGPSLon);
		}
		else {
			// Don't jump if not fixed origin - fake initial position IS the origin.
			cout << "Using fake initial GPS position as origin." << endl;
			originLLD = fakeGPSLLD;
			globalCoordConverter.setLLDReference(originLLD);
		}

		// Fake GPS means we can always calculate transponder positions straight away.
		calculateTransponderNED();
	}
	else // true gps
	{
		cout << "Using true GPS, will wait for first GPS fix to get current position." << endl;
		gotGpsFix = false;

		// With true GPS, we can only calculate transponder positions now if using fixed origin.
		if (fixedOrigin) {
			calculateTransponderNED();
		}
		else {
			cout << "Waiting for GPS fix before calculating transponder NED positions." << endl;
		}
	}

	// initialize EKF //
	double64 dT_init = 0.0; //dT is set to 0 at first
	EKF5DOF.setElapsedPeriod(dT_init);
	EKF5DOF.init(EVERY_DT);	
	// initialize variables used for filtering //
	ekfPeriod = 100; //ms, if the EVERY_DT option is chosen
	MeasurementSensor = NULL_SENSOR;
	
	//initialize variables to store the filtered state//
 	northFiltered = eastFiltered = depthFiltered = altitudeFiltered = 0.0;
	surgeVelocityFiltered = swayVelocityFiltered =  heaveVelocityFiltered = 0.0;
	pitchFiltered =  yawFiltered =  pitchRateFiltered = yawRateFiltered = 0.0;
		
	/* set send nav messages period */
	sendMsgPeriod = 200; /* in ms */
	delayDvlLock = 800; /* in ms */
	diagPeriod = 1000;
	

	/* set timers */
	m_timer.setTimer(1); /* init timer to 1ms */
	m_timer_dvl.setTimer(delayDvlLock); /* init of dvl timer */
	diagTimer.setTimer(1);
	m_timer_ekf.setTimer(ekfPeriod);// timer for ekf //

#ifdef USING_ROS
	tfMapFrame_ = "/map";
	tfOdomFrame_ = "odom";
	tfBaseLinkFrame_ = "base_link";
	tfGPSFrame_ = "gps";
	tfLBLFrame_ = "lbl";

	m_rosNavPub     = m_rosHandle->advertise<auv_msgs::NavSts>("nav/nav_sts",      10); 
	m_rosFiltNavPub = m_rosHandle->advertise<auv_msgs::NavSts>("nav/filt_nav_sts", 10);// publisher for filtered messages
	m_rosOdometryPub = m_rosHandle->advertise<nav_msgs::Odometry>("nav/odometry", 10);
	m_rosGPSOffsetPub = m_rosHandle->advertise<nav_msgs::Odometry>("nav/gps_from_origin", 10);
	m_rosGPSPub = m_rosHandle->advertise<auv_msgs::GPS>("nav/gps", 10);
	m_rosLBLPub = m_rosHandle->advertise<auv_msgs::NavSts>("nav/lbl_nav_sts", 10);
	m_rosLBLOffsetPub = m_rosHandle->advertise<nav_msgs::Odometry>("nav/lbl_from_origin", 10);
#endif // ifdef USING_USING_ROS
	return true;
} /* end init */

// ---------------------------------------------------------------------------
void navigation::jumpToGlobalPosition(double newLat, double newLon)
{
	assert(fixedOrigin);

	osl_core::T_LLD newLLD;
	osl_core::T_NED currentNEDPosition;

	newLLD.lat = newLat;
	newLLD.lon = newLon;
	newLLD.depth = depth;

	cout << "jumpToGlobalPosition: lat " << newLat << ", lon " << newLon << endl;

	cout << "jumpToGlobalPosition: Old NED position: north " << fixed << setprecision(2) << north << ", east " << setprecision(2) << east << ", depth " << setprecision(2) << depth << endl;

	currentNEDPosition = globalCoordConverter.LLD2NED(newLLD);
	north = currentNEDPosition.n;
	east = currentNEDPosition.e;

	cout << "jumpToGlobalPosition: New NED position: north " << fixed << setprecision(2) << north << ", east " << setprecision(2) << east << ", depth " << setprecision(2) << depth << endl;

	// Now recalculate the global pos back from our new NED, in line with normal running behaviour.
	updateGlobalPosition();
}

// ---------------------------------------------------------------------------
void navigation::jumpPositionByOffset(float northOffset, float eastOffset)
{
	cout << "jumpPositionByOffset: delta north " << northOffset << ", east " << eastOffset << endl;
	north += northOffset;
	east += eastOffset;
	cout << "jumpPositionByOffset: new north " << north << ", east " << east << endl;

	// Now recalculate the global pos back from our new NED, in line with normal running behaviour.
	updateGlobalPosition();
}

// ---------------------------------------------------------------------------
#ifdef USING_OSH
void navigation::handleMsg(oshMsg_Generic &msg) {

	bool updatedNED = false;

	switch (msg.GetMsgID()) {

		/* depth message */
		case ALI_MSG_DEPTH_STS_ID:
		{
			oldDepthMsg <= depthMsg; /* store previous depth message */
			depthMsg <= msg; /* store new depth msg */
			
			float oldDepth = depth;
			depth = depthMsg.GetDepth();

			// TODO parameterise depth sensor offset.
			float depthSensorOffset = 0.376;
			float depthAdjustment = depthSensorOffset * sinf(DEG2RAD(pitch));
			//cout << "Depth " << depth << ", pitch " << pitch << ", depthAdj " << depthAdjustment;
			depth = depth + depthAdjustment;
			//cout << ", adj depth " << depth << endl;
			
			/* calculate heaveVelocity, which is the depthRate */
			
			timeval oldTS, newTS; /* time stamps to calculate the heaveVelocity and to be used in filtering */
			
			if (firstDepthMsgReceived) {
				
				oldDepthMsg.GetTS(&oldTS); /* old depth msg time stamp */
				depthMsg.GetTS(&newTS); /* new depth msg time stamp */
				
				/* difference between the time stamps in seconds */
				float deltaT = newTS.tv_sec + newTS.tv_usec * 0.000001 - (oldTS.tv_sec + oldTS.tv_usec * 0.000001); /* in seconds */
				
				if (deltaT != 0) {
					heaveVelocity = (depth - oldDepth) / deltaT;
				} else
				heaveVelocity = 0.0;

			} else {
				firstDepthMsgReceived = true;
				heaveVelocity = 0.0;
			}

			depthVelocity = heaveVelocity;

			//PRINTLN_INFO("depth = " << pitchRate << " m");
			//PRINTLN_INFO("heave velocity = depth velocity = " << heaveVelocity << " m/s");

			updatedNED = true;
			//----FILTERING (DEPTH)----//			
    			if(EKF5DOF.isFilteringPossible())
    			{
				// it is not a first message for Kalman - had some before
				double64 current_time;
				current_time = newTS.tv_sec + newTS.tv_usec * 0.000001; // correct: double calculation!
				MeasurementVec.setSize(2, 0);//enable enough space
				MeasurementStates.setSize(2, NULL_STATE);//enable enough space 
				sensorType sensorDevice = DEPTH;
				MeasurementVec[0] = depth;          MeasurementStates[0] = DEPTH;
				MeasurementVec[1] = heaveVelocity;  MeasurementStates[1] = HEAVE_VELOCITY;
				if(EKF5DOF.ekfMode==EVERY_MESSAGE)
				{
					olibVector<double64> filteredValues(SYSTEM_STATE_LEN,0);
					filteredValues = EKF5DOF.filterMessage(MeasurementVec, sensorDevice, current_time);
					storeFilteringResults(filteredValues); //store it in northFiltered, eastFiltered, etc...
					
				}
				if(EKF5DOF.ekfMode==EVERY_DT)
				{//aim is to create an oservatio that will be periodically processed within timer defined interval
					addMeasurementToObservation(ObservationVec, ObservationStates, MeasurementVec, MeasurementStates);
				}
			}
			else
			{
				// it is a first message for Kalman
				// dT stays zero, initial value no filtering is being done
				//double64 dT_first_message = 0;
				//EKF5DOF.setElapsedPeriod(dT_first_message);
				olibVector<double64> X(SYSTEM_STATE_LEN, 0); // state vector
				X[DEPTH_INDEX]     = depth;
				X[HEAVE_VEL_INDEX] = depthVelocity;
				EKF5DOF.setXPrediction(X); EKF5DOF.setXCorrected(X);
				EKF5DOF.setFilteringPossible();
			}
			//----FILTERING----//			
			break;
		}

		/* compass message */
		/* compass provides roll, pitch and the first heading (yaw) */
		case Tcm2Msg_MSG_ID:
		{
			oldTcmMsg <= tcmMsg; /* store previous tcm message */
			tcmMsg <= msg; /* store new tcm msg */

			/* The initial heading is provided by the compass */
			if (!gotHeading) {
				initialHeading = tcmMsg.get_fHeading();
				yaw = initialHeading;
				gotHeading = true;

			}

			roll = tcmMsg.get_fRoll();
			pitch = tcmMsg.get_fPitch();
			yawTcm = tcmMsg.get_fHeading(); // marked because there is another yaw measured by gyro
			
			timeval oldTS, newTS; /* time stamps to calculate the pitch and roll rates and to be used in filtering */
			
			/* calculate pitch and roll rates */
			if (firstTcmMsgReceived) {
				
				oldTcmMsg.GetTS(&oldTS); /* old tcm msg time stamp */
				tcmMsg.GetTS(&newTS); /* new tcm msg time stamp */

				float oldPitch = oldTcmMsg.get_fPitch();
				float oldRoll = oldTcmMsg.get_fRoll();
				float oldYawTcm = oldTcmMsg.get_fHeading();
				/* time difference between the time stamps in seconds */
				float deltaT = newTS.tv_sec + newTS.tv_usec * 0.000001 - (oldTS.tv_sec + oldTS.tv_usec * 0.000001); /* in seconds */
				if (deltaT != 0) {
					pitchRate = (pitch - oldPitch)/deltaT;
					rollRate = (roll - oldRoll)/deltaT;
					yawTcmRate = (yawTcm - oldYawTcm)/deltaT;
				}
				else {
					pitchRate = 0.0;
					rollRate = 0.0;
					yawTcmRate = 0.0;
				}
			}

			else {
				firstTcmMsgReceived = true;
				pitchRate = 0.0;
				rollRate = 0.0;
			}

			//PRINTLN_INFO("pitch = " << pitch << " deg roll = " << roll << " deg");
			//PRINTLN_INFO("pitch rate = " << pitchRate << " deg/s roll rate = " << rollRate << " deg/s");
			
			//----FILTERING (COMPASS)----//			
    			if(EKF5DOF.isFilteringPossible)
    			{
				// it is not a first message for Kalman - had some before
				double64 current_time;
				current_time = newTS.tv_sec + newTS.tv_usec * 0.000001;  // correct: double calculation!
				MeasurementVec.setSize(3, 0);//enable enough space
				MeasurementStates.setSize(3, 0);//enable enough space 
				sensorType sensorDevice = COMPASS_SENSOR;
				MeasurementVec[0] = pitch;          MeasurementStates[0] = PITCH;
				MeasurementVec[1] = pitchRate;      MeasurementStates[1] = PITCH_RATE;
				MeasurementVec[2] = yaw;            MeasurementStates[2] = YAW;
				if(EKF5DOF.ekfMode==EVERY_MESSAGE)
				{
					olibVector<double64> filteredValues(SYSTEM_STATE_LEN,0);
					filteredValues = EKF5DOF.filterMessage(MeasurementVec, sensorDevice, current_time);
					storeFilteringResults(filteredValues); //store it in northFiltered, eastFiltered, etc...
				}
				if(EKF5DOF.ekfMode==EVERY_DT)
				{//aim is to create an oservatio that will be periodically processed within timer defined interval
					addMeasurementToObservation(ObservationVec, ObservationStates, MeasurementVec, MeasurementStates);
				}
			}
			else
			{
				// it is a first message for Kalman
				// dT stays zero, initial value no filtering is being done
				//double64 dT_first_message = 0;
				//EKF5DOF.setElapsedPeriod(dT_first_message);
				olibVector<double64> X(SYSTEM_STATE_LEN, 0); // state vector
				X[PITCH_INDEX]      = pitch;
				X[PITCH_RATE_INDEX] = pitchRate;
				X[YAW_INDEX]      = yaw;
				EKF5DOF.setXPrediction(X); EKF5DOF.setXCorrected(X);
				EKF5DOF.setFilteringPossible();
			}
			//----FILTERING----//				
			break;
		}

		/* gyro msg */
		/* gyro provides yaw */
		case ALI_MSG_GYRO_ID:
		{
			oldGyroMsg <= gyroMsg; /* store previous gyro msg */
			gyroMsg <= msg;

			//gyroMsg.Display();

			if (gyroMsg.GetType() == ALI_GYRO_RATE || gyroMsg.GetType() == ALI_GYRO_BOTH) {

				yawRate = gyroMsg.GetYawRate();// + fogOffset;
				PRINTLN_INFO("Received yawRate " << yawRate);
			}
			else {
				PRINTLN_WARN("Unsupported gyroMsg type: " << gyroMsg.GetType());
			}
			timeval newTS;
			gyroMsg.GetTS(&newTS);
			//----FILTERING (GYRO)----//			
    			if(EKF5DOF.isFilteringPossible)
    			{// it is not a first message for Kalman - had some before
				double64 current_time;
				current_time = newTS.tv_sec + newTS.tv_usec * 0.000001;  // correct: double calculation!
				MeasurementVec.setSize(1, 0);//enable enough space
				MeasurementStates.setSize(1, 0);//enable enough space 
				sensorType sensorDevice = GYRO_SENSOR;
				MeasurementVec[0] = yawRate;          MeasurementStates[0] = YAW_RATE;
				if(EKF5DOF.ekfMode==EVERY_MESSAGE)
				{
					olibVector<double64> filteredValues(SYSTEM_STATE_LEN,0);
					filteredValues = EKF5DOF.filterMessage(MeasurementVec, sensorDevice, current_time);
					storeFilteringResults(filteredValues); //store it in northFiltered, eastFiltered, etc...
				}
				if(EKF5DOF.ekfMode==EVERY_DT)
				{//aim is to create an oservatio that will be periodically processed within timer defined interval
					addMeasurementToObservation(ObservationVec, ObservationStates, MeasurementVec, MeasurementStates);
				}
			}
			else
			{
				// it is a first message for Kalman
				// dT stays zero, initial value no filtering is being done
				//double64 dT_first_message = 0;
				//EKF5DOF.setElapsedPeriod(dT_first_message);
				olibVector<double64> X(SYSTEM_STATE_LEN, 0); // state vector
				X[YAW_RATE_INDEX]      = yawRate;
				EKF5DOF.setXPrediction(X); EKF5DOF.setXCorrected(X);
				EKF5DOF.setFilteringPossible();
			}
			//----FILTERING----//
			break;
			//PRINTLN_INFO("yaw = " << yaw << " deg yawRate = " << yawRate << " deg/s");
		}

		case DVL_MSG:
		{
			oldDvlMsg <= dvlMsg;
			dvlMsg <= msg;

			m_timer_dvl.setTimer(delayDvlLock);

			altitude = dvlMsg.getBTERDDRangeToBottom(); /* in meters */
			transverseVelocity = dvlMsg.getBTSRVDTransverseVelocity()*0.001;
			longitudinalVelocity = dvlMsg.getBTSRVDLongitudinalVelocity()*0.001;

			float yawRad = yaw * Deg2Rad;
			float yawRateRad = yawRate * Deg2Rad;
			float pitchRateRad = pitchRate * Deg2Rad;
			float rollRateRad = rollRate * Deg2Rad;

			surgeVelocity = longitudinalVelocity;
			// TODO If DVL offset is wrong by epsilon,we introduce a bias of epsilon * YawRateRad. 
			// Needs Very accurate measurement of distance btw compass and DVL.There is also an offset in y not 
			// taken into account. Again a few cm might make a difference. There might also be an alignment error 
			// btw DVL and Compass
			swayVelocity = transverseVelocity - DVL_position_offset * yawRateRad;

			if (compensateDVLPitchRoll)
			{
				cout << "dvl surgeVelocity = " << surgeVelocity << ", pitchRate " << pitchRate << endl;
				cout << "dvl swayVelocity  = " << swayVelocity << ", rollRate " << rollRate << endl;

				float surgeFromPitch = altitude * tanf(pitchRateRad);
				float swayFromRoll = -altitude * tanf(rollRateRad);

				surgeVelocity -= surgeFromPitch;
				swayVelocity -= swayFromRoll;

				cout << "surgeFromPitch = " << surgeFromPitch
						<< ", compensated surgeVelocity " << surgeVelocity << endl;
				cout << "swayFromRoll = " << swayFromRoll
						<< ", compensated swayVelocity  = " << swayVelocity << endl;

				// TODO: parameterise maximum surge and sway velocities
				if (fabs(surgeVelocity) > 2.0) {
					cerr << "surgeVelocity " << surgeVelocity << " too big (surgeFromPitch " << surgeFromPitch << "); clamping to zero." << endl;
					surgeVelocity = 0;
				}
				if (fabs(swayVelocity) > 1.0) {
					cerr << "swayVelocity " << swayVelocity << " too big (swayFromRoll " << swayFromRoll << "); clamping to zero." << endl;
					swayVelocity = 0;
				}
			}
			timeval oldTS, newTS;
			if (!firstDvlMsgReceived) {
				firstDvlMsgReceived = true;
			}

			else if (lostDvlLock == true) {
				lostDvlLock = false;
			}

			else {
				oldDvlMsg.GetTS(&oldTS); /* old dvl msg time stamp */
				dvlMsg.GetTS(&newTS); /* (current) dvl msg time stamp */
				float deltaT = newTS.tv_sec + newTS.tv_usec * 0.000001 -
				(oldTS.tv_sec + oldTS.tv_usec * 0.000001); /* in seconds */

				// pitch and roll corrections are already done in DVL.
				eastVelocity = surgeVelocity * sin(yawRad) + swayVelocity * cos(yawRad);
				northVelocity = surgeVelocity * cos(yawRad) - swayVelocity * sin(yawRad);

				north += northVelocity * deltaT;
				east += eastVelocity * deltaT;
			
				//----FILTERING (DVL)----//			
	    			if(EKF5DOF.isFilteringPossible)
	    			{// it is not a first message for Kalman - had some before
					double64 current_time;
					current_time = newTS.tv_sec + newTS.tv_usec * 0.000001;  // correct: double calculation!
					MeasurementVec.setSize(3, 0);//enable enough space
					MeasurementStates.setSize(3, 0);//enable enough space 
					sensorType sensorDevice = DVL_SENSOR;
					MeasurementVec[0] = altitude;               MeasurementStates[0] = ALTITUDE;
					MeasurementVec[1] = surgeVelocity;          MeasurementStates[1] = SURGE_VELOCITY;
					MeasurementVec[2] = swayVelocity;           MeasurementStates[2] = SWAY_VELOCITY;
					if(EKF5DOF.ekfMode==EVERY_MESSAGE)
					{
						olibVector<double64> filteredValues(SYSTEM_STATE_LEN,0);
						filteredValues = EKF5DOF.filterMessage(MeasurementVec, sensorDevice, current_time);
						storeFilteringResults(filteredValues); //store it in northFiltered, eastFiltered, etc...
					}
					if(EKF5DOF.ekfMode==EVERY_DT)
					{//aim is to create an oservatio that will be periodically processed within timer defined interval
						addMeasurementToObservation(ObservationVec, ObservationStates, MeasurementVec, MeasurementStates);
					}
				}
				else
				{
					// it is a first message for Kalman
					// dT stays zero, initial value no filtering is being done
					//double64 dT_first_message = 0;
					//EKF5DOF.setElapsedPeriod(dT_first_message);
					olibVector<double64> X(SYSTEM_STATE_LEN, 0); // state vector
					X[ALTITUDE_INDEX]       = altitude;
					X[SURGE_VEL_INDEX]      = surgeVelocity;
					X[SWAY_VEL_INDEX]       = swayVelocity;
					EKF5DOF.setXPrediction(X); EKF5DOF.setXCorrected(X);
					EKF5DOF.setFilteringPossible();
				}
				//----FILTERING----//
			}

			updatedNED = true;

			break;
			//PRINTLN_INFO("north = " << north << " m east = " << east << " altitude = " << altitude);
			//PRINTLN_INFO("north velocity = " << northVelocity << " m/s east velocity = " << eastVelocity << " m/s");
			//PRINTLN_INFO("surge velocity = " << surgeVelocity << " m/s sway velocity = " << swayVelocity << " m/s");
		}
		/* LBL message */
		case ALI_MSG_TIME_DELAY_ID:
		{
			lblMsg <= msg;
			if (gotGpsFix) {
				handleTimeDelayMsg(lblMsg);
			}
			else {
				cout << "Ignoring LBL time delay message until got GPS fix." << endl;
			}
			
			timeval newTS;
			lblMsg.GetTS(&newTS);
			
			//----FILTERING (LBL)----//			
    			if(EKF5DOF.isFilteringPossible)
    			{// it is not a first message for Kalman - had some before
				double64 current_time;
				current_time = newTS.tv_sec + newTS.tv_usec * 0.000001;  // correct: double calculation!
				MeasurementVec.setSize(2, 0);//enable enough space
				MeasurementStates.setSize(2, 0);//enable enough space 
				sensorType sensorDevice = LBL_SENSOR;
				MeasurementVec[0] = north;         MeasurementStates[0] = NORTH;
				MeasurementVec[1] = east;          MeasurementStates[1] = EAST;
				if(EKF5DOF.ekfMode==EVERY_MESSAGE)
				{
					olibVector<double64> filteredValues(SYSTEM_STATE_LEN,0);
					filteredValues = EKF5DOF.filterMessage(MeasurementVec, sensorDevice, current_time);
					storeFilteringResults(filteredValues); //store it in northFiltered, eastFiltered, etc...
				}
				if(EKF5DOF.ekfMode==EVERY_DT)
				{//aim is to create an oservatio that will be periodically processed within timer defined interval
					addMeasurementToObservation(ObservationVec, ObservationStates, MeasurementVec, MeasurementStates);
				}
			}
			else
			{
				// it is a first message for Kalman
				// dT stays zero, initial value no filtering is being done
				//double64 dT_first_message = 0;
				//EKF5DOF.setElapsedPeriod(dT_first_message);
				olibVector<double64> X(SYSTEM_STATE_LEN, 0); // state vector
				X[NORTH_INDEX]       = north;
				X[EAST_INDEX]        = east;
				EKF5DOF.setXPrediction(X); EKF5DOF.setXCorrected(X);
				EKF5DOF.setFilteringPossible();
			}
			//----FILTERING----//			
			break;
		}
		/* gps message */
		case ALI_MSG_GPS_ID:
		{
			gpsMsg <= msg;

#ifdef USING_ROS
			// Repeat ALI GPS msg via ROS, as GPS module not ROS-ified.
			{
				m_rosGPSMsg.header.frame_id = "gps";
				struct timeval gpsTime;
				gpsMsg.GetTS(&gpsTime);
				m_rosGPSMsg.header.stamp = ros::Time(gpsTime.tv_sec, gpsTime.tv_usec * 1000);
				m_rosGPSMsg.fix_quality = gpsMsg.GetFixQuality();
				m_rosGPSMsg.hdop = gpsMsg.GetHDOP();
				m_rosGPSMsg.altitude = gpsMsg.GetAlt();
				m_rosGPSMsg.latitude = gpsMsg.GetLat();
				m_rosGPSMsg.longitude = gpsMsg.GetLon();

				m_rosGPSPub.publish(m_rosGPSMsg);
			}

			// Create and publish odometry message with offset from GPS origin,
			// if we have an origin, and a proper GPS fix.
			if (globalCoordConverter.isInitialised() && gpsMsg.GetFixQuality() >= 1)
			{
				osl_core::T_LLD tempLLD;
				osl_core::T_NED gpsNED;

				tempLLD.lat = gpsMsg.GetLat();
				tempLLD.lon = gpsMsg.GetLon();
				tempLLD.depth = depth;

				gpsNED = globalCoordConverter.LLD2NED(tempLLD);

				float gpsN = gpsNED.n;
				float gpsE = gpsNED.e;

				// Map from AUV to ROS coordinates.
				double rosX, rosY, rosZ, rosRoll, rosPitch, rosYaw;
				osl_core::mapAxisAUVtoROS(gpsN, gpsE, depth, rosX, rosY, rosZ);
				osl_core::mapAxisAUVtoROS(osl_core::deg2rad(roll), osl_core::deg2rad(pitch), osl_core::deg2rad(yaw),
						rosRoll, rosPitch, rosYaw);
				geometry_msgs::Quaternion rosOrientation = tf::createQuaternionMsgFromRollPitchYaw(rosRoll, rosPitch, rosYaw);

				m_rosGPSOffsetMsg.header.stamp = m_rosGPSMsg.header.stamp;
				m_rosGPSOffsetMsg.header.frame_id = tfMapFrame_;
				m_rosGPSOffsetMsg.child_frame_id = tfGPSFrame_;
				m_rosGPSOffsetMsg.pose.pose.position.x = rosX;
				m_rosGPSOffsetMsg.pose.pose.position.y = rosY;
				m_rosGPSOffsetMsg.pose.pose.position.z = rosZ;
				m_rosGPSOffsetMsg.pose.pose.orientation = rosOrientation;

				m_rosGPSOffsetPub.publish(m_rosGPSOffsetMsg);

				// Now create Transform msg.
				// Broadcaster needs to be static not member variable as it cannot be created before
				// ModuleCore has initialised ROS.
				static tf::TransformBroadcaster rosTFBroadcaster;
				tf::Transform transform;
				transform.setOrigin(tf::Vector3(rosX, rosY, rosZ));
				transform.setRotation(tf::createQuaternionFromRPY(rosRoll, rosPitch, rosYaw));
				rosTFBroadcaster.sendTransform(tf::StampedTransform(transform, m_rosGPSOffsetMsg.header.stamp,
								m_rosGPSOffsetMsg.header.frame_id, m_rosGPSOffsetMsg.child_frame_id));

				//				PRINTLN_INFO("GPS lat " << std::fixed << std::setprecision(3) << gpsMsg.GetLat()
				//						<< ", lon " << std::fixed << std::setprecision(3) << gpsMsg.GetLon());
				PRINTLN_INFO("GPS origin lat " << std::fixed << std::setprecision(8) << originLLD.lat
						<< ", lon " << std::fixed << std::setprecision(8) << originLLD.lon
						<< ": GPS current offset north " << std::fixed << std::setprecision(3) << gpsN
						<< ", east " << std::fixed << std::setprecision(3) << gpsE);
			}
			else {
				PRINTLN_INFO("Waiting for GPS fix before publishing ROS GPS offset.");
			}
#endif // USING_ROS

			// Ignore rest of GPS processing if faking GPS.
			if (fakeGPS) break;

			// Take the first good GPS fix to initialise the nav, ignore the rest.
			// Good = quality > 0 and horizontal dilution of precision (HDOP) less
			// than a certain value.
			if (!gotGpsFix) {
				if ((gpsMsg.GetFixQuality() < 1) || (gpsMsg.GetHDOP() > 7.5)) {
					cout << "Still waiting for good GPS fix... current GPS msg quality " << gpsMsg.GetFixQuality()
							<< ", hdop " << gpsMsg.GetHDOP() << endl;
				}
				else {
					cout << "GPS fix quality " << gpsMsg.GetFixQuality() << ", hdop " << gpsMsg.GetHDOP()
							<< " at lat " << gpsMsg.GetLat() << " lon " << gpsMsg.GetLon();
					gotGpsFix = true;

					if (fixedOrigin) {
						jumpToGlobalPosition(gpsMsg.GetLat(), gpsMsg.GetLon());
					}
					else {
						// We don't have a fixed origin, so want to use the first GPS fix position as origin.
						// set the gps fix as the new origin
						originLLD.lat = gpsMsg.GetLat();
						originLLD.lon = gpsMsg.GetLon();
						originLLD.depth = 0;
						globalCoordConverter.setLLDReference(originLLD);
						// calculate transponder location with respect to the new origin
						calculateTransponderNED();
					}
				}
			}
			break;
		}

	}

	if (updatedNED) {
		updateGlobalPosition();
	}
}
// ---------------------------------------------------------------------------
void navigation::addMeasurementToObservation(olibVector<double64> & ObservationVec, olibVector<stateID> & ObservationStates, olibVector<double64> & MeasurementVec, olibVector<stateID> & MeasurementStates)
{
	uint32 prevObservationLength, MeasurementLength, newObservationLength;
	prevObservationLength = ObservationVec.size();
	newObservationLength = prevObservationLength;
	MeasurementLength = MeasurementVec.size();
	if((prevObservationLength==1)&&(ObservationStates[0]==NULL_STATE))
	{//observation empty, add all measurement values to observation
		
		ObservationVec.setSize(MeasurementLength, 0);
		ObservationStates.setSize(MeasurementLength, NULL_STATE);
		ObservationVec = MeasurementVec;
		ObservationStates = MeasurementStates;
	}
	else
	{// observation not empty, check which values to add
		int mesIndex = 0;
		int obsIndex = 0;
		for(mesIndex=0; i<MeasurementLength; mesIndex++)
		{
			//check if it exists in observation
			for(obsIndex=0; j<prevObservationLength; obsIndex++)
			{
				if(MeasurementStates[mesIndex]==ObservationStates[obsIndex])
				{//existing state comes from measurement
					ObservationVec[obsIndex] = MeasurementVec[mesIndex];//just update
					break;
				}
				
			
			}
			if(obsIndex==prevObservationLength)
			{
				//add a new element in observation
				newObservationLength++;
				ObservationVec.setSize(newObservationLength,0);
				ObservationVec[newObservationLength-1] = MeasurementVec[i];
				ObservationStates[newObservationLength-1] = MeasurementStates[i];
			}
}

// ---------------------------------------------------------------------------
void navigation::handleTimeDelayMsg(aliMsg_TimeDelay lblMsg)
{
	lblMsg.Display();

	assert(gotGpsFix);
	assert(transponderNEDValid);

	if (!useLBL) {
		cout << "Unexpected time delay message when not useLBL; ignoring." << endl;
		return;
	}

	// calculate range from time of travelling
	transponderRange[0] = lblMsg.GetTimeDelayA() * speedOfSound;
	transponderRange[1] = lblMsg.GetTimeDelayB() * speedOfSound;
	transponderRange[2] = lblMsg.GetTimeDelayC() * speedOfSound;
	transponderRange[3] = lblMsg.GetTimeDelayD() * speedOfSound;

	for (int i=0; i<4; ++i) {
		if (transponderRange[i] != 0) {
			cout << "Transponder " << i << " range: " << transponderRange[i] << " m" << endl;
			transponderActive[i] = true;
		} else {
			cout << "Transponder " << i << " range: none" << endl;
			transponderActive[i] = false;
		}
	}

	osl_core::T_NED offset2current;
	bool validLBL = calculateLBLbasedLLD(LBLbasedLLD, offset2current, LBLdistance);

	if (validLBL) {
		cout << "LBLbasedLLD " << LBLbasedLLD << endl;
		cout << "LBLdistance " << LBLdistance << endl;

#ifdef USING_ROS
		// Output LBL fix as ROS message for post mission analysis.
		LBLbasedNED = globalCoordConverter.LLD2NED(LBLbasedLLD);
		PRINTLN_DEBUG("Publishing ros lbl nav msg.");
		m_rosLBLMsg = m_rosNavMsg;// this line is a problem
		m_rosLBLMsg.global_position.latitude = LBLbasedLLD.lat;
		m_rosLBLMsg.global_position.longitude = LBLbasedLLD.lon;
		m_rosNavMsg.position.north = LBLbasedNED.n;
		m_rosNavMsg.position.east = LBLbasedNED.e;
		m_rosNavMsg.position.depth = LBLbasedNED.d;
		m_rosLBLPub.publish(m_rosLBLMsg);

		// Map from AUV to ROS coordinates.
		double rosX, rosY, rosZ, rosRoll, rosPitch, rosYaw;
		osl_core::mapAxisAUVtoROS(LBLbasedNED.n, LBLbasedNED.e, depth, rosX, rosY, rosZ);
		osl_core::mapAxisAUVtoROS(osl_core::deg2rad(roll), osl_core::deg2rad(pitch), osl_core::deg2rad(yaw),
				rosRoll, rosPitch, rosYaw);
		geometry_msgs::Quaternion rosOrientation = tf::createQuaternionMsgFromRollPitchYaw(rosRoll, rosPitch, rosYaw);

		m_rosLBLOffsetMsg.header.stamp = m_rosGPSMsg.header.stamp;
		m_rosLBLOffsetMsg.header.frame_id = tfMapFrame_;
		m_rosLBLOffsetMsg.child_frame_id = tfLBLFrame_;
		m_rosLBLOffsetMsg.pose.pose.position.x = rosX;
		m_rosLBLOffsetMsg.pose.pose.position.y = rosY;
		m_rosLBLOffsetMsg.pose.pose.position.z = rosZ;
		m_rosLBLOffsetMsg.pose.pose.orientation = rosOrientation;

		m_rosLBLOffsetPub.publish(m_rosLBLOffsetMsg);
#endif // USING_ROS

		if (applyLBLSolution) {
			if (LBLdistance < maxLBLStableDistance) {
				cout << "LBLdistance " << fixed << setprecision(2) << LBLdistance << " is within stable tolerance; jumping nav to it." << endl;
				jumpToGlobalPosition(LBLbasedLLD.lat, LBLbasedLLD.lon);
				LBLOutlierHistory.clear();
				LBLfixAge->Reset();
			}
			else {
				LBLOutlierHistory.push_back(LBLOutlier(offset2current, LBLdistance));
				cout << "LBLdistance " << maxLBLStableDistance << " outside tolerance (" << maxLBLStableDistance
						<< "); num outliers now " << LBLOutlierHistory.size() << endl;

				if (LBLOutlierHistory.size() == (unsigned)maxLBLOutliersFromStable) {
					sort(LBLOutlierHistory.begin(), LBLOutlierHistory.end());
					LBLOutlier medianOutlier = LBLOutlierHistory[maxLBLOutliersFromStable / 2];
					cout << "Max num LBL outliers reached, jumping to median with offset: deltaN "
							<< medianOutlier.offsetNED.n << ", deltaE " << medianOutlier.offsetNED.e
							<< ", dist " << medianOutlier.offsetDistance << endl;
					jumpPositionByOffset(medianOutlier.offsetNED.n, medianOutlier.offsetNED.e);
					LBLOutlierHistory.clear();
					LBLfixAge->Reset();
				}
			}
		}
		else {
			cout << "applyLBLSolution is false." << endl;
		}
	}
}
#endif // USING_OSH

// ---------------------------------------------------------------------------
/* doWork is used to send the nav message only */
// filtering happens within doWork if it's set to some period
void navigation::doWork() {

	if (m_timer_dvl.getTimer() == 0) {
		lostDvlLock = true;
	}

	if (m_timer.getTimer() == 0) {
		/* reset timer to sendMsgPeriod milliseconds */
		m_timer.setTimer(sendMsgPeriod);

		// Wrap angles
		roll = osl_core::wrapPlusMinus90(roll);
		pitch = osl_core::wrapPlusMinus90(pitch);
		yaw = osl_core::wrapPlusMinus180(yaw);

		if (diagTimer.getTimer() == 0) {
			diagTimer.setTimer(diagPeriod);
			cout << endl;
			cout << "--------------- time " << setw(15) << currentTimeString()
			     << " --------------" << std::endl;
			
                        
			
		        cout << setw(20) << " OLD NAV " 
		        		 << "||" << " EKF " << endl;		
			cout << "Lat = " << latitude << " Lon = " << longitude  << 
                                        			"||   NO EKF FILTERING " << endl;
			cout << "N = " << north << " E = " << east << " D = " << depth
					<< " A = " << altitude << "m" <<
								"||"<<"north_EKF=  " << northFiltered     
								<<    "east_EKF= "   << eastFiltered     
								<<    "depth_EKF= "  << depthFiltered    
								<<    "altitude_EKF= "<< altitudeFiltered   
								<< "m" << endl;
			cout << "suVel  = " << surgeVelocity << " swVel = " << swayVelocity
					<< " heVel = " << heaveVelocity << " m/s" <<
								"||"<<"suVel_EKF= " << surgeVelocityFiltered
								<<    "swVel_EKF= " << swayVelocityFiltered 
								<<    "heVel_EKF= " << heaveVelocityFiltered
 								<< "m/s"  << endl;
			cout << "Nvel = " << northVelocity << " Evel = " << eastVelocity
					<< " Dvel = " << depthVelocity << " m/s" << 
                                                                "||   NO EKF FILTERING " << endl; 
			cout << "R = " << roll << " P = " << pitch << " Y = " << yaw
					<< " deg" <<
								"||" << "R = NO FILTERING" 
								<<      "P_EKF = "  << pitchFiltered 
								<<      "Y_EKF = "  << yawFiltered 
				  				<< "deg" << endl;
			cout << "Rvel = " << rollRate << " Pvel " << pitchRate
					<< " Yvel = " << yawRate << " deg/s" << 
								"||" << "Rvel = NOFILTERING"
								<<      "Pvel = " << pitchRateFiltered
								<<      "Yvel = " << yawRateFiltered
								<< "deg/s" <<  endl;
			cout << "Compass initial heading = " << initialHeading << " deg"
					<< endl;
			cout << "Gyro offset = " << fogOffset << " deg" << endl;
			
			if (useLBL && applyLBLSolution) {
				// fix age in microseconds
				cout << "LBLfixAge = " << fixed << setprecision(2) <<  (LBLfixAge->ReadMs()/1000.0) << " sec" << endl;
			}
			if (!gotGpsFix) {
				cout << "Waiting for GPS fix before sending any nav." << endl;
			}
			if (lostDvlLock) {
				cout << "Lost DVL lock, not sending nav until it returns." << endl;
			}
		}//diagTimer

		if (!lostDvlLock && gotGpsFix) {
			sendROSNavSts();

			sendROSOdometry();

			sendALINavSts();
		}
		

	} /* end if on timer...  m_timer */
	
	//m_timer_ekf 
	if(m_timer_ekf.getTimer() == 0){
		m_timer_ekf.setTimer(ekfPeriod);
		// following section executes every ekfPeriod
		// use the observation 
		double64 dT; 
		dT = ekfPeriod * 0.001; //sec
		olibVector<double64> filteredValues(SYSTEM_STATE_LEN,0);
		filteredValues = EKF5DOF.filterObservation(ObservationVec, ObservationStates, dT); // this is not good, should be measured dT
		storeFilteringResults(filteredValues);
		//reset observation for the next round
		ObservationVec.setSize(1,0);
		
	} 
	
	
}//doWork

//---------------------------------------------------------------------------

void navigation::updateGlobalPosition() {
	/* calculation of LLD current position */
	osl_core::T_NED currentNED;
	osl_core::T_LLD currentLLD;
	currentNED.n = north;
	currentNED.e = east;
	currentNED.d = depth;
	currentLLD = globalCoordConverter.NED2LLD(currentNED);
	latitude = currentLLD.lat;
	longitude = currentLLD.lon;
}

//---------------------------------------------------------------------------

bool navigation::calculateTransponderNED() {
	if (globalCoordConverter.isInitialised()) {
		for (uint32 i = 0; i < navMaxNumTransponders; i++) {
			if (transponderEnable[i]) {
				transponderNED[i] = globalCoordConverter.LLD2NED(
						transponderLLD[i]);
				cout << "Transponder " << i << " LLD " << transponderLLD[i] << ", position NED from origin "
						<< transponderNED[i] << endl;
			}
		}
		transponderNEDValid = true;
		return true;
	} else {
		return false;
	}
}

//---------------------------------------------------------------------------

bool navigation::calculateLBLbasedLLD(osl_core::T_LLD & solutionLLD, osl_core::T_NED & offset2current, double & distance2current) {
	// NOTE: At the moment, this method gets the best of the solutions (the closest to the current position
	// A possible improvement would be to merge the solution from the different pairs of transponders
	osl_core::T_LLD candidateLLD;
	double candidateDistance2Current;
	osl_core::T_NED candidateOffset2Current;
	bool isFirstCandidate = true;
	bool isReturnValid = false;

	for (uint32 i = 0; i < navMaxNumTransponders; i++) {
		if (!transponderEnable[i] || !transponderActive[i]) continue;
		for (uint32 j = i+1; j < navMaxNumTransponders; j++) {
			if (transponderEnable[j] && transponderActive[j]) {
				if (calculateLBLbasedLLDfromTransponderPair(i, j,
						candidateLLD, candidateOffset2Current, candidateDistance2Current)) {
					if (isFirstCandidate) {
						solutionLLD = candidateLLD;
						offset2current = candidateOffset2Current;
						distance2current = candidateDistance2Current;
						isFirstCandidate = false;
					} else {
						if (distance2current > candidateDistance2Current) {
							solutionLLD = candidateLLD;
							offset2current = candidateOffset2Current;
							distance2current = candidateDistance2Current;
						}
					}
					isReturnValid = true;
				}
			}
		}
	}

	return isReturnValid;
}
//---------------------------------------------------------------------------

bool navigation::calculateLBLbasedLLDfromTransponderPair(
		const unsigned int & first, const unsigned int & second,
		osl_core::T_LLD & solutionLLD, osl_core::T_NED & offset2current,
		double & distance2current)
{
	osl_core::T_NED firstSolutionNED, secondSolutionNED;
	osl_core::T_NED firstSolutionOffset, secondSolutionOffset;
	osl_core::T_LLD firstSolutionLLD, secondSolutionLLD;

	// NOTE: This approach at the moment does not take into consideration depths
	// depths might be relevant when the difference between the transponder depths and the vehicle depth is considerable
	// in this case the sphere equations will be necessary
	cout << "Calculating LBL position from range to transponders " << first
			<< " and " << second << endl;

	// distance between transponders
	float fDistance = sqrt(pow(transponderNED[first].n
			- transponderNED[second].n, 2) + pow(transponderNED[first].e
			- transponderNED[second].e, 2));

	cout << "Distance between transponders " << fDistance << endl;
	cout << "Range to First  " << transponderRange[first] << endl;
	cout << "Range to Second " << transponderRange[second] << endl;

	// no solution if the sum of the ranges is less than the distance between transponders
	if ((transponderRange[first] + transponderRange[second]) < fDistance) {
		cout
				<< "Sum of ranges less than distance between transponders. No solution."
				<< endl;
		return false;
	}
	// no solution if one circle covers the other and viceversa
	if (fabs(transponderRange[first] - transponderRange[second]) > fDistance) {
		cout
				<< "Absolute range difference greater than distance between transponders. No solution."
				<< endl;
		return false;
	}

	// calculate intersection in the frame reference of the long baseline, i.e x-axis along the line defined by the two transponders
	double x, y;
	// x = (r_1^2 - r_2^2 + d^2) / 2*d
	x = (pow(transponderRange[first], 2) - pow(transponderRange[second], 2)
			+ pow(fDistance, 2)) / (2 * fDistance);
	// y = +- sqrt( r_1^2 - x^2 )
	y = sqrt(pow(transponderRange[first], 2) - pow(x, 2));

	cout << "Transponder " << first << " frame: X -> " << x << endl;
	cout << "Transponder " << first << " frame: Y -> " << y << endl;

	// convert the solution point into a two possible NED points
	float angle = atan2(transponderNED[second].n - transponderNED[first].n,
			transponderNED[second].e - transponderNED[first].e);
	angle = f_wrapAngleRad(angle);
	cout << "Angle : " << angle << " deg " << osl_core::rad2deg(angle) << endl;

	// cross product
	double cross = x*y*sin(angle);
	cout << "Cross product " << cross << endl;

	double n_prima, e_prima;
	if (cross >= 0.0) {
		n_prima = x * sin(angle) + y * cos(angle);
		e_prima = x * cos(angle) - y * sin(angle);
	}
	else {
		angle = angle - M_PI/2;
		n_prima = y * sin(angle) + x * cos(angle);
		e_prima = y * cos(angle) - x * sin(angle);
	}
	cout << " N' " << n_prima << endl;
	cout << " E' " << e_prima << endl;
	firstSolutionNED.n = n_prima + transponderNED[first].n;
	firstSolutionNED.e = e_prima + transponderNED[first].e;
	cout << "First LBL point NED  : " << firstSolutionNED << endl;

	secondSolutionNED.n = transponderNED[first].n + x * sin(angle);
	secondSolutionNED.e = transponderNED[first].e - y * cos(angle);
	cout << "Second LBL point NED : " << secondSolutionNED << endl;

	firstSolutionLLD = globalCoordConverter.NED2LLD(firstSolutionNED);
	secondSolutionLLD = globalCoordConverter.NED2LLD(secondSolutionNED);

	cout << "First LBL point LLD  : " << firstSolutionLLD << endl;
	cout << "Second LBL point LLD : " << secondSolutionLLD << endl;

	// now return the closest to the current position
	firstSolutionOffset.n = firstSolutionNED.n - north;
	firstSolutionOffset.e = firstSolutionNED.e - east;
	firstSolutionOffset.d = 0;
	double firstDist2CurrentPos = sqrt(pow(firstSolutionOffset.n, 2)
			+ pow(firstSolutionOffset.e, 2));
	secondSolutionOffset.n = secondSolutionNED.n - north;
	secondSolutionOffset.e = secondSolutionNED.e - east;
	secondSolutionOffset.d = 0;
	double secondDist2CurrentPos = sqrt(pow(secondSolutionOffset.n, 2)
			+ pow(secondSolutionOffset.e, 2));

	cout << "First distance to current pos " << firstDist2CurrentPos << endl;
	cout << "Second distance to current pos " << secondDist2CurrentPos << endl;

	if (firstDist2CurrentPos < secondDist2CurrentPos) {
		solutionLLD = firstSolutionLLD;
		offset2current = firstSolutionOffset;
		distance2current = firstDist2CurrentPos;
	} else {
		solutionLLD = secondSolutionLLD;
		offset2current = secondSolutionOffset;
		distance2current = secondDist2CurrentPos;
	}

	return true;
}

//---------------------------------------------------------------------------

void navigation::sendROSNavSts() {
#ifdef USING_ROS
	PRINTLN_DEBUG("Publishing ros nav msg.");

	m_rosNavMsg.header.stamp = ros::Time::now();
	m_rosNavMsg.origin.latitude = originLLD.lat;
	m_rosNavMsg.origin.longitude = originLLD.lon;

	m_rosNavMsg.global_position.latitude = latitude;
	m_rosNavMsg.global_position.longitude = longitude;

	m_rosNavMsg.position.north = north;
	m_rosNavMsg.position.east = east;
	m_rosNavMsg.position.depth = depth;
	m_rosNavMsg.altitude = altitude;

	m_rosNavMsg.orientation.roll = osl_core::deg2rad(roll);
	m_rosNavMsg.orientation.pitch = osl_core::deg2rad(pitch);
	m_rosNavMsg.orientation.yaw = osl_core::deg2rad(yaw);

	m_rosNavMsg.body_velocity.x = surgeVelocity;
	m_rosNavMsg.body_velocity.y = swayVelocity;
	m_rosNavMsg.body_velocity.z = heaveVelocity;

	m_rosNavMsg.orientation_rate.roll = osl_core::deg2rad(rollRate);
	m_rosNavMsg.orientation_rate.pitch = osl_core::deg2rad(pitchRate);
	m_rosNavMsg.orientation_rate.yaw = osl_core::deg2rad(yawRate);

	//m_rosNavMsg.mode = auv_msgs::NavSts::MODE_DEFAULT;
	m_rosNavMsg.status = auv_msgs::NavSts::STATUS_ALL_OK;

	m_rosNavPub.publish(m_rosNavMsg);

        PRINTLN_DEBUG("Publishing filtered ros nav msg.");

        m_rosFiltNavMsg.header.stamp = ros::Time::now();
        m_rosFiltNavMsg.origin.latitude = originLLD.lat; // not filtered, just to be consistent with format
        m_rosFiltNavMsg.origin.longitude = originLLD.lon; // not filtered, just to be consistent with format

        m_rosFiltNavMsg.global_position.latitude = latitude; // not filtered, just to be consistent with format
        m_rosFiltNavMsg.global_position.longitude = longitude;// not filtered, just to be consistent with format

        m_rosFiltNavMsg.position.north = northFiltered;
        m_rosFiltNavMsg.position.east = eastFiltered;
        m_rosFiltNavMsg.position.depth = depthFiltered;
        m_rosFiltNavMsg.altitude = altitudeFiltered;

        m_rosFiltNavMsg.orientation.roll = osl_core::deg2rad(roll);// not filtered, just to be consistent with format
        m_rosFiltNavMsg.orientation.pitch = osl_core::deg2rad(pitchFiltered);
        m_rosFiltNavMsg.orientation.yaw = osl_core::deg2rad(yawFiltered);

        m_rosFiltNavMsg.body_velocity.x = surgeVelocityFiltered;
        m_rosFiltNavMsg.body_velocity.y = swayVelocityFiltered;
        m_rosFiltNavMsg.body_velocity.z = heaveVelocityFiltered;

        m_rosFiltNavMsg.orientation_rate.roll = osl_core::deg2rad(rollRate);// also not filtered
        m_rosFiltNavMsg.orientation_rate.pitch = osl_core::deg2rad(pitchRateFiltered);
        m_rosFiltNavMsg.orientation_rate.yaw = osl_core::deg2rad(yawRateFiltered);

        //m_rosNavMsg.mode = auv_msgs::NavSts::MODE_DEFAULT;
        m_rosFiltNavMsg.status = auv_msgs::NavSts::STATUS_ALL_OK;

        m_rosFiltNavPub.publish(m_rosFiltNavMsg);
	


#endif // ifdef USING_ROS
}

// ---------------------------------------------------------------------------
void navigation::sendROSOdometry() {
#ifdef USING_ROS
	// This message isn't needed by the AUV, but allows us to use ROS rviz to
	// visualise the vehicle position.

	// Map from AUV to ROS coordinates.
	double rosX, rosY, rosZ, rosRoll, rosPitch, rosYaw;
	osl_core::mapAxisAUVtoROS(north, east, depth, rosX, rosY, rosZ);
	osl_core::mapAxisAUVtoROS(osl_core::deg2rad(roll), osl_core::deg2rad(pitch), osl_core::deg2rad(yaw),
			rosRoll, rosPitch, rosYaw);
	geometry_msgs::Quaternion rosOrientation = tf::createQuaternionMsgFromRollPitchYaw(rosRoll, rosPitch, rosYaw);

	m_rosOdometryMsg.header.stamp = ros::Time::now();
	// TODO: Start sending odom as well as map transforms
	m_rosOdometryMsg.header.frame_id = tfMapFrame_;
	m_rosOdometryMsg.child_frame_id = tfBaseLinkFrame_;
	m_rosOdometryMsg.pose.pose.position.x = rosX;
	m_rosOdometryMsg.pose.pose.position.y = rosY;
	m_rosOdometryMsg.pose.pose.position.z = rosZ;
	m_rosOdometryMsg.pose.pose.orientation = rosOrientation;

	m_rosOdometryPub.publish(m_rosOdometryMsg);

	// Now create Transform msg.
	// Broadcaster needs to be static not member variable as it cannot be created before
	// ModuleCore has initialised ROS.
	static tf::TransformBroadcaster rosTFBroadcaster;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(rosX, rosY, rosZ));
	transform.setRotation(tf::createQuaternionFromRPY(rosRoll, rosPitch, rosYaw));
	rosTFBroadcaster.sendTransform(tf::StampedTransform(transform, m_rosNavMsg.header.stamp,
					m_rosOdometryMsg.header.frame_id, m_rosOdometryMsg.child_frame_id));

#endif // USING_ROS
}

//---------------------------------------------------------------------------

void navigation::sendALINavSts() {
#ifdef USING_OSH
	/* set values of the nav nessage and send message */
	navMsg.SetNorth(north);
	navMsg.SetEast(east);
	navMsg.SetAltitude(altitude);
	navMsg.SetDepth(depth);

	navMsg.SetRoll(roll);
	navMsg.SetPitch(pitch);
	navMsg.SetYaw(yaw);

	navMsg.SetSurgeVelocity(surgeVelocity);
	navMsg.SetSwayVelocity(swayVelocity);
	navMsg.SetHeaveVelocity(heaveVelocity);

	navMsg.SetRollVelocity(rollRate);
	navMsg.SetPitchVelocity(pitchRate);
	navMsg.SetYawVelocity(yawRate);

	navMode = ALI_NAVIGATION_MODE_DEFAULT;
	navStatus = ALI_NAVIGATION_STATUS_OK;

	navMsg.SetStatus(navStatus); /* to be checked */
	navMsg.SetMode(navMode); /* to be checked */

	navMsg.SetLat(latitude);
	navMsg.SetLon(longitude);

	navMsg.SetOriginLat(originLLD.lat);
	navMsg.SetOriginLon(originLLD.lon);

	sendMsg(navMsg);
#endif // USING_OSH
}

//---------------------------------------------------------------------------

void navigation::cleanup() {
	PRINTLN_INFO("cleanup entered");

}
//---------------------------------------------------------------------------
void storeFilteringResults(olibVector<double64> filteredStateVec)
{
	northFiltered = filteredStateVec[NORTH_INDEX];
	eastFiltered  = filteredStateVec[EAST_INDEX];
	depthFiltered = filteredStateVec[DEPTH_INDEX];
	altitudeFiltered = filteredStateVec[ALTITUDE_INDEX];
	surgeVelocityFiltered = filteredStateVec[SURGE_VEL_INDEX]; 
	swayVelocityFiltered = filteredStateVec[SWAY_VEL_INDEX]; 
	heaveVelocityFiltered = filteredStateVec[HEAVE_VEL_INDEX];
	yawFiltered = filteredStateVec[YAW_INDEX];
	pitchFiltered = filteredStateVec[PITCH_INDEX];
	yawRateFiltered = filteredStateVec[YAW_RATE_INDEX];
	pitchRateFiltered = filteredStateVec[PITCH_RATE_INDEX];
}
// ---------------------------------------------------------------------------

int main(int argc, char * argv[]) {
	navigation modCore;
	return modCore.main(argc, argv);
}
