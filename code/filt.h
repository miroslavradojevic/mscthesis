#ifndef _FILT_H_
#define _FILT_H_

#include <vector>
#include <OceanLIB/Processing/olibKalmanFilter.h>
#include <OceanLIB/Processing/olibKalmanMeasure.h>
#include <OceanLIB/Abstraction/olibVector.h>
#include <OceanLIB/Abstraction/olibMatrix.h>

// to read the parameters - temporary solution, AUV.ini couldn't connect
#include <fstream>

//filtering parameters
#define SYSTEM_STATE_LEN   11
#define PROCESS_NOISE_LEN  5
#define NORMAL_STATES_NR   7
#define ANGULAR_STATES_NR  4

//indexing constants
#define NORTH_INDEX       0
#define EAST_INDEX        1
#define DEPTH_INDEX       2
#define ALTITUDE_INDEX    3
#define SURGE_VEL_INDEX   4
#define SWAY_VEL_INDEX    5
#define HEAVE_VEL_INDEX   6
#define YAW_INDEX         7
#define PITCH_INDEX       8
#define YAW_RATE_INDEX    9
#define PITCH_RATE_INDEX 10

//sensor identifiers
enum sensorType {NULL_SENSOR, DEPTH_SENSOR, COMPASS_SENSOR, GYRO_SENSOR, DVL_SENSOR, LBL_SENSOR, GPS_SENSOR};
//state identifiers
enum stateID {NULL_STATE, NORTH, EAST, DEPTH, ALTITUDE, SURGE_VELOCITY, SWAY_VELOCITY, HEAVE_VELOCITY, YAW, PITCH, YAW_RATE, PITCH_RATE};
//ekf mode: choose when the filter updates
enum filterMode {EVERY_MESSAGE, EVERY_DT};

 
//#ifdef USING_OSH
//#include <OceanALI/Message/aliMsg_NavSts.h>
//#include <OceanALI/Message/aliMsg_GyroSts.h>
//#include <OceanALI/Message/aliMsg_DepthSts.h>
//#include <OceanALI/Message/aliMsg_GPSSts.h>
//#include <OceanALI/Message/aliMsg_TimeDelay.h>
//#include <DVLMsg.h>
//#include <Tcm2Msg.h>
//#endif // USING_OSH

// olibVector and olibMatrix will be used as data type for vectors and matrices
// they've been taken out from OceanLIB
class extendedKalmanFilter5DOF: public olibKalmanFilter
{
// derived from linear Kalman filter already given in OceanLIB (olibKalmanFilter.h),
// with some changes in library functions, intended for constant speed model with 5dof
// just redefinition of the existing methods and added new class members
// redefine virtual functions w.r.t. extended Kalman filter formulas

	// class is intended to do the filtering for navigation task
	// quite fixed values for particular application EKF, 5dof
	// instance of such class is included as the component in navigation class
public:
	//constructor
	extendedKalmanFilter5DOF ();
	
	// data memebers
	///****** following values should eventually end up being obtained from .INI file ******///
	///****** they are given values in init member function ******///
	// enable-disable functions	
	bool enableFiltering;
	//bool usePressSensor, useDvlSensor, useCompassSensor, useGyroSensor, useLblSensor;
	filterMode ekfMode;	
	
	// uncertainties, standard deviations of the measurement and the model
	// measurement standard devs
	float SDnorth, SDeast, SDdepth, SDaltitude; // position measurement uncertainties 
	float SDu, SDv, SDw; // velocities measurement uncertainties
	float SDyaw, SDpitch, SDyawRate, SDpitchRate;
	// model uncertainties
	float SDuModel, SDvModel, SDwModel;
	float SDyawRateModel, SDpitchRateModel;
	///*****************************///

	// member functions
	//////// redefs ////////
	void setF  (void); // no need for setting it, automatically created from data members: dT and previous state
	void Predict (void);// defined so that it implements non-linearity, specific for EKF
	///////////////////////////////////////////////////////////////////
    	//  EXTENDED KALMAN FILTER
	//  loop:
	//  X_predicted = stateTransition(X_corrected) + W * Q * W
	//  X_corrected = X_predicted + Kalman_gain * (observation - predicted_observation)
	//
	void setW  (void); // set W matrix from the definition
	olibMatrix<double64> & getW (void); // read W matrix
	double64 & getDt (void);
	
	 
	
	//specific formulas for this application, AUV, 5dof
	olibVector<double64> stateTransition(const olibVector<double64> & statePrev, double64 dT);
	
	// function that handles OSH MESSAGES - and does filtering (prediction and correction) after each new message from the sensor	
	olibVector<double64> filterMessage(olibVector<double64> measurement, sensorType whichSensor, double64 current_time);
	// function that filters current observation - filtering happens with interval blending together messages from different sensors
	olibVector<double64> filterObservation(olibVector<double64> observation, olibVector<stateID> statesInObservation,  double64 dT_interval); 

	void updateTime(double64 & current_time);
	void setElapsedPeriod(double64 & dT_input);
	
	void init(filterMode whichMode); //initialize filtering, initialize variables, assign parameter values
	
	void setFilteringPossible(void);
	bool isFilteringPossible(void);

protected:
	double64 dT;
	double64 prevMsgTime;
	
	bool firstMsgRecieved;  // flag saying if the first msg was recieved
				// no filtering in case of first message, just initialize	
	 
	/**
	 *Jacobian matrix (normalStates + angularStates, noise)
	 * partial derivative wrt process noise
	 */
	olibMatrix<double64>	m_W; // such matrix had to be added to establish connection between noise and states
								// partial derivative of state transition function per noise
};

#endif
