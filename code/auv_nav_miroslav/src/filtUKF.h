#ifndef _FILTUKF_H_
#define _FILTUKF_H_

#include <vector>
#include <OceanLIB/Processing/olibKalmanFilter.h>
#include <OceanLIB/Processing/olibKalmanMeasure.h>
#include <OceanLIB/Abstraction/olibVector.h>
#include <OceanLIB/Abstraction/olibMatrix.h>

//filtering parameters
#define SYSTEM_STATE_LEN   11
#define PROCESS_NOISE_LEN  5
#define NORMAL_STATES_NR   7
#define ANGULAR_STATES_NR  4

//indexing constants - state vector
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

//indexing constants - noise vector
#define SURGE_ACC_INDEX  0
#define SWAY_ACC_INDEX   1
#define HEAVE_ACC_INDEX  2
#define YAW_ACC_INDEX    3
#define PITCH_ACC_INDEX  4

//sensor identifiers
enum sensorType {NULL_SENSOR, DEPTH_SENSOR, COMPASS_SENSOR, GYRO_SENSOR, DVL_SENSOR, LBL_SENSOR, GPS_SENSOR};
//state identifiers
enum stateID {NULL_STATE, NORTH, EAST, DEPTH, ALTITUDE, SURGE_VELOCITY, SWAY_VELOCITY, HEAVE_VELOCITY, YAW, PITCH, YAW_RATE, PITCH_RATE};
//ekf mode: choose when the filter updates
enum filterMode {EVERY_MESSAGE, EVERY_DT};

typedef struct 
{
	float SDnorth; //// position measurement uncertainties 
	float SDeast;
	float SDdepth;
	float SDaltitude;  
	float SDu;
	float SDv;
	float SDw; // velocities measurement uncertainties
	float SDyaw;
	float SDpitch;
	float SDyawRate;
	float SDpitchRate;
	float SDuModel; // model uncertainties
	float SDvModel;
	float SDwModel;
	float SDyawRateModel;
	float SDpitchRateModel;
} T_UKF5DOF_PARAMS;		
// function that will be used for sampling
void unscentedTransformSampling(olibVector<double64> & x, olibMatrix<double64> & P, float k, 
                                olibMatrix<double64> & x_samples, olibVector<double64> & x_weights
                               );
                               
class unscentedKalmanFilter5DOF: public olibKalmanFilter
{    
public:
	//constructor
	unscentedKalmanFilter5DOF ();
	bool enableFiltering;
	filterMode ukfMode;

	float SDnorth, SDeast, SDdepth, SDaltitude; // position measurement uncertainties 
	float SDu, SDv, SDw; // velocities measurement uncertainties
	float SDyaw, SDpitch, SDyawRate, SDpitchRate;
	// model uncertainties
	float SDuModel, SDvModel, SDwModel;
	float SDyawRateModel, SDpitchRateModel;
	
		
protected:
	double64 dT;
	bool firstMsgRecieved;
};                           
#endif
