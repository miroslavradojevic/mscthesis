#include "filt.h"

/* extended Kalman filter 5dof class definition */

using namespace std;
// class extendedKalmanFilter function definitions
void extendedKalmanFilter5DOF::setElapsedPeriod(double64 & dT_input){ // 
	dT = dT_input;
}

extendedKalmanFilter5DOF::extendedKalmanFilter5DOF()
: olibKalmanFilter(NORMAL_STATES_NR, ANGULAR_STATES_NR),
  m_W(SYSTEM_STATE_LEN, PROCESS_NOISE_LEN, 0)
{
m_Q.setNumberRows(PROCESS_NOISE_LEN); 
m_Q.setNumberColumns(PROCESS_NOISE_LEN); // noise vector has not the same length as process vector
dT = 0.0; //dT initial
m_smootherSwitch = false;
}

olibVector<double64> extendedKalmanFilter5DOF::stateTransition(const olibVector<double64> & X_k_1, double64 dT){
	// non-linear state transition
	uint32 inputVectorSize = X_k_1.size();
	olibVector<double64> X_k(inputVectorSize, 0);
	/* X_k[0] is north; 1 is east, 2 is depth, 3 is altitude, 4 is surgeVel, 5 is swayVel, 6 is heaveVel, ---> normal states 
	        7 is yaw, 8 is pitch, 9 is yawRate, 10 is pitchRate   ---> angular states 
	*/
	// north transition
	// north + (surgeVel*dT)*cos(yaw)*cos(pitch) - (swayVel*dT)*sin(yaw)*cos(pitch);
	X_k[0] = X_k_1[0] + (X_k_1[4]*dT)*cos(X_k_1[7])*cos(X_k_1[8]) - (X_k_1[5]*dT)*sin(X_k_1[7])*cos(X_k_1[8]);
	//printf("dT in pred: %f\n", dT);
	// east transition
	// east  + (surgeVel*dT)*sin(yaw)*cos(pitch) + (swayVel*dT)*cos(yaw)*cos(pitch);
	X_k[1] = X_k_1[1] + (X_k_1[4]*dT)*sin(X_k_1[7])*cos(X_k_1[8]) + (X_k_1[5]*dT)*cos(X_k_1[7])*cos(X_k_1[8]);
	// depth transition
	// depth + (heaveVel*dT)*cos(pitch);
	X_k[2] = X_k_1[2] + (X_k_1[6]*dT)*cos(X_k_1[8]);
	// altitude transition
	// altitude - (heaveVel*dT)*cos(pitch);
	X_k[3] = X_k_1[3] - (X_k_1[6]*dT)*cos(X_k_1[8]);
	// surgeVel
	// stays the same in prediction
	X_k[4] = X_k_1[4];
	// swayVel
	// also the same
	X_k[5] = X_k_1[5];
	// heaveVel
	// also the same
	X_k[6] = X_k_1[6];
	// yaw
	// yaw   + vyaw*dT;
	X_k[7] = X_k_1[7] + (X_k_1[9]*dT);
	// pitch
	// pitch + vpitch*dT;
	X_k[8] = X_k_1[8] + (X_k_1[10]*dT);
	// yawRate
	// stays the same
	X_k[9] = X_k_1[9];
	//pitchRate
	// stays the same
	X_k[10] = X_k_1[10];
	return X_k;
}

void extendedKalmanFilter5DOF::Predict(void)
{
  //printf("olib KF predict\n");	
  // Prediction process
  /////////////////////////////////////////////////////

  //cout << m_xCorrected << endl;
  //cout << m_PCorrected << endl;

  // Predict the state: x = Phi*x; 
  // since it is EKF now, no matrix multiplication - instead usage of the state transition function
  // m_xPrediction = m_Phi*m_xCorrected;
  m_xPrediction = stateTransition(m_xCorrected, dT);
  // Wrap around angular states
  for(m_i = 0; m_i < m_angularIndx.size(); m_i ++)
    {
      //cout << "state " << m_angularIndx[m_i] << " in prediction was wrapped from " << m_xPrediction[(uint32) m_angularIndx[m_i]];
      m_xPrediction[(uint32) m_angularIndx[m_i]] =  Deg2Rad*olibWrap180(Rad2Deg*m_xPrediction[(uint32) m_angularIndx[m_i]]);
      //cout << " to " <<  Deg2Rad*olibWrap180(Rad2Deg*m_xPrediction[(uint32) m_angularIndx[m_i]]) << endl;
    }
		
  // Predict the covariance matrix: P = F*P*F' + Q;
  // EKF5DOF a bit different formula: P = F*P*F' + W*Q*W';
  //m_PPrediction = m_F*m_PCorrected*trans(m_F)+ m_Q;
  m_PPrediction = m_F*m_PCorrected*trans(m_F)+ m_W*m_Q*trans(m_W);
  // cout << m_xPrediction << endl;
  // cout << m_PPrediction << endl;
}

/* function that initializes filtering class*/
void extendedKalmanFilter5DOF::init(filterMode whichMode){

	firstMsgRecieved = false;
	prevMsgTime = 0.0;
	dT = 0.0;
	enableFiltering = true;
	//usePressSensor = useDvlSensor = useCompassSensor = useGyroSensor =  true;
	//useLblSensor = true;
	ekfMode = whichMode;
					/*
				// (nameInTheCode, nameInTheFile, sectionName)
				if (!getINIFloat(SDnorth, "SDnorth", "EKF"))
						return false;
				if (!getINIFloat(SDeast, "SDeast", "EKF"))
						return false;
				if (!getINIFloat(SDdepth, "SDdepth", "EKF"))
						return false;
				if (!getINIFloat(SDaltitude, "SDaltitude", "EKF"))
						return false;
				if (!getINIFloat(SDu, "SDu", "EKF"))
						return false;
				if (!getINIFloat(SDv, "SDv", "EKF"))
						return false;
				if (!getINIFloat(SDw, "SDw", "EKF"))
						return false;
				if (!getINIFloat(SDyaw, "SDyaw", "EKF"))
						return false;
				if (!getINIFloat(SDpitch, "SDpitch", "EKF"))
						return false;		
				if (!getINIFloat(SDyawRate, "SDyawRate", "EKF"))
						return false;
				if (!getINIFloat(SDpitchRate, "SDpitchRate", "EKF"))
						return false;
		
		
				if (!getINIFloat(SDuModel, "SDuModel", "EKF"))
						return false;
				if (!getINIFloat(SDvModel, "SDvModel", "EKF"))
						return false;
				if (!getINIFloat(SDwModel, "SDwModel", "EKF"))
						return false;	
				if (!getINIFloat(SDyawRateModel, "SDyawRateModel", "EKF"))
						return false;
				if (!getINIFloat(SDpitchRateModel, "SDpitchRateModel", "EKF"))
						return false;			
					*/					
	// Kalman params
	ifstream inFile;
	inFile.open("/home/miroslav/params.txt");
	if (!inFile) 
	{
		cerr << "Unable to open file datafile.txt\n";
		exit(1);   // call system to stop
	}
	// read each parameter
	inFile >> SDnorth; inFile >> SDeast; inFile >> SDdepth;	inFile >> SDaltitude;
	inFile >>  SDu; inFile >>  SDv; inFile >> SDw;
	inFile >> SDyaw; inFile >> SDpitch; inFile >> SDyawRate; inFile >> SDpitchRate;
	//model
	inFile >> SDuModel; inFile >>  SDvModel; inFile >>  SDwModel;
	inFile >>  SDyawRateModel; inFile >>  SDpitchRateModel;
	
	cout << "init stage: \n" << "SDnorth: " << SDnorth << "    SDpitchRateModel: " << SDpitchRateModel << endl;
	
	// initialize state to zeros before recieving the first message
	olibVector<double64> X(SYSTEM_STATE_LEN, 0);
	setXCorrected(X);
	setXPrediction(X);
	// initialize uncertainties
	olibMatrix<double64> P(SYSTEM_STATE_LEN, SYSTEM_STATE_LEN, 0);
		P[0][0] = pow(SDnorth,2);    P[1][1] = pow(SDeast,2); P[2][2] = pow(SDdepth,2);
		P[3][3] = pow(SDaltitude,2); P[4][4] = pow(SDu,2);    P[5][5] = pow(SDv,2);
		P[6][6] = pow(SDw,2);        P[7][7] = pow(SDyaw,2);  P[8][8] = pow(SDpitch,2);
		P[9][9] = pow(SDyawRate,2);  P[10][10] = pow(SDpitchRate,2);
	setPCorrected(P);
	setPPrediction(P);
	olibMatrix<double64> Q(PROCESS_NOISE_LEN, PROCESS_NOISE_LEN, 0);
		Q[0][0] = pow(SDuModel,2);
		Q[1][1] = pow(SDvModel,2);
		Q[2][2] = pow(SDwModel,2);
		Q[3][3] = pow(SDyawRateModel,2);
		Q[4][4] = pow(SDpitchRateModel,2);
	setQ(Q);
}

void extendedKalmanFilter5DOF::updateTime(double64 & current_time)
{
	//update dt and previous message time
	dT = current_time - prevMsgTime;
	prevMsgTime = current_time;
	
}

void extendedKalmanFilter5DOF::setFilteringPossible(void)
{
	firstMsgRecieved = true;
}

bool extendedKalmanFilter5DOF::isFilteringPossible(void)
{
	return firstMsgRecieved;
}	

void extendedKalmanFilter5DOF::setW (void)
{
	//should be fixed to 11x5 size
	//north
	m_W[0][0] =  1/2*pow(dT,2)*cos(m_xCorrected[7])*cos(m_xCorrected[8]);
	m_W[0][1] = -1/2*pow(dT,2)*sin(m_xCorrected[7])*cos(m_xCorrected[8]);
	m_W[0][2] = 0;
	m_W[0][3] = 0;
	m_W[0][4] = 0;
	//east
	m_W[1][0] =  1/2*pow(dT,2)*sin(m_xCorrected[7])*cos(m_xCorrected[8]);
	m_W[1][1] = -1/2*pow(dT,2)*cos(m_xCorrected[7])*cos(m_xCorrected[8]);
	m_W[1][2] = 0;
	m_W[1][3] = 0;
	m_W[1][4] = 0;	
	//depth
	m_W[2][0] = 0;
	m_W[2][1] = 0;
	m_W[2][2] = (1/2)*pow(dT,2)*cos(m_xCorrected[8]);
	m_W[2][3] = 0;
	m_W[2][4] = 0;	
	//altitude
	m_W[3][0] = 0;
	m_W[3][1] = 0;
	m_W[3][2] = -(1/2)*pow(dT,2)*cos(m_xCorrected[8]);
	m_W[3][3] = 0;
	m_W[3][4] = 0;	
	//surgeVel
	m_W[4][0] = dT;
	m_W[4][1] = 0;
	m_W[4][2] = 0;
	m_W[4][3] = 0;
	m_W[4][4] = 0;
	//swayVel
	m_W[5][0] = 0;
	m_W[5][1] = dT;
	m_W[5][2] = 0;
	m_W[5][3] = 0;
	m_W[5][4] = 0;
	//heaveVel
	m_W[6][0] = 0;
	m_W[6][1] = 0;
	m_W[6][2] = dT;
	m_W[6][3] = 0;
	m_W[6][4] = 0;	
	//yaw
	m_W[7][0] = 0;
	m_W[7][1] = 0;
	m_W[7][2] = 0;
	m_W[7][3] = (1/2)*pow(dT,2);
	m_W[7][4] = 0;
	//pitch
	m_W[8][0] = 0;
	m_W[8][1] = 0;
	m_W[8][2] = 0;
	m_W[8][3] = 0;
	m_W[8][4] = (1/2)*pow(dT,2);
	//yawRate
	m_W[9][0] = 0;
	m_W[9][1] = 0;
	m_W[9][2] = 0;
	m_W[9][3] = dT;
	m_W[9][4] = 0;	
	//pitchRate
	m_W[10][0] = 0;
	m_W[10][1] = 0;
	m_W[10][2] = 0;
	m_W[10][3] = 0;
	m_W[10][4] = dT;
}

void extendedKalmanFilter5DOF::setF (void){
	//fixed size 11x11
	//north
	m_F[0][0] = 1;
	m_F[0][1] = 0;
	m_F[0][2] = 0;
	m_F[0][3] = 0;
	m_F[0][4] =  dT*cos(m_xCorrected[7])*cos(m_xCorrected[8]);
	m_F[0][5] = -dT*sin(m_xCorrected[7])*cos(m_xCorrected[8]);
	m_F[0][6] = 0;
	m_F[0][7] = -(m_xCorrected[4]*dT)*sin(m_xCorrected[7])*cos(m_xCorrected[8])-(m_xCorrected[5]*dT)*cos(m_xCorrected[7])*cos(m_xCorrected[8]);	
	m_F[0][8] = -(m_xCorrected[4]*dT)*cos(m_xCorrected[7])*sin(m_xCorrected[8])+(m_xCorrected[5]*dT)*sin(m_xCorrected[7])*sin(m_xCorrected[8]);
	m_F[0][9] =  0;
	m_F[0][10] = 0;
	
	
	//east
	m_F[1][0] = 0;
	m_F[1][1] = 1;
	m_F[1][2] = 0;
	m_F[1][3] = 0;
	m_F[1][4] =  dT*sin(m_xCorrected[7])*cos(m_xCorrected[8]);
	m_F[1][5] = -dT*cos(m_xCorrected[7])*cos(m_xCorrected[8]);
	m_F[1][6] = 0;
	m_F[1][7] = (m_xCorrected[4]*dT)*cos(m_xCorrected[7])*cos(m_xCorrected[8])-(m_xCorrected[5]*dT)*sin(m_xCorrected[7])*cos(m_xCorrected[8]);	
	m_F[1][8] = -(m_xCorrected[4]*dT)*sin(m_xCorrected[7])*sin(m_xCorrected[8])-(m_xCorrected[5]*dT)*cos(m_xCorrected[7])*sin(m_xCorrected[8]);
	m_F[1][9] =  0;
	m_F[1][10] = 0;	
	
	//depth
	m_F[2][0] = 0;
	m_F[2][1] = 0;
	m_F[2][2] = 1;
	m_F[2][3] = 0;
	m_F[2][4] = 0;
	m_F[2][5] = 0;
	m_F[2][6] = dT*cos(m_xCorrected[8]);
	m_F[2][7] = 0;	
	m_F[2][8] = -(m_xCorrected[6]*dT)*sin(m_xCorrected[8]);
	m_F[2][9] = 0;
	m_F[2][10]= 0;
	
	//altitude
	m_F[3][0] = 0;
	m_F[3][1] = 0;
	m_F[3][2] = 0;
	m_F[3][3] = 1;
	m_F[3][4] = 0;
	m_F[3][5] = 0;
	m_F[3][6] = -dT*cos(m_xCorrected[8]);
	m_F[3][7] = 0;	
	m_F[3][8] = (m_xCorrected[6]*dT)*sin(m_xCorrected[8]);
	m_F[3][9] = 0;
	m_F[3][10]= 0;
	
	//surgeVel
	m_F[4][0] = 0;
	m_F[4][1] = 0;
	m_F[4][2] = 0;
	m_F[4][3] = 0;
	m_F[4][4] = 1;
	m_F[4][5] = 0;
	m_F[4][6] = 0;
	m_F[4][7] = 0;	
	m_F[4][8] = 0;
	m_F[4][9] = 0;
	m_F[4][10]= 0;
	
	//swayVel
	m_F[5][0] = 0;
	m_F[5][1] = 0;
	m_F[5][2] = 0;
	m_F[5][3] = 0;
	m_F[5][4] = 0;
	m_F[5][5] = 1;
	m_F[5][6] = 0;
	m_F[5][7] = 0;	
	m_F[5][8] = 0;
	m_F[5][9] = 0;
	m_F[5][10]= 0;
	
	//heaveVel
	m_F[6][0] = 0;
	m_F[6][1] = 0;
	m_F[6][2] = 0;
	m_F[6][3] = 0;
	m_F[6][4] = 0;
	m_F[6][5] = 0;
	m_F[6][6] = 1;
	m_F[6][7] = 0;	
	m_F[6][8] = 0;
	m_F[6][9] = 0;
	m_F[6][10]= 0;						

	//yaw
	m_F[7][0] = 0;
	m_F[7][1] = 0;
	m_F[7][2] = 0;
	m_F[7][3] = 0;
	m_F[7][4] = 0;
	m_F[7][5] = 0;
	m_F[7][6] = 0;
	m_F[7][7] = 1;	
	m_F[7][8] = 0;
	m_F[7][9] = dT;
	m_F[7][10]= 0;
	
	//pitch
	m_F[8][0] = 0;
	m_F[8][1] = 0;
	m_F[8][2] = 0;
	m_F[8][3] = 0;
	m_F[8][4] = 0;
	m_F[8][5] = 0;
	m_F[8][6] = 0;
	m_F[8][7] = 0;	
	m_F[8][8] = 1;
	m_F[8][9] = 0;
	m_F[8][10]= dT;	
	
	//yawRate
	m_F[9][0] = 0;
	m_F[9][1] = 0;
	m_F[9][2] = 0;
	m_F[9][3] = 0;
	m_F[9][4] = 0;
	m_F[9][5] = 0;
	m_F[9][6] = 0;
	m_F[9][7] = 0;	
	m_F[9][8] = 0;
	m_F[9][9] = 1;
	m_F[9][10]= 0;	
	
	//pitchRate
	m_F[10][0] = 0;
	m_F[10][1] = 0;
	m_F[10][2] = 0;
	m_F[10][3] = 0;
	m_F[10][4] = 0;
	m_F[10][5] = 0;
	m_F[10][6] = 0;
	m_F[10][7] = 0;	
	m_F[10][8] = 0;
	m_F[10][9] = 0;
	m_F[10][10]= 1;	
}

olibMatrix<double64> & extendedKalmanFilter5DOF::getW ()
{
	return m_W;
}
double64 & extendedKalmanFilter5DOF::getDt ()
{
	return dT;
} 
//handle observation periodically
olibVector<double64> extendedKalmanFilter5DOF::filterObservation(olibVector<double64> observation, olibVector<stateID> statesInObservation, double64 dT_interval)
{
	//updateTime(current_time);
	setElapsedPeriod(dT_interval);
	// prediction happens first even if there is no kalman filtering enabled
	setW(); setF(); Predict(); // extendedKalmanFilter5DOF methods
	
	// allocate H and R matrices for update stage	
	uint32 lengthObservation; 
	lengthObservation = statesInObservation.size(); // length of the observation vector
	
	//if(enableFiltering)
	//{
		olibMatrix<double64> H(lengthObservation, SYSTEM_STATE_LEN);// initialize with zeros
		olibMatrix<double64> R(lengthObservation, lengthObservation);
		
		for(uint32 i=0; i<lengthObservation ; i++)
		{
			switch(statesInObservation[i])
			{
				case NULL_STATE:
				{
					//nothing
				}	
				case NORTH:
				{
					H[i][NORTH_INDEX] = 1; R[i][i] = pow(SDnorth,2);
				}
				case EAST:
				{
					H[i][EAST_INDEX] = 1;  R[i][i] = pow(SDeast,2);
				}
				case DEPTH:
				{
					H[i][DEPTH_INDEX] = 1; R[i][i] = pow(SDdepth,2);
				}
				case ALTITUDE:
				{
					H[i][ALTITUDE_INDEX] = 1; R[i][i] = pow(SDaltitude,2);
				}
				case SURGE_VELOCITY:
				{
					H[i][SURGE_VEL_INDEX] = 1; R[i][i] = pow(SDu,2);
				}
				case SWAY_VELOCITY:
				{
					H[i][SWAY_VEL_INDEX] = 1; R[i][i] = pow(SDv,2);
				}	
				case HEAVE_VELOCITY:
				{
					H[i][HEAVE_VEL_INDEX] = 1; R[i][i] = pow(SDw,2);
				}
				case YAW:
				{
					H[i][YAW_INDEX] = 1; R[i][i] = pow(SDyaw,2);
				}
				case PITCH:
				{
					H[i][PITCH_INDEX] = 1; R[i][i] = pow(SDpitch,2);
				}
				case YAW_RATE:
				{
					H[i][YAW_RATE_INDEX] = 1; R[i][i] = pow(SDyawRate,2);
				}
				case PITCH_RATE:
				{
					H[i][PITCH_RATE_INDEX] = 1; R[i][i] = pow(SDpitchRate,2);
				}																															
			}//switch(statesInObservation[i])
		}//for loop
		// do update 
		// temporary measurement object attached with current class
		olibKalmanMeasure measurementEKF5DOF(lengthObservation, this);
		measurementEKF5DOF.setz(observation);
		measurementEKF5DOF.setR(R);
		measurementEKF5DOF.setJ(H);
		// predicted observation
		olibVector<double64> Hx;
		Hx = H * this->getXPrediction();
		measurementEKF5DOF.seth(Hx);
		this->Correct(measurementEKF5DOF);
		return this->getXCorrected();
	//}// if(enableFiltering)
	//else
	//{
		// just prediction
	//	return this->getXPrediction();
	//}
}
// handle observation after each message
olibVector<double64> extendedKalmanFilter5DOF::filterMessage(olibVector<double64> measurement, sensorType whichSensor, double64 current_time)
{
// expected measurement indexing (input has to be consistent):
// DEPTH_SENSOR: [0]- depth [1] - heaveVelocity
// COMPASS_SENSOR: [0] - pitch [1]- pitchRate [2]- yaw
// GYRO_SENSOR: [0] - yawRate
// DVL_SENSOR: [0] - altitude [1] - surgeVelocity [2] - swayVelocity
// LBL_SENSOR: [0] - north [1] - east
	updateTime(current_time);
	// prediction happens first even if there is no kalman filtering enabled
	setW(); setF(); Predict(); // extendedKalmanFilter5DOF methods
	
	// allocate H and R matrices for observation stage
	uint32 lengthMeasurement;
	lengthMeasurement = measurement.size(); // length of the measurement vector
	//olibVector<double64> Z(lengthMeasurement, 0);
	
	//if(enableFiltering)
	//{
		olibMatrix<double64> H(lengthMeasurement, SYSTEM_STATE_LEN);
		olibMatrix<double64> R(lengthMeasurement, lengthMeasurement);
		switch(whichSensor)
		{
			case NULL_SENSOR:
			{
				// nothing
			}	
			case DEPTH_SENSOR:
			{// depth sensor: depth and heave velocity update, takes two measurements
				//measurement[0] is considered as depth //depth taken from navigation class
				//measurement[1] is considered as heaveVelocity //heave Vel taken member of navigation class
				olibMatrix<double64> H(lengthMeasurement, SYSTEM_STATE_LEN);
				H[0][DEPTH_INDEX]     = 1;
				H[1][HEAVE_VEL_INDEX] = 1;
				olibMatrix<double64> R(lengthMeasurement, lengthMeasurement);
				R[0][0] = pow(SDdepth,2);
				R[1][1] = pow(SDw,2);
				break;
			}
			case COMPASS_SENSOR:
			{// compass: pitch, pitchRate, yaw ( roll is not needed since it's 5DOF)
				//measurement[0] is considered as pitch
				//measurement[1] is considered as pitchRate
				//measurement[2] is considered as yaw // infamous yaw from compass
				olibMatrix<double64> H(lengthMeasurement, SYSTEM_STATE_LEN);  
				H[0][PITCH_INDEX]      = 1; 
				H[1][PITCH_RATE_INDEX] = 1;
				H[2][YAW_INDEX]        = 1;
				olibMatrix<double64> R(lengthMeasurement, lengthMeasurement);			
				R[0][0] = pow(SDpitch,2);
				R[1][1] = pow(SDpitchRate,2);
				R[2][2] = pow(SDyaw,2);
				break;
			}
			case GYRO_SENSOR:
			{//gyro:  yaw rate
				//measurement[0] is considered as yawRate
				olibMatrix<double64> H(lengthMeasurement, SYSTEM_STATE_LEN);
				H[0][YAW_RATE_INDEX] = 1;
				olibMatrix<double64> R(lengthMeasurement, lengthMeasurement);
				R[0][0] = pow(SDyawRate,2);
				break;
			}
			case DVL_SENSOR:
			{// altitude, surge and sway velocity
				//measurement[0] is considered as altitude
				//measurement[1] is considered as surgeVelocity
				//measurement[2] is considered as swayVelocity
				olibMatrix<double64> H(lengthMeasurement, SYSTEM_STATE_LEN);
				H[0][ALTITUDE_INDEX]  = 1;
				H[1][SURGE_VEL_INDEX] = 1;
				H[2][SWAY_VEL_INDEX]  = 1;
				olibMatrix<double64> R(lengthMeasurement, lengthMeasurement);
				R[0][0] = pow(SDaltitude,2);
				R[1][1] = pow(SDu,2);
				R[2][2] = pow(SDv,2);
				break;
			}
			case LBL_SENSOR:
			{// lbl updates north, east
				//measurement[0] is considered as north
				//measurement[1] is considered as east
				olibMatrix<double64> H(lengthMeasurement, SYSTEM_STATE_LEN);
				H[0][NORTH_INDEX] = 1;
				H[1][EAST_INDEX]  = 1;
				olibMatrix<double64> R(lengthMeasurement, lengthMeasurement);
				R[0][0] = pow(SDnorth,2);
				R[1][1] = pow(SDeast,2);
				break;
			}
			case GPS_SENSOR:
			{
				//nothing
				break;
			}
		}//switch(whichSensor)
		// do update
		// temporary measurement object attached with current class
		olibKalmanMeasure measurementEKF5DOF(lengthMeasurement, this);
		measurementEKF5DOF.setR(R);
		measurementEKF5DOF.setJ(H);
		measurementEKF5DOF.setz(measurement);
		// predicted observation
		olibVector<double64> Hx;
		Hx = H * this->getXPrediction();
		measurementEKF5DOF.seth(Hx);
		this->Correct(measurementEKF5DOF);
		return this->getXCorrected();
	//}//if(enableFiltering)
	//else
	//{// if filtering not enabled just do prediction
		// just prediction
	//	return this->getXPrediction();			
	//}	
}
