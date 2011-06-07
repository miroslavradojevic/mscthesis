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
	/* X_k[NORTH_INDEX] is north; 1 is east, 2 is depth, 3 is altitude, 4 is surgeVel, 5 is swayVel, 6 is heaveVel, ---> normal states 
	        7 is yaw, 8 is pitch, 9 is yawRate, 10 is pitchRate   ---> angular states 
	*/
	// north transition
	// north + (surgeVel*dT)*cos(yaw)*cos(pitch) - (swayVel*dT)*sin(yaw)*cos(pitch);
	X_k[NORTH_INDEX] = 
	X_k_1[NORTH_INDEX] + 
	(X_k_1[SURGE_VEL_INDEX]*dT)*cos(X_k_1[YAW_INDEX])*cos(X_k_1[PITCH_INDEX]) - 
	(X_k_1[SWAY_VEL_INDEX] *dT)*sin(X_k_1[YAW_INDEX])*cos(X_k_1[PITCH_INDEX]);
	//X_k[NORTH_INDEX] = 
	//X_k_1[NORTH_INDEX] + 
	//(X_k_1[SURGE_VEL_INDEX]*dT)*cos(X_k_1[YAW_INDEX])- 
	//(X_k_1[SWAY_VEL_INDEX] *dT)*sin(X_k_1[YAW_INDEX]);
	// east transition
	// east  + (surgeVel*dT)*sin(yaw)*cos(pitch) + (swayVel*dT)*cos(yaw)*cos(pitch);
	X_k[EAST_INDEX] = X_k_1[EAST_INDEX] + 
	(X_k_1[SURGE_VEL_INDEX]*dT)*sin(X_k_1[YAW_INDEX])*cos(X_k_1[PITCH_INDEX]) + 
	(X_k_1[SWAY_VEL_INDEX]*dT) *cos(X_k_1[YAW_INDEX])*cos(X_k_1[PITCH_INDEX]);
	//X_k[EAST_INDEX] = X_k_1[EAST_INDEX] + 
	//(X_k_1[SURGE_VEL_INDEX]*dT)*sin(X_k_1[YAW_INDEX]) + 
	//(X_k_1[SWAY_VEL_INDEX]*dT) *cos(X_k_1[YAW_INDEX]);
	// depth transition
	// depth + (heaveVel*dT)*cos(pitch);
	X_k[DEPTH_INDEX] = X_k_1[DEPTH_INDEX] + 
	(X_k_1[HEAVE_VEL_INDEX]*dT)*cos(X_k_1[PITCH_INDEX]);
	// altitude transition
	// altitude - (heaveVel*dT)*cos(pitch);
	//X_k[ALTITUDE_INDEX] = X_k_1[ALTITUDE_INDEX] - (X_k_1[HEAVE_VEL_INDEX]*dT)*cos(X_k_1[PITCH_INDEX]);
	  X_k[ALTITUDE_INDEX] = X_k_1[ALTITUDE_INDEX];// - (X_k_1[HEAVE_VEL_INDEX]*dT)*cos(X_k_1[PITCH_INDEX]);
	// surgeVel
	// stays the same in prediction
	X_k[SURGE_VEL_INDEX] = X_k_1[SURGE_VEL_INDEX];
	// swayVel
	// also the same
	X_k[SWAY_VEL_INDEX] = X_k_1[SWAY_VEL_INDEX];
	// heaveVel
	// also the same
	X_k[HEAVE_VEL_INDEX] = X_k_1[HEAVE_VEL_INDEX];
	// yaw
	// yaw   + vyaw*dT;
	X_k[YAW_INDEX] = X_k_1[YAW_INDEX] + (X_k_1[YAW_RATE_INDEX]*dT);
	// pitch
	// pitch + vpitch*dT;
	X_k[PITCH_INDEX] = X_k_1[PITCH_INDEX] + (X_k_1[PITCH_RATE_INDEX]*dT);
	// yawRate
	// stays the same
	X_k[YAW_RATE_INDEX] = X_k_1[YAW_RATE_INDEX];
	//pitchRate
	// stays the same
	X_k[PITCH_RATE_INDEX] = X_k_1[PITCH_RATE_INDEX];
	return X_k;
}

void extendedKalmanFilter5DOF::Predict(void)
{
  //printf("olib KF predict\n");	
  // Prediction process
  /////////////////////////////////////////////////////

  //cout <<m_xCorrected << endl;
  //cout << m_PCorrected << endl;

  // Predict the state: x = Phi*x; 
  // since it is EKF now, no matrix multiplication - instead usage of the state transition function
  // m_xPrediction = m_Phi*m_xCorrected;
  m_xPrediction = stateTransition(m_xCorrected, dT);
  // Wrap around angular states 0-360 [degrees]
  for(m_i = 0; m_i < m_angularIndx.size(); m_i ++)
    {
      //cout << "state " << m_angularIndx[m_i] << " in prediction was wrapped from " << m_xPrediction[(uint32) m_angularIndx[m_i]];
      m_xPrediction[(uint32) m_angularIndx[m_i]] =  Deg2Rad*olibWrap180(Rad2Deg*m_xPrediction[(uint32) m_angularIndx[m_i]]);
      //cout << " to " <<  Deg2Rad*olibWrap360(Rad2Deg*m_xPrediction[(uint32) m_angularIndx[m_i]]) << endl;
    }
  // Predict the covariance matrix: P = F*P*F' + Q;
  // EKF5DOF a bit different formula: P = F*P*F' + W*Q*W';
  //m_PPrediction = m_F*m_PCorrected*trans(m_F)+ m_Q;
  m_PPrediction = m_F*m_PCorrected*trans(m_F)+ m_W*m_Q*trans(m_W);
  /*
  ofstream fout;
  fout.open("ekfFiltProgress.dat",ios::app);    // open file for appending
  fout << "F: \n" << m_F << endl;
  fout << "W: \n" << m_W << endl;
  fout << "Q: \n" << m_Q << endl;
  fout << "Ppred: \n" << m_PPrediction << endl;
  fout.close( );       //close file
  */
}

/* function that initializes filtering class*/
void extendedKalmanFilter5DOF::init(filterMode whichMode, T_EKF5DOF_PARAMS parameters){

	

	firstMsgRecieved = false;
	dT = 0.0;
	enableFiltering = true;
	ekfMode = whichMode;
	// Kalman params
	SDnorth = parameters.SDnorth;//0; //
	SDeast  = parameters.SDeast;//0; //
	SDdepth = parameters.SDdepth;//0; //
	SDaltitude = parameters.SDaltitude;//0; //p
	SDu        = parameters.SDu;//0; //;
	SDv        = parameters.SDv;//0; //
	SDw        = parameters.SDw;//0; //
	SDyaw      = parameters.SDyaw;//0; //
	SDpitch    = parameters.SDpitch;//0; //
	SDyawRate  = parameters.SDyawRate;//0; //
	SDpitchRate= parameters.SDpitchRate;//0; //
	
	SDuModel = parameters.SDuModel;
	SDvModel = parameters.SDvModel;
	SDwModel = parameters.SDwModel;
	SDyawRateModel = parameters.SDyawRateModel;
	SDpitchRateModel = parameters.SDpitchRateModel;				
	// initialize state to zeros before recieving the first message
	olibVector<double64> X(SYSTEM_STATE_LEN, 0);
	setXCorrected(X);
	setXPrediction(X);
	// initialize uncertainties
	olibMatrix<double64> P(SYSTEM_STATE_LEN, SYSTEM_STATE_LEN, 0);
		/* this was an error to include
		P[NORTH_INDEX][NORTH_INDEX] = pow(SDnorth,2);   
			P[EAST_INDEX][EAST_INDEX] = pow(SDeast,2); 
		*/
	setPCorrected(P);
	setPPrediction(P);
	olibMatrix<double64> Q(PROCESS_NOISE_LEN, PROCESS_NOISE_LEN, 0);
		Q[SURGE_ACC_INDEX][SURGE_ACC_INDEX] = pow(SDuModel,2);
		Q[SWAY_ACC_INDEX][SWAY_ACC_INDEX] = pow(SDvModel,2);
		Q[HEAVE_ACC_INDEX][HEAVE_ACC_INDEX] = pow(SDwModel,2);
		Q[YAW_ACC_INDEX][YAW_ACC_INDEX] = pow(SDyawRateModel,2); 
		Q[PITCH_ACC_INDEX][PITCH_ACC_INDEX] = pow(SDpitchRateModel,2);
	setQ(Q);
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
{// fixed size = SYSTEM_STATE_LEN x PROCESS_NOISE_LEN
	//should be fixed to 11x5 size
	//north
	m_W[NORTH_INDEX][SURGE_ACC_INDEX] =  1/2*pow(dT,2)*cos(m_xCorrected[YAW_INDEX])*cos(m_xCorrected[PITCH_INDEX]);
	m_W[NORTH_INDEX][SWAY_ACC_INDEX] = -1/2*pow(dT,2)*sin(m_xCorrected[YAW_INDEX])*cos(m_xCorrected[PITCH_INDEX]);
	m_W[NORTH_INDEX][HEAVE_ACC_INDEX] = 0;
	m_W[NORTH_INDEX][YAW_ACC_INDEX] = 0;
	m_W[NORTH_INDEX][PITCH_ACC_INDEX] = 0;
	//east
	m_W[EAST_INDEX][SURGE_ACC_INDEX] =  1/2*pow(dT,2)*sin(m_xCorrected[YAW_INDEX])*cos(m_xCorrected[PITCH_INDEX]);
	m_W[EAST_INDEX][SWAY_ACC_INDEX] =   1/2*pow(dT,2)*cos(m_xCorrected[YAW_INDEX])*cos(m_xCorrected[PITCH_INDEX]);
	m_W[EAST_INDEX][HEAVE_ACC_INDEX] = 0;
	m_W[EAST_INDEX][YAW_ACC_INDEX] = 0;
	m_W[EAST_INDEX][PITCH_ACC_INDEX] = 0;	
	//depth
	m_W[DEPTH_INDEX][SURGE_ACC_INDEX] = 0;
	m_W[DEPTH_INDEX][SWAY_ACC_INDEX] = 0;
	m_W[DEPTH_INDEX][HEAVE_ACC_INDEX] = (1/2)*pow(dT,2)*cos(m_xCorrected[PITCH_INDEX]);
	m_W[DEPTH_INDEX][YAW_ACC_INDEX] = 0;
	m_W[DEPTH_INDEX][PITCH_ACC_INDEX] = 0;	
	//altitude
	m_W[ALTITUDE_INDEX][SURGE_ACC_INDEX] = 0;
	m_W[ALTITUDE_INDEX][SWAY_ACC_INDEX] = 0;
	//m_W[ALTITUDE_INDEX][HEAVE_ACC_INDEX] = -(1/2)*pow(dT,2)*cos(m_xCorrected[PITCH_INDEX]);
	  m_W[ALTITUDE_INDEX][HEAVE_ACC_INDEX] = 0;//-(1/2)*pow(dT,2)*cos(m_xCorrected[PITCH_INDEX]);
	m_W[ALTITUDE_INDEX][YAW_ACC_INDEX] = 0;
	m_W[ALTITUDE_INDEX][PITCH_ACC_INDEX] = 0;	
	//surgeVel
	m_W[SURGE_VEL_INDEX][SURGE_ACC_INDEX] = dT;
	m_W[SURGE_VEL_INDEX][SWAY_ACC_INDEX] = 0;
	m_W[SURGE_VEL_INDEX][HEAVE_ACC_INDEX] = 0;
	m_W[SURGE_VEL_INDEX][YAW_ACC_INDEX] = 0;
	m_W[SURGE_VEL_INDEX][PITCH_ACC_INDEX] = 0;
	//swayVel
	m_W[SWAY_VEL_INDEX][SURGE_ACC_INDEX] = 0;
	m_W[SWAY_VEL_INDEX][SWAY_ACC_INDEX] = dT;
	m_W[SWAY_VEL_INDEX][HEAVE_ACC_INDEX] = 0;
	m_W[SWAY_VEL_INDEX][YAW_ACC_INDEX] = 0;
	m_W[SWAY_VEL_INDEX][PITCH_ACC_INDEX] = 0;
	//heaveVel
	m_W[HEAVE_VEL_INDEX][SURGE_ACC_INDEX] = 0;
	m_W[HEAVE_VEL_INDEX][SWAY_ACC_INDEX] = 0;
	m_W[HEAVE_VEL_INDEX][HEAVE_ACC_INDEX] = dT;
	m_W[HEAVE_VEL_INDEX][YAW_ACC_INDEX] = 0;
	m_W[HEAVE_VEL_INDEX][PITCH_ACC_INDEX] = 0;	
	//yaw
	m_W[YAW_INDEX][SURGE_ACC_INDEX] = 0;
	m_W[YAW_INDEX][SWAY_ACC_INDEX] = 0;
	m_W[YAW_INDEX][HEAVE_ACC_INDEX] = 0;
	m_W[YAW_INDEX][YAW_ACC_INDEX] = (1/2)*pow(dT,2);
	m_W[YAW_INDEX][PITCH_ACC_INDEX] = 0;
	//pitch
	m_W[PITCH_INDEX][SURGE_ACC_INDEX] = 0;
	m_W[PITCH_INDEX][SWAY_ACC_INDEX] = 0;
	m_W[PITCH_INDEX][HEAVE_ACC_INDEX] = 0;
	m_W[PITCH_INDEX][YAW_ACC_INDEX] = 0;
	m_W[PITCH_INDEX][PITCH_ACC_INDEX] = (1/2)*pow(dT,2);
	//yawRate
	m_W[YAW_RATE_INDEX][SURGE_ACC_INDEX] = 0;
	m_W[YAW_RATE_INDEX][SWAY_ACC_INDEX] = 0;
	m_W[YAW_RATE_INDEX][HEAVE_ACC_INDEX] = 0;
	m_W[YAW_RATE_INDEX][YAW_ACC_INDEX] = dT;
	m_W[YAW_RATE_INDEX][PITCH_ACC_INDEX] = 0;	
	//pitchRate
	m_W[PITCH_RATE_INDEX][SURGE_ACC_INDEX] = 0;
	m_W[PITCH_RATE_INDEX][SWAY_ACC_INDEX] = 0;
	m_W[PITCH_RATE_INDEX][HEAVE_ACC_INDEX] = 0;
	m_W[PITCH_RATE_INDEX][YAW_ACC_INDEX] = 0;
	m_W[PITCH_RATE_INDEX][PITCH_ACC_INDEX] = dT;
}

void extendedKalmanFilter5DOF::setF (void){
	//fixed size 11x11
	//north
	m_F[NORTH_INDEX][NORTH_INDEX] = 1;
	m_F[NORTH_INDEX][EAST_INDEX] = 0;
	m_F[NORTH_INDEX][DEPTH_INDEX] = 0;
	m_F[NORTH_INDEX][ALTITUDE_INDEX] = 0;
	m_F[NORTH_INDEX][SURGE_VEL_INDEX] =  dT*cos(m_xCorrected[YAW_INDEX])*cos(m_xCorrected[PITCH_INDEX]);
	m_F[NORTH_INDEX][SWAY_VEL_INDEX] = -dT*sin(m_xCorrected[YAW_INDEX])*cos(m_xCorrected[PITCH_INDEX]);
	m_F[NORTH_INDEX][HEAVE_VEL_INDEX] = 0;
	m_F[NORTH_INDEX][YAW_INDEX] = -(m_xCorrected[SURGE_VEL_INDEX]*dT)*sin(m_xCorrected[YAW_INDEX])*cos(m_xCorrected[PITCH_INDEX])-(m_xCorrected[SWAY_VEL_INDEX]*dT)*cos(m_xCorrected[YAW_INDEX])*cos(m_xCorrected[PITCH_INDEX]);	
	m_F[NORTH_INDEX][PITCH_INDEX] = -(m_xCorrected[SURGE_VEL_INDEX]*dT)*cos(m_xCorrected[YAW_INDEX])*sin(m_xCorrected[PITCH_INDEX])+(m_xCorrected[SWAY_VEL_INDEX]*dT)*sin(m_xCorrected[YAW_INDEX])*sin(m_xCorrected[PITCH_INDEX]);
	m_F[NORTH_INDEX][YAW_RATE_INDEX] =  0;
	m_F[NORTH_INDEX][PITCH_RATE_INDEX] = 0;
	
	
	//east
	m_F[EAST_INDEX][NORTH_INDEX] = 0;
	m_F[EAST_INDEX][EAST_INDEX] = 1;
	m_F[EAST_INDEX][DEPTH_INDEX] = 0;
	m_F[EAST_INDEX][ALTITUDE_INDEX] = 0;
	m_F[EAST_INDEX][SURGE_VEL_INDEX] =  dT*sin(m_xCorrected[YAW_INDEX])*cos(m_xCorrected[PITCH_INDEX]);
	m_F[EAST_INDEX][SWAY_VEL_INDEX]  = dT*cos(m_xCorrected[YAW_INDEX])*cos(m_xCorrected[PITCH_INDEX]);
	m_F[EAST_INDEX][HEAVE_VEL_INDEX] = 0;
	m_F[EAST_INDEX][YAW_INDEX] = (m_xCorrected[SURGE_VEL_INDEX]*dT)*cos(m_xCorrected[YAW_INDEX])*cos(m_xCorrected[PITCH_INDEX])-(m_xCorrected[SWAY_VEL_INDEX]*dT)*sin(m_xCorrected[YAW_INDEX])*cos(m_xCorrected[PITCH_INDEX]);	
	m_F[EAST_INDEX][PITCH_INDEX] = -(m_xCorrected[SURGE_VEL_INDEX]*dT)*sin(m_xCorrected[YAW_INDEX])*sin(m_xCorrected[PITCH_INDEX])-(m_xCorrected[SWAY_VEL_INDEX]*dT)*cos(m_xCorrected[YAW_INDEX])*sin(m_xCorrected[PITCH_INDEX]);
	m_F[EAST_INDEX][YAW_RATE_INDEX] =  0;
	m_F[EAST_INDEX][PITCH_RATE_INDEX] = 0;	
	
	//depth
	m_F[DEPTH_INDEX][NORTH_INDEX] = 0;
	m_F[DEPTH_INDEX][EAST_INDEX] = 0;
	m_F[DEPTH_INDEX][DEPTH_INDEX] = 1;
	m_F[DEPTH_INDEX][ALTITUDE_INDEX] = 0;
	m_F[DEPTH_INDEX][SURGE_VEL_INDEX] = 0;
	m_F[DEPTH_INDEX][SWAY_VEL_INDEX] = 0;
	m_F[DEPTH_INDEX][HEAVE_VEL_INDEX] = dT*cos(m_xCorrected[PITCH_INDEX]);
	m_F[DEPTH_INDEX][YAW_INDEX] = 0;	
	m_F[DEPTH_INDEX][PITCH_INDEX] = -(m_xCorrected[HEAVE_VEL_INDEX]*dT)*sin(m_xCorrected[PITCH_INDEX]);
	m_F[DEPTH_INDEX][YAW_RATE_INDEX] = 0;
	m_F[DEPTH_INDEX][PITCH_RATE_INDEX]= 0;
	
	//altitude
	m_F[ALTITUDE_INDEX][NORTH_INDEX] = 0;
	m_F[ALTITUDE_INDEX][EAST_INDEX] = 0;
	m_F[ALTITUDE_INDEX][DEPTH_INDEX] = 0;
	m_F[ALTITUDE_INDEX][ALTITUDE_INDEX] = 1;
	m_F[ALTITUDE_INDEX][SURGE_VEL_INDEX] = 0;
	m_F[ALTITUDE_INDEX][SWAY_VEL_INDEX] = 0;
	//m_F[ALTITUDE_INDEX][HEAVE_VEL_INDEX] = -dT*cos(m_xCorrected[PITCH_INDEX]);
	  m_F[ALTITUDE_INDEX][HEAVE_VEL_INDEX] = 0;//-dT*cos(m_xCorrected[PITCH_INDEX]);
	m_F[ALTITUDE_INDEX][YAW_INDEX] = 0;	
	m_F[ALTITUDE_INDEX][PITCH_INDEX] = (m_xCorrected[HEAVE_VEL_INDEX]*dT)*sin(m_xCorrected[PITCH_INDEX]);
	m_F[ALTITUDE_INDEX][YAW_RATE_INDEX] = 0;
	m_F[ALTITUDE_INDEX][PITCH_RATE_INDEX]= 0;
	
	//surgeVel
	m_F[SURGE_VEL_INDEX][NORTH_INDEX] = 0;
	m_F[SURGE_VEL_INDEX][EAST_INDEX] = 0;
	m_F[SURGE_VEL_INDEX][DEPTH_INDEX] = 0;
	m_F[SURGE_VEL_INDEX][ALTITUDE_INDEX] = 0;
	m_F[SURGE_VEL_INDEX][SURGE_VEL_INDEX] = 1;
	m_F[SURGE_VEL_INDEX][SWAY_VEL_INDEX] = 0;
	m_F[SURGE_VEL_INDEX][HEAVE_VEL_INDEX] = 0;
	m_F[SURGE_VEL_INDEX][YAW_INDEX] = 0;	
	m_F[SURGE_VEL_INDEX][PITCH_INDEX] = 0;
	m_F[SURGE_VEL_INDEX][YAW_RATE_INDEX] = 0;
	m_F[SURGE_VEL_INDEX][PITCH_RATE_INDEX]= 0;
	
	//swayVel
	m_F[SWAY_VEL_INDEX][NORTH_INDEX] = 0;
	m_F[SWAY_VEL_INDEX][EAST_INDEX] = 0;
	m_F[SWAY_VEL_INDEX][DEPTH_INDEX] = 0;
	m_F[SWAY_VEL_INDEX][ALTITUDE_INDEX] = 0;
	m_F[SWAY_VEL_INDEX][SURGE_VEL_INDEX] = 0;
	m_F[SWAY_VEL_INDEX][SWAY_VEL_INDEX] = 1;
	m_F[SWAY_VEL_INDEX][HEAVE_VEL_INDEX] = 0;
	m_F[SWAY_VEL_INDEX][YAW_INDEX] = 0;	
	m_F[SWAY_VEL_INDEX][PITCH_INDEX] = 0;
	m_F[SWAY_VEL_INDEX][YAW_RATE_INDEX] = 0;
	m_F[SWAY_VEL_INDEX][PITCH_RATE_INDEX]= 0;
	
	//heaveVel
	m_F[HEAVE_VEL_INDEX][NORTH_INDEX] = 0;
	m_F[HEAVE_VEL_INDEX][EAST_INDEX] = 0;
	m_F[HEAVE_VEL_INDEX][DEPTH_INDEX] = 0;
	m_F[HEAVE_VEL_INDEX][ALTITUDE_INDEX] = 0;
	m_F[HEAVE_VEL_INDEX][SURGE_VEL_INDEX] = 0;
	m_F[HEAVE_VEL_INDEX][SWAY_VEL_INDEX] = 0;
	m_F[HEAVE_VEL_INDEX][HEAVE_VEL_INDEX] = 1;
	m_F[HEAVE_VEL_INDEX][YAW_INDEX] = 0;	
	m_F[HEAVE_VEL_INDEX][PITCH_INDEX] = 0;
	m_F[HEAVE_VEL_INDEX][YAW_RATE_INDEX] = 0;
	m_F[HEAVE_VEL_INDEX][PITCH_RATE_INDEX]= 0;						

	//yaw
	m_F[YAW_INDEX][NORTH_INDEX] = 0;
	m_F[YAW_INDEX][EAST_INDEX] = 0;
	m_F[YAW_INDEX][DEPTH_INDEX] = 0;
	m_F[YAW_INDEX][ALTITUDE_INDEX] = 0;
	m_F[YAW_INDEX][SURGE_VEL_INDEX] = 0;
	m_F[YAW_INDEX][SWAY_VEL_INDEX] = 0;
	m_F[YAW_INDEX][HEAVE_VEL_INDEX] = 0;
	m_F[YAW_INDEX][YAW_INDEX] = 1;	
	m_F[YAW_INDEX][PITCH_INDEX] = 0;
	m_F[YAW_INDEX][YAW_RATE_INDEX] = dT;
	m_F[YAW_INDEX][PITCH_RATE_INDEX]= 0;
	
	//pitch
	m_F[PITCH_INDEX][NORTH_INDEX] = 0;
	m_F[PITCH_INDEX][EAST_INDEX] = 0;
	m_F[PITCH_INDEX][DEPTH_INDEX] = 0;
	m_F[PITCH_INDEX][ALTITUDE_INDEX] = 0;
	m_F[PITCH_INDEX][SURGE_VEL_INDEX] = 0;
	m_F[PITCH_INDEX][SWAY_VEL_INDEX] = 0;
	m_F[PITCH_INDEX][HEAVE_VEL_INDEX] = 0;
	m_F[PITCH_INDEX][YAW_INDEX] = 0;	
	m_F[PITCH_INDEX][PITCH_INDEX] = 1;
	m_F[PITCH_INDEX][YAW_RATE_INDEX] = 0;
	m_F[PITCH_INDEX][PITCH_RATE_INDEX]= dT;	
	
	//yawRate
	m_F[YAW_RATE_INDEX][NORTH_INDEX] = 0;
	m_F[YAW_RATE_INDEX][EAST_INDEX] = 0;
	m_F[YAW_RATE_INDEX][DEPTH_INDEX] = 0;
	m_F[YAW_RATE_INDEX][ALTITUDE_INDEX] = 0;
	m_F[YAW_RATE_INDEX][SURGE_VEL_INDEX] = 0;
	m_F[YAW_RATE_INDEX][SWAY_VEL_INDEX] = 0;
	m_F[YAW_RATE_INDEX][HEAVE_VEL_INDEX] = 0;
	m_F[YAW_RATE_INDEX][YAW_INDEX] = 0;	
	m_F[YAW_RATE_INDEX][PITCH_INDEX] = 0;
	m_F[YAW_RATE_INDEX][YAW_RATE_INDEX] = 1;
	m_F[YAW_RATE_INDEX][PITCH_RATE_INDEX]= 0;	
	
	//pitchRate
	m_F[PITCH_RATE_INDEX][NORTH_INDEX] = 0;
	m_F[PITCH_RATE_INDEX][EAST_INDEX] = 0;
	m_F[PITCH_RATE_INDEX][DEPTH_INDEX] = 0;
	m_F[PITCH_RATE_INDEX][ALTITUDE_INDEX] = 0;
	m_F[PITCH_RATE_INDEX][SURGE_VEL_INDEX] = 0;
	m_F[PITCH_RATE_INDEX][SWAY_VEL_INDEX] = 0;
	m_F[PITCH_RATE_INDEX][HEAVE_VEL_INDEX] = 0;
	m_F[PITCH_RATE_INDEX][YAW_INDEX] = 0;	
	m_F[PITCH_RATE_INDEX][PITCH_INDEX] = 0;
	m_F[PITCH_RATE_INDEX][YAW_RATE_INDEX] = 0;
	m_F[PITCH_RATE_INDEX][PITCH_RATE_INDEX]= 1;	
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
	setElapsedPeriod(dT_interval);
	// prediction happens first even if there is no kalman filtering update
	setW(); setF(); Predict(); // extendedKalmanFilter5DOF methods
		
		olibVector<double64> outputFiltered(SYSTEM_STATE_LEN,0);
                //prepare the output
                outputFiltered = this->getXPrediction();
		// allocate H and R matrices for update stage	
		uint32 lengthObservation; 
		lengthObservation = statesInObservation.size(); // length of the observation vector
	
		olibMatrix<double64> H(lengthObservation, SYSTEM_STATE_LEN);// initialize with zeros
		olibMatrix<double64> R(lengthObservation, lengthObservation);
		bool thereWasAnObservation = false;
		for(uint32 i=0; i<lengthObservation ; i++)
		{
			switch(statesInObservation[i])
			{
				case NULL_STATE:
				{
					//nothing
					break;
				}	
				case NORTH:
				{
					H[i][NORTH_INDEX] = 1; R[i][i] = pow(SDnorth,2);
					thereWasAnObservation = true;
					break;
				}
				case EAST:
				{
					H[i][EAST_INDEX] = 1;  R[i][i] = pow(SDeast,2);
					thereWasAnObservation = true;
					break;
				}
				case DEPTH:
				{
					H[i][DEPTH_INDEX] = 1; R[i][i] = pow(SDdepth,2);
					thereWasAnObservation = true;
					break;
				}
				case ALTITUDE:
				{
					H[i][ALTITUDE_INDEX] = 1; R[i][i] = pow(SDaltitude,2);
					thereWasAnObservation = true;
					break;
				}
				case SURGE_VELOCITY:
				{
					H[i][SURGE_VEL_INDEX] = 1; R[i][i] = pow(SDu,2);
					thereWasAnObservation = true;
					break;
				}
				case SWAY_VELOCITY:
				{
					H[i][SWAY_VEL_INDEX] = 1; R[i][i] = pow(SDv,2);
					thereWasAnObservation = true;
					break;
				}	
				case HEAVE_VELOCITY:
				{
					H[i][HEAVE_VEL_INDEX] = 1; R[i][i] = pow(SDw,2);
					thereWasAnObservation = true;
					break;
				}
				case YAW:
				{
					H[i][YAW_INDEX] = 1; R[i][i] = pow(SDyaw,2);
					thereWasAnObservation = true;
					break;
				}
				case PITCH:
				{
					H[i][PITCH_INDEX] = 1; R[i][i] = pow(SDpitch,2);
					thereWasAnObservation = true;
					break;
				}
				case YAW_RATE:
				{
					H[i][YAW_RATE_INDEX] = 1; R[i][i] = pow(SDyawRate,2);
					thereWasAnObservation = true;
					break;
				}
				case PITCH_RATE:
				{
					H[i][PITCH_RATE_INDEX] = 1; R[i][i] = pow(SDpitchRate,2);
					thereWasAnObservation = true;
					break;
				}																															
			}//switch(statesInObservation[i])
		}//for loop
		//cout << "H:  " << H << endl;
		//cout << "R:  " << R << endl;
		// do update  
		if (thereWasAnObservation) {
			// temporary measurement object attached with current class
			olibKalmanMeasure measurementEKF5DOF(lengthObservation, this);
			measurementEKF5DOF.setz(observation);
			//cout << "observation: " << observation << endl; 
			measurementEKF5DOF.setR(R);
			measurementEKF5DOF.setJ(H);
			// predicted observation
			olibVector<double64> Hx(lengthObservation,0);
			Hx = H * this->getXPrediction();
			//cout << "predicted observation:  " << Hx << endl;
			measurementEKF5DOF.seth(Hx);
			this->Correct(measurementEKF5DOF);
			outputFiltered = this->getXCorrected();
		}	
		return outputFiltered;
}
// handle observation after each message
olibVector<double64> extendedKalmanFilter5DOF::filterMessage(olibVector<double64> measurement, sensorType whichSensor, double64 dT_interval)
{
// if the NULL_SENSOR is at the input, function does only prediction within filtering
// expected measurement indexing (input has to be consistent):
// DEPTH_SENSOR: measurement[0]= depth, measurement[1] - heaveVelocity
// COMPASS_SENSOR: [0] - pitch [1]- pitchRate [2]- yaw
// GYRO_SENSOR: [0] - yawRate
// DVL_SENSOR: [0] - altitude [1] - surgeVelocity [2] - swayVelocity
// LBL_SENSOR: [0] - north [1] - east
	//updateTime(current_time);
	setElapsedPeriod(dT_interval);
	// prediction happens first even if there is no kalman filtering enabled
	setW(); setF(); Predict(); // extendedKalmanFilter5DOF methods
		olibVector<double64> outputFiltered(SYSTEM_STATE_LEN,0);
                //prepare the output
                outputFiltered = this->getXPrediction();
		// allocate H and R matrices for observation stage
		uint32 lengthMeasurement;
		lengthMeasurement = measurement.size(); // length of the measurement vector
		olibMatrix<double64> H(lengthMeasurement, SYSTEM_STATE_LEN);
		olibMatrix<double64> R(lengthMeasurement, lengthMeasurement);
		bool thereWasAMessage = false;
		switch(whichSensor)
		{
			case NULL_SENSOR:
			{
				// nothing
				break;
			}	
			case DEPTH_SENSOR:
			{// depth sensor: depth and heave velocity update, takes two measurements
				//measurement[0] is considered as depth //depth taken from navigation class
				//measurement[1] is considered as heaveVelocity //heave Vel taken member of navigation class
				//olibMatrix<double64> H(lengthMeasurement, SYSTEM_STATE_LEN);
				H[0][DEPTH_INDEX]     = 1;
				H[1][HEAVE_VEL_INDEX] = 1;
				//olibMatrix<double64> R(lengthMeasurement, lengthMeasurement);
				R[0][0] = pow(SDdepth,2);
				R[1][1] = pow(SDw,2);
				thereWasAMessage = true;
				break;
			}
			case COMPASS_SENSOR:
			{// compass: pitch, pitchRate, yaw ( roll is not needed since it's 5DOF)
				//measurement[0] is considered as pitch
				//measurement[1] is considered as pitchRate
				//measurement[2] is considered as yaw // infamous yaw from compass
				//olibMatrix<double64> H(lengthMeasurement, SYSTEM_STATE_LEN);  
				H[0][PITCH_INDEX]      = 1; 
				H[1][PITCH_RATE_INDEX] = 1;
				H[2][YAW_INDEX]        = 1;
				//olibMatrix<double64> R(lengthMeasurement, lengthMeasurement);			
				R[0][0] = pow(SDpitch,2);
				R[1][1] = pow(SDpitchRate,2);
				R[2][2] = pow(SDyaw,2);
				thereWasAMessage = true;
				break;
			}
			case GYRO_SENSOR:
			{//gyro:  yaw rate
				//measurement[0] is considered as yawRate
				//olibMatrix<double64> H(lengthMeasurement, SYSTEM_STATE_LEN);
				H[0][YAW_RATE_INDEX] = 1;
				//olibMatrix<double64> R(lengthMeasurement, lengthMeasurement);
				R[0][0] = pow(SDyawRate,2);
				thereWasAMessage = true;
				break;
			}
			case DVL_SENSOR:
			{// altitude, surge and sway velocity
				//measurement[0] is considered as altitude
				//measurement[1] is considered as surgeVelocity
				//measurement[2] is considered as swayVelocity
				//olibMatrix<double64> H(lengthMeasurement, SYSTEM_STATE_LEN);
				H[0][ALTITUDE_INDEX]  = 1;
				H[1][SURGE_VEL_INDEX] = 1;
				H[2][SWAY_VEL_INDEX]  = 1;
				//olibMatrix<double64> R(lengthMeasurement, lengthMeasurement);
				R[0][0] = pow(SDaltitude,2);
				R[1][1] = pow(SDu,2);
				R[2][2] = pow(SDv,2);
				thereWasAMessage = true;
				break;
			}
			case LBL_SENSOR:
			{// lbl updates north, east
				//measurement[0] is considered as north
				//measurement[1] is considered as east
				//olibMatrix<double64> H(lengthMeasurement, SYSTEM_STATE_LEN);
				H[0][NORTH_INDEX] = 1;
				H[1][EAST_INDEX]  = 1;
				//olibMatrix<double64> R(lengthMeasurement, lengthMeasurement);
				R[0][0] = pow(SDnorth,2);
				R[1][1] = pow(SDeast,2);
				thereWasAMessage = true;
				break;
			}
			case GPS_SENSOR:
			{
				//nothing
				break;
			}
		}//switch(whichSensor)
		// do update
		if(thereWasAMessage) {
		
			// temporary measurement object attached with current class
			olibKalmanMeasure measurementEKF5DOF(lengthMeasurement, this);
			measurementEKF5DOF.setR(R);
			measurementEKF5DOF.setJ(H);
			measurementEKF5DOF.setz(measurement);
			// predicted observation
			olibVector<double64> Hx(lengthMeasurement, 0) ;
			Hx = H * this->getXPrediction();
			measurementEKF5DOF.seth(Hx);
			this->Correct(measurementEKF5DOF);
			outputFiltered = this->getXCorrected();
		}			
		return outputFiltered;
}
//////////////// UNSCENTED DEFINITIONS ///////////////////////////
/*
void unscentedTransformSampling(olibVector<double64> & x_mean, olibMatrix<double64> & P, float k, 
                                olibMatrix<double64> & x_samples, olibVector<double64> & x_weights
                               )
{
// calculation of sigma points  - collection of samples neeeded for unscented Kalman filtering
// calculated using mean value (x_mean) and covariance (P)
// outputs are samples (x_samples) and weights for each sample (x_weights)
// http://cslu.cse.ogi.edu/nsel/ukf/node6.html
// 
	//length of the input vector
	uint32 x_len, P_rows, P_cols;
	x_len = x_mean.size();
	P_rows = P.numberRows();
	P_cols = P.numberColumns();
	if(P_cols!=P_rows){
		printf ("Covariance matrix is not rectangular.");
    		exit (1);
    	}
	if(x_len!=P_rows){
		printf ("Covariance matrix size does not match sample size.");
    		exit (1);
    	}
	//define samples matrix size: one row is one sample, size matrix=(2*x_len+1) X x_len 
	x_samples.setSize(2*x_len+1, x_len, 0);
	// define vector with weights
	x_weights.setSize(2*x_len+1);
	// first sample is mean, it is given as input and stored in first row of the samples matrix
	x_samples[0] = x_mean;
	//olibMatrix<double64> Coeff(1,1,0); Coeff[0][0]=x_len+k;
	double64 Coeff; Coeff = x_len+k;
	P = Coeff * P; //square root of P needs to be calculated
	
		// MATRIX SQUARE ROOT CALCULATION ///////////
		// dummy serves as standars C storage to store and be used to 
		// transfer to format used by TooN library - to calculate square root
		// temporary solution since root does not exist in library
		//Matrix<3> dummy;//(x_len,x_len)
		//Matrix<3> dummyRoot;//(x_len,x_len)
		static const int N = x_len;
		double64 dummy[N][N];
		
		for(uint32 row=0; row<x_len; row++){
			for(uint32 col=0; col<x_len; col++){
				dummy[row][col]=P[row][col];
			}
		}
		cout << "dummy(0,0): \n" << dummy[0][0] << endl;
		cout << "dummy(2,2): \n" << dummy[2][2] << endl;
		//Cholesky<3> chol_dummy(dummy);
		//dummyRoot = chol_dummy.backsub();
		//cout << "dummy root: \n" << dummyRoot << endl;
		/////////////////////////////////////////////
	for (uint32 i=1; i<=x_len; i++){
		x_samples[i] = x_mean;
	}
	for (uint32 i=x_len+1; i<=2*x_len; i++){
		x_samples[i] = x_mean;
	}
		
}
*/


