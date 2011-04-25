#include <algorithm>
#include <osl_core/GeneralMath.h>
#include "nav_ros.h"

		extern FILE *pInputMessages;
		extern FILE *pOutputMessages;
		extern FILE *pPredictedState;
		extern FILE *pCorrectedState;
		extern FILE *pPredictedP;
		extern FILE *pCorrectedP;

using namespace std;

navigationRos::navigationRos(){

}

navigationRos::~navigationRos(){
}

bool navigationRos::init(int argc, char * argv[]) {
	double64 dT_init = 0.0;
	EKF5DOF.setElapsedPeriod(dT_init);
	EKF5DOF.init();

	firstMsgReceived = false;
	LBLjustHappened =false;

	pitch = yaw = 0.0;
	pitchRate = yawRate = 0.0;
	surgeVelocity = swayVelocity = heaveVelocity = 0.0;
	north = east = altitude = depth = 0.0;

	m_timer.setTimer(1);  

	ros::TransportHints hints = ros::TransportHints().udp().tcpNoDelay();
	m_rosInputNavSub = m_rosHandle->subscribe<auv_msgs::NavSts>("nav/nav_sts",
 			1, &navigationRos::handleROSnavMsg, this, hints);
	m_rosInputLblNavSub = m_rosHandle->subscribe<auv_msgs::NavSts>("nav/lbl_nav_sts",
 			1, &navigationRos::handleROSlblMsg, this, hints);
	m_rosFiltNavPub = m_rosHandle->advertise<auv_msgs::NavSts>("nav/filt_nav_sts", 10);
	return true;
} 

void navigationRos::handleROSnavMsg(const auv_msgs::NavStsConstPtr & msg)
{
		double64 dT = (double64)msg->header.stamp.sec+((double64)msg->header.stamp.nsec*1e-9) -
	   (double64)m_rosInputNavMsg.header.stamp.sec-((double64)m_rosInputNavMsg.header.stamp.nsec*1e-9);

		// how to handle ROS message - store it in temporary buffers
		m_rosInputNavMsg.header.stamp = msg->header.stamp;
    	m_rosInputNavMsg.position.north = msg->position.north;
    	m_rosInputNavMsg.position.east  = msg->position.east;
    	m_rosInputNavMsg.position.depth = msg->position.depth;
    	m_rosInputNavMsg.altitude       = msg->altitude;
        m_rosInputNavMsg.orientation.yaw = msg->orientation.yaw;
        m_rosInputNavMsg.orientation.pitch = msg->orientation.pitch;
    	m_rosInputNavMsg.body_velocity.x = msg->body_velocity.x;
    	m_rosInputNavMsg.body_velocity.y = msg->body_velocity.y;
    	m_rosInputNavMsg.body_velocity.z = msg->body_velocity.z;
    	m_rosInputNavMsg.orientation_rate.yaw   = msg->orientation_rate.yaw;
    	m_rosInputNavMsg.orientation_rate.pitch = msg->orientation_rate.pitch; 

    	if(EKF5DOF.firstMsgRecieved)
    	{
    		EKF5DOF.setElapsedPeriod(dT);
    		if (LBLjustHappened)
    		{
    			whoWasIt = 2;
        		EKF5DOF.useLblSensor = true;
        		EKF5DOF.usePressSensor = EKF5DOF.useDvlSensor = EKF5DOF.useCompassSensor = EKF5DOF.useGyroSensor = false;
        		LBLjustHappened = false;
    		}
    		else
    		{
    			whoWasIt = 1;
        		EKF5DOF.useLblSensor = false;
        		EKF5DOF.usePressSensor = EKF5DOF.useDvlSensor = EKF5DOF.useCompassSensor = EKF5DOF.useGyroSensor = true;

    		}


    		cout << "NAV" << endl;
    		filtering(msg, EKF5DOF); // the output is updated Kalman class instance
    	}
    	else
    	{
    		// it is a first message
    		// dT stays zero, initial value no filtering is being done
    		double64 dT_initial = 0;
    		EKF5DOF.setElapsedPeriod(dT_initial);
				olibVector<double64> X(11, 0);
    			X[0] = m_rosInputNavMsg.position.north;
    			X[1] = m_rosInputNavMsg.position.east;
    			X[2] = m_rosInputNavMsg.position.depth;
    			X[3] = m_rosInputNavMsg.altitude;
    			X[4] = m_rosInputNavMsg.body_velocity.x;
    			X[5] = m_rosInputNavMsg.body_velocity.y;
    			X[6] = m_rosInputNavMsg.body_velocity.z;
    			X[7] = m_rosInputNavMsg.orientation.yaw;
    			X[8] = m_rosInputNavMsg.orientation.pitch;
    			X[9] = m_rosInputNavMsg.orientation_rate.yaw;
    			X[10] = m_rosInputNavMsg.orientation_rate.pitch;
    		EKF5DOF.setXPrediction(X);
    		EKF5DOF.setXCorrected(X);
    		EKF5DOF.firstMsgRecieved = true;

    	}
} // Nav Msg

void navigationRos::handleROSlblMsg(const auv_msgs::NavStsConstPtr & msg)
{
	LBLjustHappened = true;

		double64 dT = (double64)msg->header.stamp.sec+((double64)msg->header.stamp.nsec*1e-9) -
				(double64)m_rosInputNavMsg.header.stamp.sec-((double64)m_rosInputNavMsg.header.stamp.nsec*1e-9);

		// how to handle LBL message- store it in temporary buffers
		m_rosInputNavMsg.header.stamp = msg->header.stamp;
    	m_rosInputNavMsg.position.north = msg->position.north;
    	m_rosInputNavMsg.position.east  = msg->position.east;
    	m_rosInputNavMsg.position.depth = msg->position.depth;
    	m_rosInputNavMsg.altitude       = msg->altitude;
        m_rosInputNavMsg.orientation.yaw = msg->orientation.yaw;
        m_rosInputNavMsg.orientation.pitch = msg->orientation.pitch;
    	m_rosInputNavMsg.body_velocity.x = msg->body_velocity.x;
    	m_rosInputNavMsg.body_velocity.y = msg->body_velocity.y;
    	m_rosInputNavMsg.body_velocity.z = msg->body_velocity.z;
    	m_rosInputNavMsg.orientation_rate.yaw   = msg->orientation_rate.yaw;
    	m_rosInputNavMsg.orientation_rate.pitch = msg->orientation_rate.pitch;

    	if(EKF5DOF.firstMsgRecieved)
    	{
    		EKF5DOF.setElapsedPeriod(dT);
    		EKF5DOF.useLblSensor = false;
    		EKF5DOF.usePressSensor = EKF5DOF.useDvlSensor = EKF5DOF.useCompassSensor = EKF5DOF.useGyroSensor = true;
    		whoWasIt = 1;
    		//cout << "LBL mereno: " << msg->position.north << " and " << msg->position.east << endl;
    		filtering(msg, EKF5DOF); // the output is updated Kalman class instance

    	}
    	else
    	{
    		// it is a first message
    		// dT stays zero, initial value no filtering is being done
    		double64 dT_initial = 0;
    		EKF5DOF.setElapsedPeriod(dT_initial);
				olibVector<double64> X(11, 0);
    			X[0] = m_rosInputNavMsg.position.north;
    			X[1] = m_rosInputNavMsg.position.east;
    			X[2] = m_rosInputNavMsg.position.depth;
    			X[3] = m_rosInputNavMsg.altitude;
    			X[4] = m_rosInputNavMsg.body_velocity.x;
    			X[5] = m_rosInputNavMsg.body_velocity.y;
    			X[6] = m_rosInputNavMsg.body_velocity.z;
    			X[7] = m_rosInputNavMsg.orientation.yaw;
    			X[8] = m_rosInputNavMsg.orientation.pitch;
    			X[9] = m_rosInputNavMsg.orientation_rate.yaw;
    			X[10] = m_rosInputNavMsg.orientation_rate.pitch;
    		EKF5DOF.setXPrediction(X);
    		EKF5DOF.setXCorrected(X);
    		EKF5DOF.firstMsgRecieved = true;

    	}
} // LBL Msg

// ---------------------------------------------------------------------------
void navigationRos::doWork()
{
	if (m_timer.getTimer() == 0)
	{
		m_timer.setTimer(2000);
		// Wrap angles
		//roll = osl_core::wrapPlusMinus90(roll);
		pitch = osl_core::wrapPlusMinus90(pitch);
		yaw   = osl_core::wrapPlusMinus180(yaw);

		/*
		cout <<  
		"#------ ### EXTENDED KALMAN TRIAL ### ------"  << "\n" <<
		//"# time     =  " << (float)m_rosFiltNavMsg.header.stamp.sec+(float)m_rosFiltNavMsg.header.stamp.nsec*1e-9  << "\n" <<
		"##                 FILTERED     |        ORIGINAL               |     DIFF      "                                << "\n" <<
		"##  N        = " <<  north   << "   |  " << m_rosInputNavMsg.position.north << "  |  " <<  fabs(north-m_rosInputNavMsg.position.north)  << "\n" <<
		"##  E        = " <<  east    << "   |  " << m_rosInputNavMsg.position.east  <<  fabs(east-m_rosInputNavMsg.position.east)    << "\n" <<
		"##  Depth    = " <<  depth   << "   |  " << m_rosFiltNavMsg.position.depth  <<  fabs(depth-m_rosFiltNavMsg.position.depth)   << "\n" <<
		"## ***************************" << "\n" << endl;

		"##  altitude = " <<  altitude << "\n" <<
		"##-----------------------------------------------" << "\n" <<
		"##  YAW      = " <<  yaw << "\n" <<
		"##  PITCH    = " <<  pitch << "\n" <<
		"##-----------------------------------------------" << "\n" <<
		"##  SurgeVel = " << surgeVelocity << "\n" <<
		"##  SwayVel  = " << swayVelocity << "\n" <<
		"##  HeaveVel = " << heaveVelocity << "\n" <<
		"##  YawRate  = " << yawRate << "\n" <<
		"##  PitchRate  = " << pitchRate << "\n" <<
		endl;
		*/
		sendROSNavSts();
	} 
}

//---------------------------------------------------------------------------
void navigationRos::sendROSNavSts() {
	// input: m_rosInputNavMsg.header.stamp -> output m_rosFiltNavMsg
	m_rosFiltNavMsg.header.stamp = ros::Time::now();
	// take the values stored during filtering() stage
	m_rosFiltNavMsg.position.north = north;
	m_rosFiltNavMsg.position.east  = east;
	m_rosFiltNavMsg.position.depth = depth;
	m_rosFiltNavMsg.altitude       = altitude;

	m_rosFiltNavMsg.orientation.pitch = pitch;
	m_rosFiltNavMsg.orientation.yaw   = yaw;

	m_rosFiltNavMsg.body_velocity.x = surgeVelocity;
	m_rosFiltNavMsg.body_velocity.y = swayVelocity;
	m_rosFiltNavMsg.body_velocity.z = heaveVelocity;

	m_rosFiltNavMsg.orientation_rate.pitch = pitchRate;
	m_rosFiltNavMsg.orientation_rate.yaw   = yawRate;

	//m_rosNavMsg.mode = auv_msgs::NavSts::MODE_DEFAULT;
	m_rosFiltNavMsg.status = auv_msgs::NavSts::STATUS_ALL_OK;

	m_rosFiltNavPub.publish(m_rosInputNavMsg); //Filt
}

void navigationRos::filtering (const auv_msgs::NavStsConstPtr & msg, extendedKalmanFilter5DOF & kalman_filter)
{

	//does prediction whether there was observation or not (not sure if that's good)
	kalman_filter.setW(); kalman_filter.setF(); kalman_filter.Predict();

	// allocate, Z, H and R for observation stage
		olibVector<double64> Z_pattern(1, 0);
		olibMatrix<double64> H_pattern(1, 11);
		olibMatrix<double64> R_pattern(1, 1);
	uint32 lengthMeasurement = 0;
	if(!(kalman_filter.usePressSensor || kalman_filter.useDvlSensor || kalman_filter.useCompassSensor
			|| kalman_filter.useGyroSensor || kalman_filter.useLblSensor)
	    && !kalman_filter.enableFiltering)
	{
		cout << "None of the sensors is set to be active or filtering in general is not enabled!" << endl;
	}
	else // does observation
	{
		//***
		if(kalman_filter.usePressSensor){
		lengthMeasurement ++;
		// include depth
		Z_pattern.setSize(lengthMeasurement); Z_pattern[lengthMeasurement-1] = msg->position.depth;
		H_pattern.setSize(lengthMeasurement, SYSTEM_STATE_LEN, 0); H_pattern[lengthMeasurement-1][2] = 1;
		R_pattern.setSize(lengthMeasurement, lengthMeasurement, 0); R_pattern[lengthMeasurement-1][lengthMeasurement-1] = pow(kalman_filter.SDdepth,2);
		lengthMeasurement ++;
		// include heaveVel
		Z_pattern.setSize(lengthMeasurement); Z_pattern[lengthMeasurement-1] = msg->body_velocity.z;
		H_pattern.setSize(lengthMeasurement, SYSTEM_STATE_LEN, 0); H_pattern[lengthMeasurement-1][6] = 1;
		R_pattern.setSize(lengthMeasurement, lengthMeasurement, 0); R_pattern[lengthMeasurement-1][lengthMeasurement-1] = pow(kalman_filter.SDw,2);
		}
		//***
		if(kalman_filter.useDvlSensor){
		lengthMeasurement ++;
		// include altitude
		Z_pattern.setSize(lengthMeasurement); Z_pattern[lengthMeasurement-1] = msg->altitude;
		H_pattern.setSize(lengthMeasurement, SYSTEM_STATE_LEN, 0); H_pattern[lengthMeasurement-1][3] = 1;
		R_pattern.setSize(lengthMeasurement, lengthMeasurement, 0); R_pattern[lengthMeasurement-1][lengthMeasurement-1] = pow(kalman_filter.SDaltitude,2);
		lengthMeasurement ++;
		// include surgeVel
		Z_pattern.setSize(lengthMeasurement); Z_pattern[lengthMeasurement-1] = msg->body_velocity.x;
		H_pattern.setSize(lengthMeasurement, SYSTEM_STATE_LEN, 0); H_pattern[lengthMeasurement-1][4] = 1;
		R_pattern.setSize(lengthMeasurement, lengthMeasurement, 0); R_pattern[lengthMeasurement-1][lengthMeasurement-1] = pow(kalman_filter.SDu,2);
		lengthMeasurement ++;
		// include swayVel
		Z_pattern.setSize(lengthMeasurement); Z_pattern[lengthMeasurement-1] = msg->body_velocity.y;
		H_pattern.setSize(lengthMeasurement, SYSTEM_STATE_LEN, 0); H_pattern[lengthMeasurement-1][5] = 1;
		R_pattern.setSize(lengthMeasurement, lengthMeasurement, 0); R_pattern[lengthMeasurement-1][lengthMeasurement-1] = pow(kalman_filter.SDv,2);
		}
		//***
		if(kalman_filter.useCompassSensor){
		lengthMeasurement ++;
		// include pitch
		Z_pattern.setSize(lengthMeasurement); Z_pattern[lengthMeasurement-1] = msg->orientation.pitch;
		H_pattern.setSize(lengthMeasurement, SYSTEM_STATE_LEN, 0); H_pattern[lengthMeasurement-1][8] = 1;
		R_pattern.setSize(lengthMeasurement, lengthMeasurement, 0); R_pattern[lengthMeasurement-1][lengthMeasurement-1] = pow(kalman_filter.SDpitch,2);
		lengthMeasurement ++;
		// include pitchRate
		Z_pattern.setSize(lengthMeasurement); Z_pattern[lengthMeasurement-1] = msg->orientation_rate.pitch ;
		H_pattern.setSize(lengthMeasurement, SYSTEM_STATE_LEN, 0); H_pattern[lengthMeasurement-1][10] = 1;
		R_pattern.setSize(lengthMeasurement, lengthMeasurement, 0); R_pattern[lengthMeasurement-1][lengthMeasurement-1] = pow(kalman_filter.SDpitchRate,2);
		}
		//***
		if(kalman_filter.useGyroSensor){
		lengthMeasurement ++;
		// include yaw
		Z_pattern.setSize(lengthMeasurement); Z_pattern[lengthMeasurement-1] = msg->orientation.yaw;
		H_pattern.setSize(lengthMeasurement, SYSTEM_STATE_LEN, 0); H_pattern[lengthMeasurement-1][7] = 1;
		R_pattern.setSize(lengthMeasurement, lengthMeasurement, 0); R_pattern[lengthMeasurement-1][lengthMeasurement-1] = pow(kalman_filter.SDyaw,2);
		lengthMeasurement ++;
		// include yawRate
		Z_pattern.setSize(lengthMeasurement); Z_pattern[lengthMeasurement-1] = msg->orientation_rate.yaw ;
		H_pattern.setSize(lengthMeasurement, SYSTEM_STATE_LEN, 0); H_pattern[lengthMeasurement-1][9] = 1;
		R_pattern.setSize(lengthMeasurement, lengthMeasurement, 0); R_pattern[lengthMeasurement-1][lengthMeasurement-1] = pow(kalman_filter.SDyawRate,2);
		}
		//***
		if(kalman_filter.useLblSensor){
		lengthMeasurement ++;
		// include north
		Z_pattern.setSize(lengthMeasurement); Z_pattern[lengthMeasurement-1] = msg->position.north;
		H_pattern.setSize(lengthMeasurement, SYSTEM_STATE_LEN, 0); H_pattern[lengthMeasurement-1][0] = 1;
		R_pattern.setSize(lengthMeasurement, lengthMeasurement, 0); R_pattern[lengthMeasurement-1][lengthMeasurement-1] = pow(kalman_filter.SDnorth,2);
		lengthMeasurement ++;
		// include east
		Z_pattern.setSize(lengthMeasurement); Z_pattern[lengthMeasurement-1] = msg->position.east;
		H_pattern.setSize(lengthMeasurement, SYSTEM_STATE_LEN, 0); H_pattern[lengthMeasurement-1][1] = 1;
		R_pattern.setSize(lengthMeasurement, lengthMeasurement, 0); R_pattern[lengthMeasurement-1][lengthMeasurement-1] = pow(kalman_filter.SDeast,2);
		// depth as well should be there
		}
		cout << "measurement size: " << lengthMeasurement << endl;
	olibKalmanMeasure measurement_kalman_filter(lengthMeasurement, &kalman_filter);// temporary measurement object
	measurement_kalman_filter.setR(R_pattern);
	measurement_kalman_filter.setJ(H_pattern);
	measurement_kalman_filter.setz(Z_pattern);
		// predicted obsetvation
	olibVector<double64> Hx;
	Hx = H_pattern * EKF5DOF.getXPrediction();
	measurement_kalman_filter.seth(Hx);
		// debug code - extract out the innovation
		//olibVector <double64> inn;
		//inn = measurementEKF5DOF.getInnovation();
		//fprintf(debugFile, "%f\n", inn[0]);
		// debug code
	kalman_filter.Correct(measurement_kalman_filter);



	// store the result, correction has been done
	olibVector<double64> CorrectedState(SYSTEM_STATE_LEN, 0);
	olibVector<double64> PredictedState(SYSTEM_STATE_LEN, 0);
	olibMatrix<double64> CorrectedP(SYSTEM_STATE_LEN, SYSTEM_STATE_LEN);
	olibMatrix<double64> PredictedP(SYSTEM_STATE_LEN, SYSTEM_STATE_LEN);
	CorrectedState = kalman_filter.getXCorrected();
	PredictedState = kalman_filter.getXPrediction();
	CorrectedP = kalman_filter.getPCorrected();
	PredictedP = kalman_filter.getPPrediction();

	cout << "Measurement: \n"          << measurement_kalman_filter.getMeasurement() << endl;
	cout << "PredictedMeasurement: \n" << measurement_kalman_filter.getPredictedMeasurement() << endl;
	cout << "PredictedState: \n" << PredictedState[0] << " and " << PredictedState[1] << endl;
	cout << "CorrectedState: \n" << CorrectedState[0] << " and " << CorrectedState[1] << endl;

	// store values in files
	north = CorrectedState[0]; east = CorrectedState[1]; depth = CorrectedState[2]; altitude = CorrectedState[3];
	surgeVelocity = CorrectedState[4]; swayVelocity = CorrectedState[5]; heaveVelocity = CorrectedState[6];
	yaw = CorrectedState[7]; pitch = CorrectedState[8];
	yawRate = CorrectedState[9]; pitchRate = CorrectedState[10];

	double64 interval = kalman_filter.getDt();
	fprintf(pInputMessages,"%f, %f, %i, %f, %f, %f,  %f, %f,  %f, %f, %f, %f,  %f, %f\n",
			(double64)msg->header.stamp.sec+((double64)msg->header.stamp.nsec*1e-9),
			interval, whoWasIt,
			msg->position.north, msg->position.east, msg->position.depth, msg->altitude,
			msg->body_velocity.x, msg->body_velocity.y, msg->body_velocity.z,
			msg->orientation.yaw, msg->orientation.pitch,
			msg->orientation_rate.yaw, msg->orientation_rate.pitch
			);
	fprintf(pPredictedState, "%f, %f, %f,  %f, %f,  %f, %f, %f, %f,  %f, %f\n",
			PredictedState[0], PredictedState[1], PredictedState[2], PredictedState[3],
			PredictedState[4], PredictedState[5], PredictedState[6], PredictedState[7],
			PredictedState[8], PredictedState[9], PredictedState[10]
			);

	fprintf(pCorrectedState, "%f, %f, %f,  %f, %f,  %f, %f, %f, %f,  %f, %f\n",
			CorrectedState[0], CorrectedState[1], CorrectedState[2], CorrectedState[3],
			CorrectedState[4], CorrectedState[5], CorrectedState[6], CorrectedState[7],
			CorrectedState[8], CorrectedState[9], CorrectedState[10]
			);
	fprintf(pCorrectedP, "%f, %f, %f,  %f, %f,  %f, %f, %f, %f,  %f, %f\n",
			CorrectedP[0][0], CorrectedP[1][1], CorrectedP[2][2], CorrectedP[3][3],
			CorrectedP[4][4], CorrectedP[5][5], CorrectedP[6][6], CorrectedP[7][7],
			CorrectedP[8][8], CorrectedP[9][9], CorrectedP[10][10]
			);
	fprintf(pPredictedP, "%f, %f, %f,  %f, %f,  %f, %f, %f, %f,  %f, %f\n",
			PredictedP[0][0], PredictedP[1][1], PredictedP[2][2], PredictedP[3][3],
			PredictedP[4][4], PredictedP[5][5], PredictedP[6][6], PredictedP[7][7],
			PredictedP[8][8], PredictedP[9][9], PredictedP[10][10]
			);
	}
}
//---------------------------------------------------------------------------
void navigationRos::cleanup() {
	PRINTLN_INFO("cleanup entered");
}
// ---------------------------------------------------------------------------
int main(int argc, char * argv[]) {
	navigationRos modCore;
	pInputMessages = fopen ("inputMessages.csv" , "w");
	pPredictedState = fopen("predictedState.csv", "w");
	pCorrectedState = fopen("correctedState.csv", "w");
	pPredictedP     = fopen("predictedP.csv", "w");
	pCorrectedP     = fopen("correctedP.csv", "w");
	//fprintf(pFile,"\"northOrig\",\"eastOrig\", \"yawOrig\", \"yawRateOrig\", \"FILTER\",\"northPred\",\"eastPred\",\"depthPred\",\"uPred\",\"vPred\",\"wPred\",\"yawPred\",\"pitchPred\",\"yawRatePred\",\"pitchRatePred\", \"dT\", \"northCorr\",\"eastCorr\",\"depthCorr\",\"uCorr\",\"vCorr\",\"wCorr\",\"yawCorr\",\"pitchCorr\",\"yawRatecorr\",\"pitchRateCorr\"\n");
	//fprintf(debugFile, "\"innovationNorth\"\n");
	return modCore.main(argc, argv);
	fclose (pInputMessages);
	fclose (pPredictedState);
	fclose (pCorrectedState);
	fclose (pPredictedP);
	fclose (pCorrectedP);
}
