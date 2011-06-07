#include <algorithm>
#include <osl_core/GeneralMath.h>
#include "nav_ekf_rosREAD.h"

		extern FILE *pInputMessages;
		extern FILE *pOutputMessages;
		extern FILE *pPredictedState;
		extern FILE *pCorrectedState;
		extern FILE *pCorrectedP;
		extern FILE *pObservation;
		extern FILE *pPredictedObservation;

using namespace std;

navigationRos::navigationRos()
: ObservationVec(1,0),
  ObservationStates(1, NULL_STATE),
  MeasurementVec(1,0),
  MeasurementStates(1,NULL_STATE)
{

}

navigationRos::~navigationRos(){
}

bool navigationRos::init(int argc, char * argv[]) {
	
	sendMsgPeriod = 200;
	ekfPeriod     = 100;//default value assign
	ekfMode = EVERY_MESSAGE; //default, possible   EVERY_MESSAGE; //EVERY_DT

	if(argc == 3)
	{// periodic kalman update
		if(strcmp(argv[argc-2], "a") == 0) { ekfMode = EVERY_MESSAGE; }	
		else if (strcmp(argv[argc-2], "p") == 0) { 
			ekfMode = EVERY_DT; 
			ekfPeriod = atof(argv[argc-1]);
		}
		else { cout << "ERROR! Invalid periodicity argument. Place 'a' (aperiodic) or 'p' (periodic)." << endl; 
		requestShutdown(); 
		//exit(1); 
		}
	}	
	if(argc == 2)
	{// assumed aperiodic
		if(strcmp(argv[argc-1], "a") == 0) {  ekfMode = EVERY_MESSAGE; }
		else{ cout << "ERROR! Invalid periodicity argument. Place 'a' for aperiodic update." << endl; 
		//exit(1); 
		requestShutdown();
		}
	}	
	//COUNT_FILT = 0;

	//requestShutdown();
				T_EKF5DOF_PARAMS EKF5DOF_params; 
	                        // load filtering variables
				// (nameInTheCode, nameInTheFile, sectionName)
				if (!getINIFloat(EKF5DOF_params.SDnorth, "SDnorth", "EKF"))
						return false;
						cout << EKF5DOF_params.SDnorth << endl;
				if (!getINIFloat(EKF5DOF_params.SDeast, "SDeast", "EKF"))
						return false;
				if (!getINIFloat(EKF5DOF_params.SDdepth, "SDdepth", "EKF"))
						return false;
				if (!getINIFloat(EKF5DOF_params.SDaltitude, "SDaltitude", "EKF"))
						return false;
				if (!getINIFloat(EKF5DOF_params.SDu, "SDu", "EKF"))
						return false;
				if (!getINIFloat(EKF5DOF_params.SDv, "SDv", "EKF"))
						return false;
				if (!getINIFloat(EKF5DOF_params.SDw, "SDw", "EKF"))
						return false;
				if (!getINIFloat(EKF5DOF_params.SDyaw, "SDyaw", "EKF"))
						return false;
				if (!getINIFloat(EKF5DOF_params.SDpitch, "SDpitch", "EKF"))
						return false;		
				if (!getINIFloat(EKF5DOF_params.SDyawRate, "SDyawRate", "EKF"))
						return false;
				if (!getINIFloat(EKF5DOF_params.SDpitchRate, "SDpitchRate", "EKF"))
						return false;
		
		
				if (!getINIFloat(EKF5DOF_params.SDuModel, "SDuModel", "EKF"))
						return false;
				if (!getINIFloat(EKF5DOF_params.SDvModel, "SDvModel", "EKF"))
						return false;
				if (!getINIFloat(EKF5DOF_params.SDwModel, "SDwModel", "EKF"))
						return false;	
				if (!getINIFloat(EKF5DOF_params.SDyawRateModel, "SDyawRateModel", "EKF"))
						return false;
				if (!getINIFloat(EKF5DOF_params.SDpitchRateModel, "SDpitchRateModel", "EKF"))
						return false;			
	
	
	EKF5DOF.init(ekfMode, EKF5DOF_params);
	gotHeading = gotGpsFix = false;
	pitch = yaw = roll = 0.0;
	pitchRate = yawRate =  rollRate = 0.0;
	surgeVelocity = swayVelocity = heaveVelocity = 0.0;
	north = east = altitude = depth = 0.0;
	northNAV =  northLBL = eastNAV = eastLBL = 0.0;

	northFiltered = eastFiltered = depthFiltered = altitudeFiltered = 0.0;
	surgeVelocityFiltered = swayVelocityFiltered = heaveVelocityFiltered = 0.0;
	pitchFiltered = yawFiltered = pitchRateFiltered = yawRateFiltered = 0.0;
	
	MeasurementSensor = NULL_SENSOR;
	
	m_timer.setTimer(1);
	m_timer_ekf_TRIGGER.setTimer(1);
	m_timer_ekf_MES = new osl_core::Chrono();
	m_timer_ekf_MES->Reset();  
	
	/////////////////////////////////  ROS   //////////////////////////////
//	tfMapFrame_ = "/map";
//	tfBaseLinkFrame_ = "base_link";

	ros::TransportHints hints = ros::TransportHints().udp().tcpNoDelay();
	m_rosInputNavSub = m_rosHandle->subscribe<auv_msgs::NavSts>("nav/nav_sts",
 			1, &navigationRos::handleROSnavMsg, this, hints);
//	m_rosInputLblNavSub = m_rosHandle->subscribe<nav_msgs::Odometry>("nav/lbl_from_origin",
// 			1, &navigationRos::handleROSlblMsg, this, hints);
 	// advertise for navigation and rviz		
//	m_rosFiltNavPub = m_rosHandle->advertise<auv_msgs::NavSts>("nav/filt_nav_sts", 10);
//	m_rosFiltOdometryPub = m_rosHandle->advertise<nav_msgs::Odometry>("nav/filt_odometry", 10);

        ///////////////////////////////////////////////////////////////////////
	return true;

} 

void navigationRos::addMeasurementToObservation(olibVector<double64> & ObservationVec, olibVector<stateID> & ObservationStates, olibVector<double64> & MeasurementVec, olibVector<stateID> & MeasurementStates)
{
	uint32 prevObservationLength, MeasurementLength, newObservationLength;
	prevObservationLength = ObservationVec.size();
	newObservationLength = prevObservationLength;
	MeasurementLength = MeasurementVec.size();
	cout << "mes length = " << MeasurementLength << "  obs length = " << prevObservationLength << endl;
	
	if((prevObservationLength==1)&&(ObservationStates[0]==NULL_STATE))
	{//observation empty, add all measurement values to observation
		cout << "renew..." << endl;
		ObservationVec.setSize(MeasurementLength, 0);
		ObservationStates.setSize(MeasurementLength, NULL_STATE);
		ObservationVec = MeasurementVec;
		ObservationStates = MeasurementStates;
	}
	else
	{// observation not empty, check which values to add
		uint32 mesIndex = 0;
		uint32 obsIndex = 0;
		for(mesIndex=0; mesIndex<MeasurementLength; mesIndex++)
		{
			//check if it exists in observation
			for(obsIndex=0; obsIndex<prevObservationLength; obsIndex++)
			{
				if(MeasurementStates[mesIndex]==ObservationStates[obsIndex])
				{//existing state comes from measurement
					ObservationVec[obsIndex] = MeasurementVec[mesIndex];//just update
					break;
				}
				
			
			}
			if(obsIndex==prevObservationLength)
			{
				//cout << "before  " << ObservationStates << endl;
				//add a new element in observation
				newObservationLength++;
				//cout << "new length " << newObservationLength << endl;
				ObservationVec.setSize(newObservationLength,0);
				ObservationVec[newObservationLength-1] = MeasurementVec[mesIndex];
				//cout << "after  " << ObservationStates << endl;
				//cout << MeasurementStates[mesIndex] << endl;
				ObservationStates.setSize(newObservationLength, NULL_STATE);
				ObservationStates[newObservationLength-1] = MeasurementStates[mesIndex];
				//cout << "come on?" << endl;
			}
			//cout << mesIndex << "   " << MeasurementLength << endl;
 		}
	}
}

void navigationRos::handleROSnavMsg(const auv_msgs::NavStsConstPtr & msg)
{
    		northNAV = msg->position.north;
    		eastNAV  = msg->position.east;
    		depth = msg->position.depth;
    		altitude       = msg->altitude;
    		surgeVelocity = msg->body_velocity.x;
    		swayVelocity = msg->body_velocity.y;
    		heaveVelocity = msg->body_velocity.z;
    		yaw = msg->orientation.yaw;
       	 	pitch = msg->orientation.pitch;
    		yawRate   = msg->orientation_rate.yaw;
    		pitchRate = msg->orientation_rate.pitch;
    	if(EKF5DOF.isFilteringPossible())
    	{
    		// it is not a first message for Kalman - had some before
    		// elapsed time
		MeasurementVec.setSize(8, 0);//enable enough space
		MeasurementStates.setSize(8, NULL_STATE);//enable enough space 
    		
    		//MeasurementVec[0] = north;         MeasurementStates[0] = NORTH;
		//MeasurementVec[1] = east;          MeasurementStates[1] = EAST;
    		MeasurementVec[0] = depth;         MeasurementStates[0] = DEPTH;
		MeasurementVec[1] = altitude;      MeasurementStates[1] = ALTITUDE;
    		MeasurementVec[2] = surgeVelocity; MeasurementStates[2] = SURGE_VELOCITY;
		MeasurementVec[3] = swayVelocity;  MeasurementStates[3] = SWAY_VELOCITY;
    		MeasurementVec[4] = heaveVelocity; MeasurementStates[4] = HEAVE_VELOCITY;
		MeasurementVec[5] = yaw;           MeasurementStates[5] = YAW;		
		MeasurementVec[6] = pitch;         MeasurementStates[6] = PITCH;
    		//temporarily expell
    		//MeasurementVec[7] = yawRate;       MeasurementStates[7] = YAW_RATE;
		MeasurementVec[7] = pitchRate;     MeasurementStates[7] = PITCH_RATE;
    		if(EKF5DOF.ekfMode==EVERY_MESSAGE)
    		{// this option is not ment to work with ROS - just a dummy code
    			//cout << "#------ ### NAV ROS MSG ### ------   "  << elapsedPeriod << endl;
    		}	
		if(EKF5DOF.ekfMode==EVERY_DT)
		{//aim is to create an oservatio that will be periodically processed within timer defined interval
			addMeasurementToObservation(ObservationVec, ObservationStates, MeasurementVec, MeasurementStates);
		}
    	}
    	else
    	{// first msg
     		olibVector<double64> X(SYSTEM_STATE_LEN, 0);
    		cout << X << endl;
    		X[NORTH_INDEX]     = northNAV;
    		X[EAST_INDEX]      = eastNAV;
		X[DEPTH_INDEX]     = depth;
		X[ALTITUDE_INDEX]  = altitude;
		X[SURGE_VEL_INDEX] = surgeVelocity;
		X[SWAY_VEL_INDEX]  = swayVelocity;
		X[HEAVE_VEL_INDEX] = heaveVelocity;
		X[YAW_INDEX]       = yaw;
		X[PITCH_INDEX]     = pitch;
		X[YAW_RATE_INDEX]  = yawRate;
		X[PITCH_RATE_INDEX]= pitchRate;
		cout << X << endl;		
		m_timer_ekf_MES->Reset();

		EKF5DOF.setXPrediction(X); EKF5DOF.setXCorrected(X);
		gotHeading = true; gotGpsFix = true;
		if(gotHeading && gotGpsFix) {
		EKF5DOF.setFilteringPossible();
		cout << "FILTERING ENABLED... " << endl;
		}
	} 
} // Nav Msg

void navigationRos::handleROSlblMsg(const nav_msgs::OdometryConstPtr & msg)
{
} // LBL Msg
// ---------------------------------------------------------------------------
void navigationRos::doWork()
{
	if (m_timer.getTimer() == 0)
	{
		m_timer.setTimer(sendMsgPeriod);//send message period
		//sendROSNavSts();
		//sendROSFiltOdometry();
	} 
	
	if(EKF5DOF.ekfMode==EVERY_DT)
	{
	//m_timer_ekf_TRIGGER
	if(m_timer_ekf_TRIGGER.getTimer() == 0){
		m_timer_ekf_TRIGGER.setTimer(ekfPeriod);
		

		
		if(EKF5DOF.isFilteringPossible())
		{
		// following section executes every ekfPeriod
		// use the observation 
		
		double64 elapsedPeriod;
		uint32_t elapsedMicroSeconds;
		elapsedMicroSeconds = m_timer_ekf_MES->ReadAndReset();
		elapsedPeriod = elapsedMicroSeconds * 0.000001;
		
		//cout << "LBL values: " << northLBL << " " << eastLBL << endl;
		olibVector<double64> filteredValues(SYSTEM_STATE_LEN,0);
		cout << "#------ ### EXTENDED KALMAN (EVERY_DT) TRIGGERED ### ------   "  << elapsedPeriod << endl;

		filteredValues = EKF5DOF.filterObservation(ObservationVec, ObservationStates, elapsedPeriod); 
		
		storeFilteringResults(filteredValues, elapsedPeriod);
		uint32 len;
		len = ObservationVec.size();
		if(len==8){
		fprintf(pObservation, "%f, %f, %f,  %f, %f,  %f, %f, %f\n",
			ObservationVec[0], ObservationVec[1], ObservationVec[2], ObservationVec[3],
			ObservationVec[4], ObservationVec[5], ObservationVec[6], ObservationVec[7]
			);
		}				

		//reset observation for the next round
		ObservationVec.setSize(1,0);    ObservationVec[0] = 0;
		ObservationStates.setSize(1,NULL_STATE); ObservationStates[0] = NULL_STATE;
		}	
	} 	
	}
}
//---------------------------------------------------------------------------
void navigationRos::sendROSNavSts() {
}
//---------------------------------------------------------------------------
void navigationRos::sendROSFiltOdometry() {
}

void navigationRos::storeFilteringResults(olibVector<double64> filteredStateVec, double64 elapsedPeriod)
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

	// store the result, correction has been done
	olibVector<double64> CorrectedState(SYSTEM_STATE_LEN, 0);
	olibVector<double64> PredictedState(SYSTEM_STATE_LEN, 0);
	olibMatrix<double64> CorrectedP(SYSTEM_STATE_LEN, SYSTEM_STATE_LEN);
	//olibMatrix<double64> PredictedP(SYSTEM_STATE_LEN, SYSTEM_STATE_LEN);
	CorrectedState = EKF5DOF.getXCorrected();
	PredictedState = EKF5DOF.getXPrediction();
	CorrectedP = EKF5DOF.getPCorrected();
	//PredictedP = EKF5DOF.getPPrediction();

	//cout << "Measurement: \n"          << measurement_kalman_filter.getMeasurement() << endl;
	//cout << "PredictedMeasurement: \n" << measurement_kalman_filter.getPredictedMeasurement() << endl;
	//cout << "PredictedState: \n" << PredictedState[0] << " and " << PredictedState[1] << endl;
	//cout << "CorrectedState: \n" << CorrectedState[0] << " and " << CorrectedState[1] << endl;

	// store values in files
	////north = CorrectedState[0]; east = CorrectedState[1]; depth = CorrectedState[2]; altitude = CorrectedState[3];
	////surgeVelocity = CorrectedState[4]; swayVelocity = CorrectedState[5]; heaveVelocity = CorrectedState[6];
	////yaw = CorrectedState[7]; pitch = CorrectedState[8];
	////yawRate = CorrectedState[9]; pitchRate = CorrectedState[10];

	/*
	fprintf(pInputMessages,"%f, %f, %f,  %f, %f,  %f, %f, %f, %f,  %f, %f\n",
			north, east, depth, altitude,
			surgeVelocity, swayVelocity, heaveVelocity,
			yaw, pitch,
			yawRate, pitchRate
			);
	*/
		fprintf(pInputMessages,"%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
			northNAV, eastNAV, northLBL, eastLBL, northFiltered, eastFiltered, 
			surgeVelocity, surgeVelocityFiltered, swayVelocity, swayVelocityFiltered,
			yaw, yawFiltered, pitch, pitchFiltered, 
			yawRate, yawRateFiltered, pitchRate, pitchRateFiltered,
			elapsedPeriod
			);		
			
			
	fprintf(pCorrectedState, "%f, %f, %f,  %f, %f,  %f, %f, %f, %f,  %f, %f\n",
			northFiltered, eastFiltered, depthFiltered, altitudeFiltered,
			surgeVelocityFiltered, swayVelocityFiltered, heaveVelocityFiltered, 
			yawFiltered, pitchFiltered, 
			yawRateFiltered, pitchRateFiltered
			);			
	
	fprintf(pPredictedState, "%f, %f, %f,  %f, %f,  %f, %f, %f, %f,  %f, %f\n",
			PredictedState[0], PredictedState[1], PredictedState[2], PredictedState[3],
			PredictedState[4], PredictedState[5], PredictedState[6], PredictedState[7],
			PredictedState[8], PredictedState[9], PredictedState[10]
			);
	
	fprintf(pCorrectedP, "%f, %f, %f,  %f, %f,  %f, %f, %f, %f,  %f, %f\n",
			CorrectedP[0][0], CorrectedP[1][1], CorrectedP[2][2], CorrectedP[3][3],
			CorrectedP[4][4], CorrectedP[5][5], CorrectedP[6][6], CorrectedP[7][7],
			CorrectedP[8][8], CorrectedP[9][9], CorrectedP[10][10]
			);

//	fprintf(pPredictedP, "%f, %f, %f,  %f, %f,  %f, %f, %f, %f,  %f, %f\n",
//			PredictedP[0][0], PredictedP[1][1], PredictedP[2][2], PredictedP[3][3],
//			PredictedP[4][4], PredictedP[5][5], PredictedP[6][6], PredictedP[7][7],
//			PredictedP[8][8], PredictedP[9][9], PredictedP[10][10]
//			);
	
} //store

//---------------------------------------------------------------------------
void navigationRos::cleanup() {

	PRINTLN_INFO("cleanup entered");
	delete m_timer_ekf_MES;

}

// ---------------------------------------------------------------------------
int main(int argc, char * argv[]) {
	navigationRos modCore;
	cout << "class created..." << endl;
	pInputMessages = fopen ("inputMessages.csv" , "w");
	pPredictedState = fopen("predictedState.csv", "w");
	pCorrectedState = fopen("correctedState.csv", "w");
	//pPredictedP     = fopen("predictedP.csv", "w");
	pCorrectedP     = fopen("correctedP.csv", "w");
	pObservation    = fopen("Observation.csv", "w");
	pPredictedObservation = fopen("predObservation.csv", "w");
	//fprintf(debugFile, "\"innovationNorth\"\n");
	return modCore.main(argc, argv);
	fclose (pInputMessages);
	fclose (pPredictedState);
	fclose (pCorrectedState);
	//fclose (pPredictedP);
	fclose (pCorrectedP);
	fclose(pObservation);
	fclose(pPredictedObservation);
}
