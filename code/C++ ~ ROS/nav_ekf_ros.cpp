///////// SIMULATION ///////////
#include <algorithm>
#include<string>
#include <osl_core/GeneralMath.h>
#include "nav_ekf_ros.h"
/// LOG FILES ///
	extern FILE *pLogInput;
	extern FILE *pLogOutput;
	extern FILE *pLogObser;	
	extern FILE *pLogGps;
	extern FILE *pLogParams;
	extern FILE *pOldNav;

using namespace std;
// fn for string
char * removeWhiteSpaces(char * inputString){
	char *p1  = inputString;
  	char *p2  = inputString;
  	char *p2_;
  	p2_ = p2; 
   	while(*p1 != 0) {
   		if(isspace(*p1)) {//ispunct(*p1) || 
   			++p1;
   		}
	   	else {
	   		*p2++ = *p1++; 
   		}
   	}
   	*p2 = 0; 
   	return p2_;	
}


navigationRos::navigationRos()
: ObservationVec(1,0),
  ObservationStates(1, NULL_STATE),
  MeasurementVec(1,0),
  MeasurementStates(1,NULL_STATE)
{

}
// 
navigationRos::~navigationRos(){
}

bool navigationRos::init(int argc, char * argv[]) {
	
	sendMsgPeriod = 100;
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

				if (!getINIBool(fakeGPS, "fakeGPS", "NAVIGATION"))
					return false;					
				T_EKF5DOF_PARAMS EKF5DOF_params; 
	                        // load filtering variables
				// (nameInTheCode, nameInTheFile, sectionName)
				if (!getINIFloat(EKF5DOF_params.SDnorth, "SDnorth", "EKF"))
						return false;
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
fprintf(pLogParams,"SDnorth: %f \nSDeast: %f \nSDdepth: %f \nSDaltitude: %f \nSDu: %f \nSDv: %f \nSDw: %f \nSDyaw: %f \nSDpitch: %f \nSDyawRate: %f \nSDpitchRate: %f \nSDuModel: %f \nSDvModel: %f \nSDwModel: %f \nSDyawRateModel: %f \nSDpitchRateModel: %f \nPeriodEKF: %i \n", EKF5DOF_params.SDnorth,EKF5DOF_params.SDeast,EKF5DOF_params.SDdepth,EKF5DOF_params.SDaltitude,
				EKF5DOF_params.SDu,EKF5DOF_params.SDv,EKF5DOF_params.SDw,EKF5DOF_params.SDyaw,EKF5DOF_params.SDpitch,EKF5DOF_params.SDyawRate,EKF5DOF_params.SDpitchRate,
				EKF5DOF_params.SDuModel,EKF5DOF_params.SDvModel,EKF5DOF_params.SDwModel,EKF5DOF_params.SDyawRateModel,EKF5DOF_params.SDpitchRateModel,ekfPeriod);	
	
	EKF5DOF.init(ekfMode, EKF5DOF_params);
	gotHeading = gotGpsFix = firstDepthMsgReceived = gotLBL = false;
	//if (fakeGPS)
	//{
	//gotGpsFix = true;
	//}
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
	tfMapFrame_ = "/map";
	tfBaseLinkFrame_ = "base_link";

	ros::TransportHints hints = ros::TransportHints().udp().tcpNoDelay();
	m_rosInputNavSub = m_rosHandle->subscribe<auv_msgs::NavSts>("nav/nav_sts",
 			1, &navigationRos::handleROSnavMsg, this, hints);
	m_rosInputLblNavSub = m_rosHandle->subscribe<nav_msgs::Odometry>("nav/lbl_from_origin",
 			1, &navigationRos::handleROSlblMsg, this, hints);
 	// advertise for navigation and rviz		
	m_rosFiltNavPub = m_rosHandle->advertise<auv_msgs::NavSts>("nav/filt_nav_sts", 10);
	m_rosFiltOdometryPub = m_rosHandle->advertise<nav_msgs::Odometry>("nav/filt_odometry", 10);

        ///////////////////////////////////////////////////////////////////////
	return true;

} 

void navigationRos::addMeasurementToObservation(olibVector<double64> & ObservationVec, olibVector<stateID> & ObservationStates, olibVector<double64> & MeasurementVec, olibVector<stateID> & MeasurementStates)
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
				newObservationLength++;
				//cout << "new length " << newObservationLength << endl;
				ObservationVec.setSize(newObservationLength,0);
				ObservationVec[newObservationLength-1] = MeasurementVec[mesIndex];
				ObservationStates.setSize(newObservationLength, NULL_STATE);
				ObservationStates[newObservationLength-1] = MeasurementStates[mesIndex];
			}
		}
	}
}
// ---------------------------------------------------------------------------
void navigationRos::handleROSnavMsg(const auv_msgs::NavStsConstPtr & msg)
{
cout << "GOT NAV MSG " << endl;
    		north = northNAV = msg->position.north;
    		east = eastNAV  = msg->position.east;
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
 
    		if (gotLBL){
    		//cout << "GOT NAV AS LBL" << endl;
		MeasurementVec.setSize(2, 0);//enable enough space
		MeasurementStates.setSize(2, NULL_STATE);//enable enough space 
     		MeasurementVec[0] = north;         MeasurementStates[0] = NORTH;
		MeasurementVec[1] = east;          MeasurementStates[1] = EAST;
		northLBL = north;
		eastLBL  = east;
    			double64 elapsedPeriod;  
    			uint32_t elapsedMicroSeconds;
    			elapsedMicroSeconds = m_timer_ekf_MES->ReadAndReset();
    			elapsedPeriod = elapsedMicroSeconds * 0.000001;
    		sensorType sensorDevice = LBL_SENSOR; 
						// for logging...
						olibVector<double64> previousState(SYSTEM_STATE_LEN,0);
						previousState = EKF5DOF.getXCorrected();
						olibMatrix<double64> previousP(SYSTEM_STATE_LEN,SYSTEM_STATE_LEN,0);
						previousP     = EKF5DOF.getPCorrected();
						////
		olibVector<double64> filteredValues(SYSTEM_STATE_LEN,0); // storage for filtered state
		cout << "#------ ### LBL TRIGGER EKF ### ------   "  << elapsedPeriod << endl;
		cout << "obs: " << MeasurementVec << endl;
		filteredValues = EKF5DOF.filterMessage(MeasurementVec, sensorDevice, elapsedPeriod);  
						// for logging...
						olibMatrix<double64> filteredP(SYSTEM_STATE_LEN,SYSTEM_STATE_LEN,0);
						filteredP     = EKF5DOF.getPCorrected();
						////		
		storeFilteringResults(filteredValues, elapsedPeriod);
		storeLogFile(elapsedPeriod, previousState, previousP, MeasurementVec, MeasurementStates, filteredValues, filteredP);  
		gotLBL = false;  			
    		}
    		else
    		{
    		//cout << "GOT NAV " << endl;
    		//MeasurementVec[0] = north;         MeasurementStates[0] = NORTH;
		//MeasurementVec[1] = east;          MeasurementStates[1] = EAST;
		MeasurementVec.setSize(9, 0);//enable enough space
		MeasurementStates.setSize(9, NULL_STATE);//enable enough space 
    		MeasurementVec[0] = depth;         MeasurementStates[0] = DEPTH;
		MeasurementVec[1] = altitude;      MeasurementStates[1] = ALTITUDE;
    		MeasurementVec[2] = surgeVelocity; MeasurementStates[2] = SURGE_VELOCITY;
		MeasurementVec[3] = swayVelocity;  MeasurementStates[3] = SWAY_VELOCITY;
    		MeasurementVec[4] = heaveVelocity; MeasurementStates[4] = HEAVE_VELOCITY;
		MeasurementVec[5] = yaw;           MeasurementStates[5] = YAW;		
		MeasurementVec[6] = pitch;         MeasurementStates[6] = PITCH;
    		MeasurementVec[7] = yawRate;       MeasurementStates[7] = YAW_RATE;
		MeasurementVec[8] = pitchRate;     MeasurementStates[8] = PITCH_RATE;
		addMeasurementToObservation(ObservationVec, ObservationStates, MeasurementVec, MeasurementStates);
		}

    	}
    	else
    	{// first msg
     		olibVector<double64> X(SYSTEM_STATE_LEN, 0);
    		X[NORTH_INDEX]     = north;
    		X[EAST_INDEX]      = east;
		X[DEPTH_INDEX]     = depth;
		X[ALTITUDE_INDEX]  = altitude;
		X[SURGE_VEL_INDEX] = surgeVelocity;
		X[SWAY_VEL_INDEX]  = swayVelocity;
		X[HEAVE_VEL_INDEX] = heaveVelocity;
		X[YAW_INDEX]       = yaw;
		X[PITCH_INDEX]     = pitch;
		X[YAW_RATE_INDEX]  = yawRate;
		X[PITCH_RATE_INDEX]= pitchRate;
		
		m_timer_ekf_MES->Reset();

		EKF5DOF.setXPrediction(X); EKF5DOF.setXCorrected(X);
					olibMatrix<double64> P(SYSTEM_STATE_LEN, SYSTEM_STATE_LEN, 0); // covariance matrix
					P = EKF5DOF.getPCorrected(); // take current
					P[DEPTH_INDEX][DEPTH_INDEX] = pow(EKF5DOF.SDdepth,2);
					P[ALTITUDE_INDEX][ALTITUDE_INDEX] = pow(EKF5DOF.SDaltitude,2); 
					P[SURGE_VEL_INDEX][SURGE_VEL_INDEX] = pow(EKF5DOF.SDu,2);    
					P[SWAY_VEL_INDEX][SWAY_VEL_INDEX] = pow(EKF5DOF.SDv,2);
					P[HEAVE_VEL_INDEX][HEAVE_VEL_INDEX] = pow(EKF5DOF.SDw,2);        
					P[YAW_INDEX][YAW_INDEX] = pow(EKF5DOF.SDyaw,2);  
					P[PITCH_INDEX][PITCH_INDEX] = pow(EKF5DOF.SDpitch,2);
					P[YAW_RATE_INDEX][YAW_RATE_INDEX] = pow(EKF5DOF.SDyawRate,2);  
					P[PITCH_RATE_INDEX][PITCH_RATE_INDEX] = pow(EKF5DOF.SDpitchRate,2);
					EKF5DOF.setPPrediction(P); EKF5DOF.setPCorrected(P);
		gotHeading = true;
		firstDepthMsgReceived =true;
		if(gotHeading && gotGpsFix && firstDepthMsgReceived) {
		EKF5DOF.setFilteringPossible();
		}
	} 
} // Nav Msg

void navigationRos::handleROSlblMsg(const nav_msgs::OdometryConstPtr & msg)
{
		// how to handle LBL message- store it in temporary buffers
		//originNED was supposed to be set on first message
		//if(globalCoordConverter.isInitialised())
		//{
		//	LLD.lat = msg->global_position.latitude;
		//	LLD.lon = msg->global_position.longitude;
		//	LLD.depth = msg->position.depth;
		//	NED = globalCoordConverter.LLD2NED(LLD);
		//}
			
    		//north = northLBL = msg->pose.pose.position.x;
    		//east = eastLBL  = (-1) * msg->pose.pose.position.y;
    		/*
    		depth = msg->position.depth;
    		altitude       = msg->altitude;
    		surgeVelocity = msg->body_velocity.x;
    		swayVelocity = msg->body_velocity.y;
    		heaveVelocity = msg->body_velocity.z;
    		yaw = msg->orientation.yaw;
       	 	pitch = msg->orientation.pitch;
    		yawRate   = msg->orientation_rate.yaw;
    		pitchRate = msg->orientation_rate.pitch;
		*/
    	if(EKF5DOF.isFilteringPossible())
    	{
    		// it is not a first message for Kalman - had some before
    		// elapsed time
    		if(!gotLBL){
    		gotLBL = true;
    		}
    		
		//MeasurementVec.setSize(2, 0);//enable enough space
		//MeasurementStates.setSize(2, NULL_STATE);//enable enough space 
    		//sensorType sensorDevice = LBL_SENSOR;
    		//MeasurementVec[0] = northLBL;         MeasurementStates[0] = NORTH;
		//MeasurementVec[1] = eastLBL;          MeasurementStates[1] = EAST;
		/*
    		if (0)//if((EKF5DOF.ekfMode==EVERY_MESSAGE) || (EKF5DOF.ekfMode==EVERY_DT))//
    		{
    			double64 elapsedPeriod;
    			uint32_t elapsedMicroSeconds;
    			elapsedMicroSeconds = m_timer_ekf_MES->ReadAndReset();
    			elapsedPeriod = elapsedMicroSeconds * 0.000001;
			cout << "#------ ### LBL ROS MSG ### ------   "  << elapsedPeriod << endl;
						// for logging...
						olibVector<double64> previousState(SYSTEM_STATE_LEN,0);
						previousState = EKF5DOF.getXCorrected();
						olibMatrix<double64> previousP(SYSTEM_STATE_LEN,SYSTEM_STATE_LEN,0);
						previousP     = EKF5DOF.getPCorrected();
						////
			olibVector<double64> filteredValues(SYSTEM_STATE_LEN,0); // storage for filtered state
			filteredValues = EKF5DOF.filterMessage(MeasurementVec, sensorDevice, elapsedPeriod); 
						// for logging...
						olibMatrix<double64> filteredP(SYSTEM_STATE_LEN,SYSTEM_STATE_LEN,0);
						filteredP     = EKF5DOF.getPCorrected();
						////		
			storeFilteringResults(filteredValues, elapsedPeriod);
			storeLogFile(elapsedPeriod, previousState, previousP, MeasurementVec, MeasurementStates, filteredValues, filteredP);
    		}
    		*/	
		/*
		if(EKF5DOF.ekfMode==EVERY_DT)
		{//aim is to create an oservatio that will be periodically processed within timer defined interval
			addMeasurementToObservation(ObservationVec, ObservationStates, MeasurementVec, MeasurementStates);
		}
		*/
		
    	}
    	else
    	{// first msg
    		olibVector<double64> X(SYSTEM_STATE_LEN, 0);
    		X[NORTH_INDEX]     = north;
    		X[EAST_INDEX]      = east;
		northLBL = north;
		eastLBL  = east;		
		X[DEPTH_INDEX]     = depth;
		X[ALTITUDE_INDEX]  = altitude;
		X[SURGE_VEL_INDEX] = surgeVelocity;
		X[SWAY_VEL_INDEX]  = swayVelocity;
		X[HEAVE_VEL_INDEX] = heaveVelocity;
		X[YAW_INDEX]       = yaw;
		X[PITCH_INDEX]     = pitch;
		X[YAW_RATE_INDEX]  = yawRate;
		X[PITCH_RATE_INDEX]= pitchRate;
		
		//originNED.n = north;
		//originNED.e = east;
		//originNED.d = depth;
		//globalCoordConverter.setNEDReference(originNED);
		
		m_timer_ekf_MES->Reset();
				olibMatrix<double64> P(SYSTEM_STATE_LEN, SYSTEM_STATE_LEN, 0); // covariance matrix
				P = EKF5DOF.getPCorrected(); // take current
				P[NORTH_INDEX][NORTH_INDEX] = pow(EKF5DOF.SDnorth,2);   
				P[EAST_INDEX][EAST_INDEX] = pow(EKF5DOF.SDeast,2);
				EKF5DOF.setPPrediction(P); EKF5DOF.setPCorrected(P);

		EKF5DOF.setXPrediction(X); EKF5DOF.setXCorrected(X);
		gotGpsFix = true;

		if(gotHeading && gotGpsFix && firstDepthMsgReceived) {
		EKF5DOF.setFilteringPossible();
		}
	}
} // LBL Msg
// ---------------------------------------------------------------------------
void navigationRos::doWork()
{
	if (m_timer.getTimer() == 0)
	{
		m_timer.setTimer(sendMsgPeriod);//send message period
		sendROSNavSts();
		sendROSFiltOdometry();
		//cout << "got Heading: " << gotHeading << "  gotGpsFix: " << gotGpsFix << endl;
	} 
	
	if(EKF5DOF.ekfMode==EVERY_DT)
	{
	//m_timer_ekf_TRIGGER
	if(m_timer_ekf_TRIGGER.getTimer() == 0){
		m_timer_ekf_TRIGGER.setTimer(ekfPeriod);
		
		if(EKF5DOF.isFilteringPossible() && !((ObservationStates.size()==1) && (ObservationStates[0]==NULL_STATE))  )
		{
		double64 elapsedPeriod;
		uint32_t elapsedMicroSeconds;
		elapsedMicroSeconds = m_timer_ekf_MES->ReadAndReset();
		elapsedPeriod = elapsedMicroSeconds * 0.000001;
		cout << "#------ ### EVERY_DT TRIGGERED ### ------   "  << elapsedPeriod << endl;
						// for logging...
						olibVector<double64> previousState(SYSTEM_STATE_LEN,0);
						previousState = EKF5DOF.getXCorrected();
						olibMatrix<double64> previousP(SYSTEM_STATE_LEN,SYSTEM_STATE_LEN,0);
						previousP     = EKF5DOF.getPCorrected();
						////
		olibVector<double64> filteredValues(SYSTEM_STATE_LEN,0);
		//cout << "obs: " << ObservationVec << "obs state: " << ObservationStates << endl;
		filteredValues = EKF5DOF.filterObservation(ObservationVec, ObservationStates, elapsedPeriod); 
						// for logging...
						olibMatrix<double64> filteredP(SYSTEM_STATE_LEN,SYSTEM_STATE_LEN,0);
						filteredP     = EKF5DOF.getPCorrected();
						////		
		storeFilteringResults(filteredValues, elapsedPeriod);
		storeLogFile(elapsedPeriod, previousState, previousP, MeasurementVec, MeasurementStates, filteredValues, filteredP);
		//reset observation for the next round
		ObservationVec.setSize(1,0);    ObservationVec[0] = 0;
		ObservationStates.setSize(1,NULL_STATE); ObservationStates[0] = NULL_STATE;
		}	
	} 	
	}
}
//---------------------------------------------------------------------------
void navigationRos::sendROSNavSts() {
	// input: m_rosInputNavMsg.header.stamp -> output m_rosFiltNavMsg
	m_rosFiltNavMsg.header.stamp = ros::Time::now();
	// take the values stored during filtering() stage
	m_rosFiltNavMsg.position.north = northFiltered;
	m_rosFiltNavMsg.position.east  = eastFiltered;
	m_rosFiltNavMsg.position.depth = depthFiltered;
	m_rosFiltNavMsg.altitude       = altitudeFiltered;

	m_rosFiltNavMsg.orientation.pitch = pitchFiltered;
	m_rosFiltNavMsg.orientation.yaw   = yawFiltered;

	m_rosFiltNavMsg.body_velocity.x = surgeVelocityFiltered;
	m_rosFiltNavMsg.body_velocity.y = swayVelocityFiltered;
	m_rosFiltNavMsg.body_velocity.z = heaveVelocityFiltered;

	m_rosFiltNavMsg.orientation_rate.pitch = pitchRateFiltered;
	m_rosFiltNavMsg.orientation_rate.yaw   = yawRateFiltered;

	//m_rosNavMsg.mode = auv_msgs::NavSts::MODE_DEFAULT;
	m_rosFiltNavMsg.status = auv_msgs::NavSts::STATUS_ALL_OK;

	m_rosFiltNavPub.publish(m_rosFiltNavMsg); //Filt
}
//---------------------------------------------------------------------------
void navigationRos::sendROSFiltOdometry() {
	// This message isn't needed by the AUV, but allows us to use ROS rviz to
	// visualise the vehicle position after filtering.

	// Map from AUV to ROS coordinates.
	double rosX, rosY, rosZ, rosRoll, rosPitch, rosYaw;
	osl_core::mapAxisAUVtoROS(northFiltered, eastFiltered, depthFiltered, rosX, rosY, rosZ);
	osl_core::mapAxisAUVtoROS(osl_core::deg2rad(roll), osl_core::deg2rad(pitchFiltered), osl_core::deg2rad(yawFiltered),
			rosRoll, rosPitch, rosYaw); // roll without filtering (5dof)
	geometry_msgs::Quaternion rosOrientation = tf::createQuaternionMsgFromRollPitchYaw(rosRoll, rosPitch, rosYaw);

	m_rosFiltOdometryMsg.header.stamp = ros::Time::now();
	// TODO: Start sending odom as well as map transforms
	m_rosFiltOdometryMsg.header.frame_id = tfMapFrame_;
	m_rosFiltOdometryMsg.child_frame_id = tfBaseLinkFrame_;
	m_rosFiltOdometryMsg.pose.pose.position.x = rosX;
	m_rosFiltOdometryMsg.pose.pose.position.y = rosY;
	m_rosFiltOdometryMsg.pose.pose.position.z = rosZ;
	m_rosFiltOdometryMsg.pose.pose.orientation = rosOrientation;

	m_rosFiltOdometryPub.publish(m_rosFiltOdometryMsg);

	// Now create Transform msg.
	// Broadcaster needs to be static not member variable as it cannot be created before
	// ModuleCore has initialised ROS.
	static tf::TransformBroadcaster rosTFBroadcaster;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(rosX, rosY, rosZ));
	transform.setRotation(tf::createQuaternionFromRPY(rosRoll, rosPitch, rosYaw));
	rosTFBroadcaster.sendTransform(tf::StampedTransform(transform, m_rosFiltNavMsg.header.stamp,
					m_rosFiltOdometryMsg.header.frame_id, m_rosFiltOdometryMsg.child_frame_id));
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
	/*
		fprintf(pInputMessages,"%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
			northNAV, eastNAV, northLBL, eastLBL, northFiltered, eastFiltered, 
			surgeVelocity, surgeVelocityFiltered, yaw, yawFiltered, yawRate, pitch, pitchFiltered, elapsedPeriod
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
*/
	
} //store
//-------------------------------------------------------------------------------
void navigationRos::storeLogFile(double64 interval, olibVector<double64> previousState, olibMatrix<double64> previousP, olibVector<double64> Obser, olibVector<stateID> ObserStates, olibVector<double64> filteredState, olibMatrix<double64> filteredP) {
        // write to log files
	// time interval
	// store the old nav
	fprintf(pOldNav, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",north, east, depth, altitude, surgeVelocity,swayVelocity, heaveVelocity, 
	yaw,pitch, yawRate, pitchRate, northLBL, eastLBL);

	uint32 count;
        fprintf(pLogInput, "%f,", interval);
	for(count = 0; count<SYSTEM_STATE_LEN;count++) {
		fprintf(pLogInput,"%f,", previousState[count]);//%f, %f, %f, %f, %f, %f, %f, %f, %f, %f,     %f, %f, %f, %f, %f, %f\n",
        }
	for(count = 0; count<SYSTEM_STATE_LEN-1;count++) {
		fprintf(pLogInput,"%f,", previousP[count][count]);//%f, %f, %f, %f, %f, %f, %f, %f, %f, %f,     %f, %f, %f, %f, %f, %f\n",
        }        
        fprintf(pLogInput,"%f\n", previousP[SYSTEM_STATE_LEN-1][SYSTEM_STATE_LEN-1]);
	///////////////////////
	uint32 ObserLength;
	ObserLength = Obser.size();

	fprintf(pLogObser, "%i,", ObserLength);
        for(count = 0; count<ObserLength;count++) {
                fprintf(pLogObser," %f,", Obser[count]);//%f, %f, %f, %f, %f, %f, %f, %f, %f, %f,     %f, %f, %f, %f, %f, %f\n",
        }
        for(count = 0; count<ObserLength-1;count++) {
                fprintf(pLogObser," %i,", ObserStates[count]);//%f, %f, %f, %f, %f, %f, %f, %f, %f, %f,     %f, %f, %f, %f, %f, %f\n",
        }
        fprintf(pLogObser," %i\n", ObserStates[ObserLength-1]);
        ///////////////////////////
        for(count = 0; count<SYSTEM_STATE_LEN;count++) {
                fprintf(pLogOutput,"%f,", filteredState[count]);//%f, %f, %f, %f, %f, %f, %f, %f, %f, %f,     %f, %f, %f, %f, %f, %f\n",
        }
	for(count = 0; count<SYSTEM_STATE_LEN-1;count++) {
		fprintf(pLogOutput,"%f,", filteredP[count][count]);//%f, %f, %f, %f, %f, %f, %f, %f, %f, %f,     %f, %f, %f, %f, %f, %f\n",
        }        
        fprintf(pLogOutput,"%f\n", filteredP[SYSTEM_STATE_LEN-1][SYSTEM_STATE_LEN-1]);

	/*
	north,            east,             depth,            surgeVelocity,         swayVelocity,         yaw,
        northFiltered,    eastFiltered,     depthFiltered,    surgeVelocityFiltered, swayVelocityFiltered, yawFiltered,
        CorrectedP[NORTH_INDEX][NORTH_INDEX],
                          CorrectedP[EAST_INDEX][EAST_INDEX],
                                            CorrectedP[DEPTH_INDEX][DEPTH_INDEX],
                                                              CorrectedP[SURGE_VEL_INDEX][SURGE_VEL_INDEX],
                                                                                      CorrectedP[SWAY_VEL_INDEX][SWAY_VEL_INDEX],
                                                                                                            CorrectedP[YAW_INDEX][YAW_INDEX]);
        */
}
//---------------------------------------------------------------------------
void navigationRos::cleanup() {

	PRINTLN_INFO("cleanup entered");
	delete m_timer_ekf_MES;

}

// ---------------------------------------------------------------------------
int main(int argc, char * argv[]) {
	time_t rawtime;
	struct tm * timeinfo;
	time ( &rawtime );
	timeinfo = localtime ( &rawtime );
	char *timeString;
	timeString = asctime (timeinfo);
	//char *fileNameInput;
	//char *fileNameOutput;
	//char *fileNameObser;	
	//char *fileNameGps;
	char *pattern;
	pattern = removeWhiteSpaces(timeString);
	//fileNameOutput = strcat(removeWhiteSpaces(timeString), ".output");
	//fileNameObser = removeWhiteSpaces(timeString);
	//fileNameGps = removeWhiteSpaces(timeString);
	//strcat (fileNameInput, ".input");
	//strcat (fileNameOutput,".output");
	//strcat (fileNameObser, ".obser");
	//strcat (fileNameGps,   ".gps");

	pLogInput  = fopen ("logFile.input",  "w");
	pLogOutput = fopen ("logFile.output", "w");
	pLogObser  = fopen ("logFile.obser",  "w");
	pLogGps    = fopen ("logFile.gps",    "w");
	pLogParams    = fopen ("logFile.params",    "w");
	pOldNav    = fopen("logFile.old",    "w");
	navigationRos modCore;
	cout << "class created..." << endl;
		//pInputMessages = fopen ("inputMessages.csv" , "w");
		//pPredictedState = fopen("predictedState.csv", "w");
		//pCorrectedState = fopen("correctedState.csv", "w");
		//pPredictedP     = fopen("predictedP.csv", "w");
		//pCorrectedP     = fopen("correctedP.csv", "w");
		//pObservation    = fopen("Observation.csv", "w");
		//pPredictedObservation = fopen("predObservation.csv", "w");
		//fprintf(debugFile, "\"innovationNorth\"\n");
	return modCore.main(argc, argv);
	fclose (pLogParams);
	fclose (pOldNav);
	fclose (pLogInput);
	fclose (pLogOutput);
	fclose (pLogObser);
	fclose (pLogGps);
}
