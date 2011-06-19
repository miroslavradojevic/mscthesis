#include <vector>

#if 0
#include <OceanLIB/Abstraction/olibVector.h>
#include <OceanLIB/Abstraction/olibMatrix.h>

#include <OceanLIB/Processing/olibKalmanFilter.h>
#include <OceanLIB/Processing/olibKalmanMeasure.h>
#include <OceanLIB/Processing/olibRTS.h>

#include <osl_core/Chrono.h>
#include <osl_core/ModuleCore.h>
#include <osl_core/GlobalCoordConverter.h>
#include <osl_core/TimerMillis.h>


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

#include <ros/ros.h>
#endif // if 0

#include <time.h>
#include <string.h>
//#include "filt.h"
#include "filtUKF.h"
//#include <TooN/TooN.h>
//using namespace TooN;
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

int main(int argc, char **argv)
{


	
	olibMatrix<double64> P(3,3,0);
	olibVector<double64> x(3,0);
	x[0] = 4.5; x[1] = 8.9; x[2] = 9.32;
	P[0][0] = 4.5; P[1][1] = 1.5; P[2][2] = 0.5;
	cout << "x   : \n" << x << endl;
	cout << "P   : \n" << P << endl;
	olibMatrix<double64> x_samples(1,1,0);
	olibVector<double64> x_weights(1,0);
        unscentedTransformSampling(x,P,1.0,x_samples,x_weights);
        cout << "samples: \n" << x_samples << endl;
        cout << "weights: \n" << x_weights << endl;
	
	/*
	olibMatrix<double64> S(8,8,0);
	S[0][0] = 0.000122493; S[0][1] = -1.72297e-05; S[0][2] = 0; S[0][3] = 0; S[0][4] = 8.38684e-06; S[0][5] = 0; S[0][6] = 3.19031e-09; S[0][7] = 1.85808e-09;
	S[1][0] = -1.72297e-05; S[1][1] = 0.0105435; S[1][2] = 0; S[1][3] = 0; S[1][4] = -8.38684e-06; S[1][5] = 0; S[1][6] = -3.19031e-09; S[1][7] = -1.85808e-09;
	S[2][0] = 0; S[2][1] = 0; S[2][2] = 0; S[2][3] = 1.05263e-10; S[2][4] = 0; S[2][5] = 0; S[2][6] = 0; S[2][7] = 0;
	S[3][0] = 0; S[3][1] = 0; S[3][2] = 0; S[3][3] = 1.05263e-10; S[3][4] = 0; S[3][5] = 0; S[3][6] = 0; S[3][7] = 0;
	S[4][0] = 8.38684e-06; S[4][1] = -8.38684e-06; S[4][2] = 0; S[4][3] = 0; S[4][4] = 0.0100041; S[4][5] = 0; S[4][6] = -3.12509e-09; S[4][7] = 2.07398e-10;
	S[5][0] = 0; S[5][1] = 0; S[5][2] = 0; S[5][3] = 0; S[5][4] = 0; S[5][5] = 0.000122788; S[5][6] = 0; S[5][7] = 0;
	S[6][0] = 3.19031e-09; S[6][1] = -3.19031e-09; S[6][2] = 0; S[6][3] = 0; S[6][4] = -3.12509e-09; S[6][5] = 0; S[6][6] = 0.000115085; S[6][7] = 4.76676e-06;
	S[7][0] = 1.85808e-09; S[7][1] = -1.85808e-09; S[7][2] = 0; S[7][3] = 0; S[7][4] = 2.07398e-10; S[7][5] = 0; S[7][6] = 4.76676e-06; S[7][7] = 0.000102313;
	cout << "S: \n" << S << endl;
	olibMatrix<double64> Sinv(8,8,0);
	Sinv = inv(S);
	cout << "S(-1): \n" << Sinv << endl;
	
	FILE *pFILE;
	
	time_t rawtime;
  	struct tm * timeinfo;
  	time ( &rawtime );
	timeinfo = localtime ( &rawtime );
	char *date, *folderName;//, *path;
	date = asctime (timeinfo);
	folderName = removeWhiteSpaces(date);
	printf ( "current folder name is: %s\n", date );
	
	//path = "";
	strcat (folderName,".csv");
	printf ( "current folder without spaces is: %s\n", folderName );
	pFILE = fopen (folderName , "w");
	fprintf(pFILE, "%f\n", 3.7);
	fclose (pFILE);
	*/
	/*
	ros::init(argc, argv, "test_node");
	ros::NodeHandle nh;
	// try the time
	uint32_t current_time;
	osl_core::Chrono *timer;
	timer = new osl_core::Chrono();
	timer->Reset();
	while(current_time<=1000)
	{
	current_time = timer->ReadMs();
	cout << "Time is: " << current_time << endl;
	}
	
		delete timer;
	
	extendedKalmanFilter5DOF EXAMPLE;
	
	//char mode;
	//mode = *argv[argc-1];
	//cout << "mode: " << mode << endl;
	filterMode modeEKF=EVERY_MESSAGE;
	cout << "ekf mode default: " <<  modeEKF << endl;
	switch(*argv[argc-1])
	{
	case '1' :
	{
		
		modeEKF = EVERY_DT;
		cout << "ekf mode: " <<  modeEKF << endl;
		break;
	}
	
	case '2' :
	{
		modeEKF = EVERY_MESSAGE;
		break;
	}
	
	default :
	{
	cout << "dflt..." << endl;// nothing
	}		
	}
		
	//cout << mode << endl;
	
	//modeEKF = mode;
	cout << "ekf mode after: " << modeEKF << endl;
	EXAMPLE.init(modeEKF);
	*/
	
	
														
														//olibVector<double64> Q(3,2);
														//Q[0] = 6; Q[2] = 99;
														//cout << Q << endl;
														//stateID dummy=NULL_STATE;
														//Q.setSize(1,0);
														//cout << Q << endl;
														/*
														if (argc != 6)
														{
															cout << "wrong" << endl;
															return 0;
														}
														cout << "argc: " << argc << endl;
														cout << "argc-1 " << argv[argc-1] << endl;
														cout << "argc-2 " << argv[argc-2] << endl;
														cout << "argc-3 " << argv[argc-3] << endl;
														cout << "argc-4 " << argv[argc-4] << endl;
	
														int w=999;
														cout << w << endl;
														*/
	 	
	return 0;
}
