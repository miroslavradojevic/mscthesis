#include <OceanLIB/Abstraction/olibVector.h>
#include <OceanLIB/Abstraction/olibMatrix.h>

#include <OceanLIB/Processing/olibKalmanFilter.h>
#include <OceanLIB/Processing/olibKalmanMeasure.h>
#include <OceanLIB/Processing/olibRTS.h>

#include <osl_core/Chrono.h>
#include "filt.h"
int main(int argc, char **argv)
{
	extendedKalmanFilter5DOF EXAMPLE;
	filterMode mode = EVERY_DT;
	EXAMPLE.init(mode);
	
	olibVector<double64> Q(1, 0);
	Q[0] = 6; //Q[3] = EAST;
	cout << Q << endl;
	//stateID dummy=NULL_STATE;
	Q.setSize(1, 0);
	cout << Q << endl;
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
	return 0;
}
