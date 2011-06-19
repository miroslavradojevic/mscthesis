#include "filtUKF.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <cblas.h>

using namespace std;
/////// auxilliary functions used for square root calculation

static int dsyevr(char JOBZ, char RANGE, char UPLO, int N,
       double *A, int LDA, double VL, double VU,
       int IL, int IU, double ABSTOL, int *M,
       double *W, double *Z, int LDZ, int *ISUPPZ,
       double *WORK, int LWORK, int *IWORK, int LIWORK) {
  extern void dsyevr_(char *JOBZp, char *RANGEp, char *UPLOp, int *Np,
                      double *A, int *LDAp, double *VLp, double *VUp,
                      int *ILp, int *IUp, double *ABSTOLp, int *Mp,
                      double *W, double *Z, int *LDZp, int *ISUPPZ,
                      double *WORK, int *LWORKp, int *IWORK, int *LIWORKp,
                      int *INFOp);
  int INFO;
  dsyevr_(&JOBZ, &RANGE, &UPLO, &N, A, &LDA, &VL, &VU,
          &IL, &IU, &ABSTOL, M, W, Z, &LDZ, ISUPPZ,
          WORK, &LWORK, IWORK, &LIWORK, &INFO);
  return INFO;
}

static double dlamch(char CMACH) {
  int N = 1;
  float X = 0;
  int incX = 1;
  cblas_snrm2(N, &X, incX);
  
  extern double dlamch_(char *CMACHp);
  return dlamch_(&CMACH);
}

/// matrix square root function

void matrixRoot(double *inputMatrix, int inputSize, double *outputMatrix){
 	double *A, *W, *Z, *WORK;
	int *ISUPPZ, *IWORK; 
	int  M;

	// allocate and initialize dynamically alloc matrix A with inputMatrix
	A = new double[inputSize*inputSize];
  	for (int i=0; i<inputSize*inputSize; ++i) {
       		A[i]  = inputMatrix[i];
  	}	
	//allocate space for the output parameters and workspace arrays
	W = new double[inputSize];
  	Z = new double[inputSize*inputSize];
  	ISUPPZ = new int [2*inputSize];
  	WORK = new double[26*inputSize];
  	IWORK = new int[10*inputSize];
  	//get the eigenvalues and eigenvectors
  	static const int N = inputSize;
  	dsyevr('V', 'A', 'L', N, A, N, 0, 0, 0, 0, dlamch('S'), &M, W, Z, N, ISUPPZ, WORK, 26*N, IWORK, 10*N);


  	for (int i=0; i<inputSize*inputSize; ++i) {
      		outputMatrix[i] = A[i];
  	}

  	delete[] A;
  	delete[] W;
  	delete[] Z;
  	delete[] ISUPPZ;
  	delete[] WORK;
  	delete[] IWORK;
}//matrixRoot

//////////////// UNSCENTED TRANSFORM FUNCTIONS DEFINITIONS ///////////////////////////
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
		// temporary solution since root does not exist in OceanLIB library
		cout << "matrix : \n" << P << endl;
		static const int N = x_len*x_len;
		double dummy[N];
		double dummyRoot[N];
		for(uint32 row=0; row<x_len; row++){
			for(uint32 col=0; col<x_len; col++){
				dummy[row+col*x_len]=P[row][col];
			}
		}
		
		matrixRoot(dummy, x_len, dummyRoot);
		
		for(uint32 row=0; row<x_len; row++){
			for(uint32 col=0; col<x_len; col++){
				P[row][col]=dummyRoot[row+col*x_len];
			}
		}		
		//cout << dummy[0] << " " << dummy[1] << " " << dummy[2] << " " << dummy[3] << dummy[4] <<  dummy[5] << endl;
		
		
		cout << "matrix root: \n" << P << endl;
		/////////////////////////////////////////////
	for (uint32 i=1; i<=x_len; i++){
		x_samples[i] = x_mean;
	}
	for (uint32 i=x_len+1; i<=2*x_len; i++){
		x_samples[i] = x_mean;
	}
}

/* unscented Kalman filter 5dof class definition */

// class unscentedKalmanFilter function definitions






