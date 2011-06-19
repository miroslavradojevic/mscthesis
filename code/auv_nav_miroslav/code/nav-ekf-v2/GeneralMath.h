/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Heriot-Watt University, UK.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Heriot-Watt University nor the names of 
*     its contributors may be used to endorse or promote products 
*     derived from this software without specific prior written
*     permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*  Author: Pedro Patron, Joel Cartwright
*
*********************************************************************/

#ifndef _OSLCORE_GENERALMATH_H_
#define _OSLCORE_GENERALMATH_H_

// ================================================================== includes

#ifdef WIN32
	#define _USE_MATH_DEFINES
#endif
#include <math.h>

// ========================================================== external defines

namespace osl_core
{

#ifndef M_PI
#define M_PI    3.141592653589793
#endif

#ifndef SQRT_2
#define SQRT_2  1.414213562373095
#endif

#ifndef SQRT_3
#define SQRT_3  1.732050807568877
#endif

#ifndef RAD_PER_DEG
#define RAD_PER_DEG (M_PI/180.0)
#endif

#ifndef DEG_PER_RAD
#define DEG_PER_RAD (180.0/M_PI)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x)   ((x) * (((double)180.0) / M_PI))
#endif

#ifndef DEG2RAD
#define DEG2RAD(x)   ((x) * (((double)M_PI) / 180.0))
#endif

//-------------------------------------------------------------------------
inline double rad2deg(double radians)
{
	return radians * ((double)180.0 / M_PI);
}

//-------------------------------------------------------------------------
inline double deg2rad(double degrees)
{
	return degrees * ((double)M_PI / 180.0);
}

//-------------------------------------------------------------------------
// Wraps input angle to the range [0, 360)
inline double wrap0to360 (const double input)
{
	return input - 360.0 * floor(input / 360.0);
}

//-------------------------------------------------------------------------
// Wraps input angle to the range [0, 180)
inline double wrap0to180 (const double input)
{
	return input - 180.0 * floor(input / 180.0);
}

//-------------------------------------------------------------------------
// Wraps input angle to the range [0, 90)
inline double wrap0to90 (const double input)
{
	return input - 90.0 * floor(input / 90.0);
}

//-------------------------------------------------------------------------
// Wraps input angle to the range (-180, 180]
inline double wrapPlusMinus180 (const double input)
{
	double angle = wrap0to360(input);

	if (angle > 180.0) angle -= 360.0;

	return angle;
}

//-------------------------------------------------------------------------
// Wraps input angle to the range (-90, 90]
inline double wrapPlusMinus90 (const double input)
{
	double angle = wrap0to180(input);

	if (angle > 90.0) angle -= 180.0;

	return angle;
}

//-------------------------------------------------------------------------
// Wraps input angle in RADIANS to the range [0, 2PI)
inline double wrap0to2PI (const double input)
{
	return input - (2*M_PI) * floor(input / (2*M_PI));
}

//-------------------------------------------------------------------------
// Wraps input angle in RADIANS to the range [0, PI)
inline double wrap0toPI (const double input)
{
	return input - M_PI * floor(input / M_PI);
}

//-------------------------------------------------------------------------
// Wraps input angle in RADIANS to the range [0, PI/2)
inline double wrap0toPIhalf (const double input)
{
	return input - (M_PI/2) * floor(input / (M_PI/2));
}

//-------------------------------------------------------------------------
// Wraps input angle in RADIANS to the range (-PI, PI]
inline double wrapPlusMinusPI (const double input)
{
	double angle = wrap0to2PI(input);

	if (angle > M_PI) angle -= 2*M_PI;

	return angle;
}

//-------------------------------------------------------------------------
// Wraps input angle in RADIANS to the range (-PI/2, PI/2]
inline double wrapPlusMinusPIhalf (const double input)
{
	double angle = wrap0toPI(input);

	if (angle > (M_PI/2)) angle -= M_PI;

	return angle;
}

//-------------------------------------------------------------------------
//-------------------------------------------------------------------------

//-------------------------------------------------------------------------
inline double limitPlusMinus(double value, double limit)
{
	if (value < -limit) value = -limit;
	else if (value > limit) value = limit;

	return value;
}

//-------------------------------------------------------------------------
inline double limitPositive(double value, double limit)
{
	if (value < 0) value = 0;
	else if (value > limit) value = limit;

	return value;
}

//-------------------------------------------------------------------------
// Maps AUV axis values to ROS axis values (doubles)
inline void mapAxisAUVtoROS (const double auvX, const double auvY, const double auvZ,
		double & rosX, double & rosY, double & rosZ)
{
	rosX = auvX;
	rosY = -auvY;
	rosZ = -auvZ;
}

//-------------------------------------------------------------------------
// Maps AUV axis values to ROS axis values (floats)
inline void mapAxisAUVtoROS (const float auvX, const float auvY, const float auvZ,
		float & rosX, float & rosY, float & rosZ)
{
	rosX = auvX;
	rosY = -auvY;
	rosZ = -auvZ;
}

//-------------------------------------------------------------------------
// Maps ROS axis values to AUV axis values (doubles)
inline void mapAxisROStoAUV (const double rosX, const double rosY, const double rosZ,
		double & auvX, double & auvY, double & auvZ)
{
	auvX = rosX;
	auvY = -rosY;
	auvZ = -rosZ;
}

//-------------------------------------------------------------------------
// Maps ROS axis values to AUV axis values (floats)
inline void mapAxisROStoAUV (const float rosX, const float rosY, const float rosZ,
		float & auvX, float & auvY, float & auvZ)
{
	auvX = rosX;
	auvY = -rosY;
	auvZ = -rosZ;
}

} // namespace

#endif
