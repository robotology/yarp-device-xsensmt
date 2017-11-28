/*	WARNING: COPYRIGHT (C) 2017 XSENS TECHNOLOGIES OR SUBSIDIARIES WORLDWIDE. ALL RIGHTS RESERVED.
	THIS FILE AND THE SOURCE CODE IT CONTAINS (AND/OR THE BINARY CODE FILES FOUND IN THE SAME
	FOLDER THAT CONTAINS THIS FILE) AND ALL RELATED SOFTWARE (COLLECTIVELY, "CODE") ARE SUBJECT
	TO A RESTRICTED LICENSE AGREEMENT ("AGREEMENT") BETWEEN XSENS AS LICENSOR AND THE AUTHORIZED
	LICENSEE UNDER THE AGREEMENT. THE CODE MUST BE USED SOLELY WITH XSENS PRODUCTS INCORPORATED
	INTO LICENSEE PRODUCTS IN ACCORDANCE WITH THE AGREEMENT. ANY USE, MODIFICATION, COPYING OR
	DISTRIBUTION OF THE CODE IS STRICTLY PROHIBITED UNLESS EXPRESSLY AUTHORIZED BY THE AGREEMENT.
	IF YOU ARE NOT AN AUTHORIZED USER OF THE CODE IN ACCORDANCE WITH THE AGREEMENT, YOU MUST STOP
	USING OR VIEWING THE CODE NOW, REMOVE ANY COPIES OF THE CODE FROM YOUR COMPUTER AND NOTIFY
	XSENS IMMEDIATELY BY EMAIL TO INFO@XSENS.COM. ANY COPIES OR DERIVATIVES OF THE CODE (IN WHOLE
	OR IN PART) IN SOURCE CODE FORM THAT ARE PERMITTED BY THE AGREEMENT MUST RETAIN THE ABOVE
	COPYRIGHT NOTICE AND THIS PARAGRAPH IN ITS ENTIRETY, AS REQUIRED BY THE AGREEMENT.
*/

#include "xsmath.h"
#include <math.h>
#include <float.h>

/*! \namespace XsMath
	\brief Namespace for mathematical constants and operations.
*/
/*! \addtogroup cinterface C Interface
	@{
*/

//! \brief The value e
const XsReal XsMath_e = 2.7182818284590452353602874713527;
//! \brief The value pi
const XsReal XsMath_pi = 3.1415926535897932384626433832795028841971693993751058209749;
//! \brief A really small value
const XsReal XsMath_tinyValue = 1.0e-16;
//! \brief A convincingly large number
const XsReal XsMath_hugeValue = 1.0e+16;

//! \brief A value related to the precisson of floating point arithmetic (2.2204460492503131e-016)
const XsReal XsMath_epsilon = 2.2204460492503131e-016;
//! \brief Square root of XsMath_epsilon
const XsReal XsMath_sqrtEpsilon = 1.4901161193847656e-008;

#ifdef XSENS_SINGLE_PRECISION
	//! \brief Value that represents the subnormal number in floating point wizardry
	const XsReal XsMath_denormalized = 1e-37;
	//! \brief Square root of XsMath_denormalized
	const XsReal XsMath_sqrtDenormalized = 3.1622776601683793319988935444327e-19;
#else
	//! \brief Value that represents the subnormal number in floating point wizardry
	const XsReal XsMath_denormalized = 1e-307;
	//! \brief Square root of XsMath_denormalized
	const XsReal XsMath_sqrtDenormalized = 3.1622776601683793319988935444327e-154;
#endif

//! \brief Value to convert radians to degrees by multiplication
const XsReal XsMath_rad2degValue = 57.295779513082320876798154814105;	// (180.0/pi)
//! \brief Value to convert degrees to radians by multiplication
const XsReal XsMath_deg2radValue = 0.017453292519943295769236907684886;	// (pi/180.0)

//! \brief 0
const XsReal XsMath_zero = 0.0;
//! \brief 0.25
const XsReal XsMath_pt25 = 0.25;
//! \brief 0.5
const XsReal XsMath_pt5 = 0.5;
//! \brief -0.5
const XsReal XsMath_minusPt5 = -0.5;
//! \brief 1.0
const XsReal XsMath_one = 1.0;
//! \brief -1.0
const XsReal XsMath_minusOne = -1.0;
//! \brief 2
const XsReal XsMath_two = 2.0;
//! \brief 4
const XsReal XsMath_four = 4.0;
//! \brief -2
const XsReal XsMath_minusTwo = -2.0;

//! \brief -pi/2
const XsReal XsMath_minusHalfPi = -1.5707963267948966192313216916397514420985846996875529104874;
//! \brief pi/2
const XsReal XsMath_halfPi = 1.5707963267948966192313216916397514420985846996875529104874;
//! \brief 2*pi
const XsReal XsMath_twoPi = 6.2831853071795864769252867665590057683943387987502116419498;
//! \brief sqrt(2)
const XsReal XsMath_sqrt2 = 1.4142135623730950488016887242097;
//! \brief sqrt(0.5)
const XsReal XsMath_sqrtHalf = 0.5*1.4142135623730950488016887242097;

#ifdef XSENS_SINGLE_PRECISION
//! \brief infinity value
const XsReal XsMath_infinity = FLT_MAX;
#else
//! \brief infinity value
const XsReal XsMath_infinity = DBL_MAX;
#endif

/*! \brief Returns asin(\a x) for -1 < x < 1
*/
XsReal XsMath_asinClamped(XsReal x)
{
	if (x <= XsMath_minusOne)
		return XsMath_minusHalfPi;

	if (x >= XsMath_one)
		return XsMath_halfPi;

	return asin(x);
}

/*!	\brief Convert radians to degrees
*/
XsReal XsMath_rad2deg(XsReal radians)
{
	return XsMath_rad2degValue * radians;
}

/*!	\brief Convert degrees to radians
*/
XsReal XsMath_deg2rad(XsReal degrees)
{
	return XsMath_deg2radValue * degrees;
}

/*!	\brief Returns \a a to the power of 2
*/
XsReal XsMath_pow2(XsReal a)
{
	return a*a;
}

/*!	\brief Returns \a a to the power of 3
*/
XsReal XsMath_pow3(XsReal a)
{
	return a*a*a;
}

/*! \brief Returns non-zero if \a x is finite
*/
int XsMath_isFinite(XsReal x)
{
#ifdef _MSC_VER
	switch (_fpclass(x))
	{
	case _FPCLASS_SNAN:
	case _FPCLASS_QNAN:
	case _FPCLASS_NINF:
	case _FPCLASS_PINF:
		return 0;

	case _FPCLASS_NN:
	case _FPCLASS_ND:
	case _FPCLASS_NZ:
	case _FPCLASS_PZ:
	case _FPCLASS_PD:
	case _FPCLASS_PN:
		return 1;
	}
	return _finite(x);
#elif __GNUC__
	return isfinite(x);
#elif defined(isfinite)
	return isfinite(x);
#elif defined(_ADI_COMPILER)
	return !(isnan(x) || isinf(x));
#else
	return 1;
#endif
}

/*! \brief Returns \a d integer converted from a double precision floating point value
*/
int32_t XsMath_doubleToLong(double d)
{
	if (d >= 0)
		return (int32_t) floor(d+0.5);
	else
		return (int32_t) ceil(d-0.5);
}

/*! \brief Returns \a d integer converted from a double precision floating point value
*/
int64_t XsMath_doubleToInt64(double d)
{
	if (d >= 0)
		return (int64_t) floor(d+0.5);
	else
		return (int64_t) ceil(d-0.5);
}

/*! @} */
