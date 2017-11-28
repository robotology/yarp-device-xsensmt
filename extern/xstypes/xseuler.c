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

#include "xseuler.h"
#include "xsquaternion.h"
#include <math.h>

/*! \class XsEuler
	\brief Contains Euler Angle data and conversion from Quaternion
	\details Euler Angles are computed as a rotation around the x-axis, followed by a rotation
	around the y-axis, followed by a rotation around the z-axis.
	In aerospace terms, x,y and z are also often referred to as roll pitch and yaw, so those names
	are also available as a convenience.
*/

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \relates XsEuler
	\brief Clears all angles in the XsEuler object by setting them to 0
*/
void XsEuler_destruct(XsEuler* thisPtr)
{
	thisPtr->m_x = XsMath_zero;
	thisPtr->m_y = XsMath_zero;
	thisPtr->m_z = XsMath_zero;
}

/*! \relates XsEuler
	\brief Returns true if all angles in this object are zero
*/
int XsEuler_empty(const XsEuler* thisPtr)
{
	return thisPtr->m_x == XsMath_zero && thisPtr->m_y == XsMath_zero && thisPtr->m_z == XsMath_zero;
}

/*! \relates XsEuler
	\brief Get an euler angle representation of the quaternion.
*/
void XsEuler_fromQuaternion(XsEuler* thisPtr, const XsQuaternion* quat)
{
	XsReal sqw, dphi, dpsi;

	if (XsQuaternion_empty(quat))
	{
		XsEuler_destruct(thisPtr);
		return;
	}

	sqw = quat->m_w * quat->m_w;
	dphi = XsMath_two * (sqw + quat->m_z * quat->m_z) - XsMath_one;
	dpsi = XsMath_two * (sqw + quat->m_x * quat->m_x) - XsMath_one;

	thisPtr->m_x =  XsMath_rad2deg(atan2(XsMath_two*(quat->m_y*quat->m_z + quat->m_w*quat->m_x), dphi));
	thisPtr->m_y = -XsMath_rad2deg(XsMath_asinClamped(XsMath_two*(quat->m_x*quat->m_z - quat->m_w*quat->m_y)));
	thisPtr->m_z =  XsMath_rad2deg(atan2(XsMath_two*(quat->m_x*quat->m_y + quat->m_w*quat->m_z), dpsi));
}

/*! @} */
