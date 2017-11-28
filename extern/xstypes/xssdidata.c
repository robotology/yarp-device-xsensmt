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

#include "xssdidata.h"

/*! \class XsSdiData
	\brief Contains StrapDown Integration (SDI) data.
	\details SDI data consists of a rotation and an acceleration, expressed as an orientation increment
	(also known as deltaQ) and a velocity increment (also known as deltaV).
*/

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \relates XsSdiData
	\brief Initialize an %XsSdiData object with the optional arguments.
	\param orientationIncrement The orientation increment to initialize the object with, may be 0
	\param velocityIncrement The velocity increment to initialize the object with, may be 0
*/
void XsSdiData_construct(XsSdiData* thisPtr, const XsReal* orientationIncrement, const XsReal* velocityIncrement)
{
	if (orientationIncrement)
	{
		thisPtr->m_orientationIncrement.m_data[0] = orientationIncrement[0];
		thisPtr->m_orientationIncrement.m_data[1] = orientationIncrement[1];
		thisPtr->m_orientationIncrement.m_data[2] = orientationIncrement[2];
		thisPtr->m_orientationIncrement.m_data[3] = orientationIncrement[3];
	}
	else
		XsQuaternion_destruct(&thisPtr->m_orientationIncrement);

	if (velocityIncrement)
		XsVector3_assign(&thisPtr->m_velocityIncrement, velocityIncrement);
	else
		XsVector3_destruct(&thisPtr->m_velocityIncrement);
}

/*! \relates XsSdiData
	\brief Destruct the object, makes the fields invalid
*/
void XsSdiData_destruct(XsSdiData* thisPtr)
{
	XsQuaternion_destruct(&thisPtr->m_orientationIncrement);
	XsVector3_destruct(&thisPtr->m_velocityIncrement);
}

/*! @} */ 
