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

#include "xsdeviceidarray.h"
#include "xsdeviceid.h"

/*! \struct XsDeviceIdArray
	\brief A list of XsDeviceId values
	\sa XsArray
*/

void initDeviceId(XsDeviceId* did)
{
	did->m_deviceId = 0;
}

void initDeviceIdToValue(XsDeviceId* did, XsDeviceId const* src)
{
	did->m_deviceId = src->m_deviceId;
}

int compareDeviceIds(XsDeviceId const* a, XsDeviceId const* b)
{
	return ((int) a->m_deviceId) - ((int) b->m_deviceId);
}

//! \brief Descriptor for XsDeviceIdArray
XsArrayDescriptor const g_xsDeviceIdArrayDescriptor = {
	//lint --e{64} ignore exact type mismatches here
	sizeof(XsDeviceId),
	XSEXPCASTITEMSWAP XsDeviceId_swap,		// swap
	XSEXPCASTITEMMAKE initDeviceId,			// construct
	XSEXPCASTITEMCOPY initDeviceIdToValue,	// copy construct
	0,										// destruct
	XSEXPCASTITEMCOPY initDeviceIdToValue,	// copy
	XSEXPCASTITEMCOMP compareDeviceIds,		// compare
	XSEXPCASTRAWCOPY XsArray_rawCopy		// raw copy
};

/*! \copydoc XsArray_construct
	\note Specialization for XsStringArray
*/
void XsDeviceIdArray_construct(XsDeviceIdArray* thisPtr, XsSize count, XsDeviceId const* src)
{
	XsArray_construct(thisPtr, &g_xsDeviceIdArrayDescriptor, count, src);
}
