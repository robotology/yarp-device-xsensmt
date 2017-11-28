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

#include "xssyncsettingarray.h"
#include "xssyncsetting.h"
#include <memory.h>
#include <string.h>

/*! \struct XsSyncSettingArray
	\brief A list of XsSyncSetting values
	\sa XsArray
*/

/*! \copydoc XsArrayDescriptor::itemCopy
	\note Specialization for XsSyncSetting*/
void copySyncSetting(XsSyncSetting* to, XsSyncSetting const* from)
{
	*to = *from;
}

/*! \copydoc XsArrayDescriptor::itemCompare
	\note Specialization for XsSyncSetting*/
int compareSyncSetting(XsSyncSetting const* a, XsSyncSetting const* b)
{
	return XsSyncSetting_compare(a, b);
}

//! \brief zero the pointer value
void zeroSyncSetting(XsSyncSetting* a)
{
	memset(a, 0, sizeof(XsSyncSetting));
	a->m_line = XSL_Invalid;
}

//! \brief Descriptor for XsSyncSettingArray
XsArrayDescriptor const g_xsSyncSettingArrayDescriptor = {
	//lint --e{64} ignore exact type mismatches here
	sizeof(XsSyncSetting),
	XSEXPCASTITEMSWAP XsSyncSetting_swap,
	XSEXPCASTITEMMAKE zeroSyncSetting,		// construct
	XSEXPCASTITEMCOPY copySyncSetting,		// copy construct
	0,										// destruct
	XSEXPCASTITEMCOPY copySyncSetting,
	XSEXPCASTITEMCOMP compareSyncSetting,
	XSEXPCASTRAWCOPY XsArray_rawCopy	// raw copy
};

/*! \copydoc XsArray_construct
	\note Specialization for XsSyncSettingArray
*/
void XsSyncSettingArray_construct(XsSyncSettingArray* thisPtr, XsSize count, XsSyncSetting const* src)
{
	XsArray_construct(thisPtr, &g_xsSyncSettingArrayDescriptor, count, src);
}
