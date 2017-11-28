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

#include "xsoutputconfigurationarray.h"
#include "xsoutputconfiguration.h"

/*! \struct XsOutputConfigurationArray
	\brief A list of XsOutputConfiguration values
	\sa XsArray
*/

/*! \copydoc XsArrayDescriptor::itemSwap
	\note Specialization for XsOutputConfiguration*/
void swapXsOutputConfiguration(XsOutputConfiguration* a, XsOutputConfiguration* b)
{
	XsOutputConfiguration tmp = *a;
	*a = *b;
	*b = tmp;
}

/*! \copydoc XsArrayDescriptor::itemCopy
	\note Specialization for XsOutputConfiguration*/
void copyXsOutputConfiguration(XsOutputConfiguration* to, XsOutputConfiguration const* from)
{
	*to = *from;
}

/*! \copydoc XsArrayDescriptor::itemCompare
	\note Specialization for XsOutputConfiguration*/
int compareXsOutputConfiguration(XsOutputConfiguration const* a, XsOutputConfiguration const* b)
{
	if (a->m_dataIdentifier != b->m_dataIdentifier || a->m_frequency != b->m_frequency)
	{
		if (a->m_dataIdentifier == b->m_dataIdentifier)
			return (a->m_frequency < b->m_frequency) ? -1 : 1;
		return (a->m_dataIdentifier < b->m_dataIdentifier) ? -1 : 1;
	}

	return 0;
}


//! \brief Descriptor for XsOutputConfigurationArray
XsArrayDescriptor const g_xsOutputConfigurationArrayDescriptor = {
	//lint --e{64} ignore exact type mismatches here
	sizeof(XsOutputConfiguration),
	XSEXPCASTITEMSWAP swapXsOutputConfiguration,	// swap
	0,												// construct
	XSEXPCASTITEMCOPY copyXsOutputConfiguration,	// copy construct
	0,												// destruct
	XSEXPCASTITEMCOPY copyXsOutputConfiguration,	// copy
	XSEXPCASTITEMCOMP compareXsOutputConfiguration,	// compare
	XSEXPCASTRAWCOPY XsArray_rawCopy	// raw copy
};

/*! \copydoc XsArray_construct
	\note Specialization for XsOutputConfigurationArray
*/
void XsOutputConfigurationArray_construct(XsOutputConfigurationArray* thisPtr, XsSize count, XsOutputConfiguration const* src)
{
	XsArray_construct(thisPtr, &g_xsOutputConfigurationArrayDescriptor, count, src);
}
