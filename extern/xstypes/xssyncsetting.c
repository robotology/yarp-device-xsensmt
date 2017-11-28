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

#include "xssyncsetting.h"
#include <string.h>

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \brief Returns whether the selected line is configured as an input line
*/
int XsSyncSetting_isInput(const XsSyncSetting* thisPtr)
{
	switch (thisPtr->m_line)
	{
	case XSL_In1:
	case XSL_In2:
	case XSL_Bi1In:
	case XSL_ClockIn:
	case XSL_CtsIn:
		return 1;

	default:
		return 0;
	}
}

/*! \brief Returns whether the selected line is configured as an output line
*/
int XsSyncSetting_isOutput(const XsSyncSetting* thisPtr)
{
	switch (thisPtr->m_line)
	{
	case XSL_Out1:
	case XSL_Out2:
	case XSL_Bi1Out:
	case XSL_RtsOut:
		return 1;

	default:
		return 0;
	}
}

/*! \brief Swap the contents of \a a with \a b
*/
void XsSyncSetting_swap(XsSyncSetting* a, XsSyncSetting* b)
{
	XsSyncSetting tmp;
	memcpy(&tmp, a, sizeof(XsSyncSetting));
	memcpy(a, b, sizeof(XsSyncSetting));
	memcpy(b, &tmp, sizeof(XsSyncSetting));
}

/*! \brief Compares \a a with \a b
	\returns 0 if \a a equals \a b, negative value if \a a < \a b, positive value if \a a > \a b
	\param[in] a Sync setting a
	\param[in] b Sync setting b
*/
int XsSyncSetting_compare(const struct XsSyncSetting* a, const struct XsSyncSetting* b)
{
	assert(a && b);

	if (a->m_line < b->m_line) return -1;
	if (b->m_line < a->m_line) return 1;
	if (a->m_function < b->m_function) return -1;
	if (b->m_function < a->m_function) return 1;
	if (a->m_polarity < b->m_polarity) return -1;
	if (b->m_polarity < a->m_polarity) return 1;
	if (a->m_pulseWidth < b->m_pulseWidth) return -1;
	if (b->m_pulseWidth < a->m_pulseWidth) return 1;
	if (a->m_offset < b->m_offset) return -1;
	if (b->m_offset < a->m_offset) return 1;
	if (a->m_skipFirst < b->m_skipFirst) return -1;
	if (b->m_skipFirst < a->m_skipFirst) return 1;
	if (a->m_skipFactor < b->m_skipFactor) return -1;
	if (b->m_skipFactor < a->m_skipFactor) return 1;
	if (a->m_clockPeriod < b->m_clockPeriod) return -1;
	if (b->m_clockPeriod < a->m_clockPeriod) return 1;
	if (a->m_triggerOnce < b->m_triggerOnce) return -1;
	if (b->m_triggerOnce < a->m_triggerOnce) return 1;

	return 0;
}

/*! @} */
