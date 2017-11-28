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

#include "xsrange.h"

/*! \class XsRange
	\brief A class whose objects can be used to store a range. It provides method to
		   check whether a value is inside the range.
*/

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \relates XsRange \brief Get the number of values in the range.
    \note The range is *inclusive* [first, last] instead of [first, last>. So count [1, 2] = 2
	\returns The number of values in the range (inclusive)
*/
int XsRange_count(const XsRange* thisPtr)
{
	if (thisPtr->m_last < thisPtr->m_first)
		return 0;
	return 1 + thisPtr->m_last - thisPtr->m_first;
}

/*! \relates XsRange \brief Get the number of values in the range.
    \note The range is *exclusive* [first, last> instead of [first, last]. So interval [1, 2] = 1
	\returns The number of values in the range (exclusive)
*/
int XsRange_interval(const XsRange* thisPtr)
{
	if (thisPtr->m_last <= thisPtr->m_first)
		return 0;
	return thisPtr->m_last - thisPtr->m_first;
}

/*! \relates XsRange \brief Test if the range contains the given value \a i. */
int XsRange_contains(const XsRange* thisPtr, int i)
{
	return (i >= thisPtr->m_first && i <= thisPtr->m_last);
}

/*! \relates XsRange \brief Set a new range. */
void XsRange_setRange(XsRange* thisPtr, int f, int l)
{
	thisPtr->m_first = f;
	thisPtr->m_last = l;
}

/*!	\relates XsRange \brief Test if the range is empty.
	\details An empty range has a last element that is lower than its first element.
	\returns true if the range is empty, false otherwise
*/
int XsRange_empty(const XsRange* thisPtr)
{
	return thisPtr->m_last < thisPtr->m_first;
}

/*! @} */
