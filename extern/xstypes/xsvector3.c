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

#include "xsvector3.h"
#include <string.h>

//lint -e641 conversion from enum to int should not be a problem

/*! \class XsVector3
	\brief A class that represents a fixed size (3) vector
*/

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \relates XsVector3 \brief Init the %XsVector3 and copy the data from \a src into the vector if \a src is not null */
void XsVector3_construct(XsVector3* thisPtr, const XsReal* src)
{
	XsVector_ref(&thisPtr->m_vector, 3, (XsReal*) thisPtr->m_fixedData, XSDF_FixedSize);
	if (src)
		memcpy((XsReal*) thisPtr->m_fixedData, src, 3*sizeof(XsReal));
}

/*! \relates XsVector3 \brief Init the %XsVector3 and copy the data from \a src into the vector if \a src is not null */
void XsVector3_assign(XsVector3* thisPtr, const XsReal* src)
{
	if (src)
		memcpy((XsReal*) thisPtr->m_fixedData, (XsReal*) src, 3*sizeof(XsReal));
}

/*! \relates XsVector3 \brief Frees the XsVector3 */
void XsVector3_destruct(XsVector3* thisPtr)
{
	// don't do anything, no memory needs to be freed
	assert(thisPtr->m_vector.m_flags & XSDF_FixedSize);
	(void)thisPtr;
}

/*! \relates XsVector3 \brief Copy the contents of the %XsVector3 to \a copy */
void XsVector3_copy(XsVector* copy, XsVector3 const* src)
{
	XsVector_copy(copy, &src->m_vector);
}

/*! @} */
