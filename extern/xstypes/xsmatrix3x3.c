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

#include "xsmatrix3x3.h"
#include <string.h>

//lint -e641 conversion from enum to int should not be a problem

/*! \class XsMatrix3x3
	\brief A class that represents a fixed size (3x3) matrix
*/

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \relates XsMatrix3x3 \brief Init the %XsMatrix3x3 */
void XsMatrix3x3_construct(XsMatrix3x3* thisPtr)
{
	XsMatrix_ref(&thisPtr->m_matrix, 3, 3, 3, (XsReal*) thisPtr->m_fixedData, XSDF_FixedSize);
}

/*! \relates XsMatrix3x3 \brief Init the %XsMatrix3x3 and copy the data from \a src into the matrix if \a src is not null */
void XsMatrix3x3_assign(XsMatrix3x3* thisPtr, const XsReal* src, XsSize srcStride)
{
	XsSize r, c;

	if (src)
	{
		if (srcStride == 0 || srcStride == 3)
			memcpy(thisPtr->m_matrix.m_data, src, 3*3*sizeof(XsReal));
		else
		{
			for (r = 0; r < 3; ++r)
				for (c = 0; c < 3; ++c)
					thisPtr->m_matrix.m_data[r*3+c] = src[r*srcStride + c];
		}
	}
}

/*! \relates XsMatrix3x3 \brief Frees the Matrix3x3 */
void XsMatrix3x3_destruct(XsMatrix3x3* thisPtr)
{
	// don't do anything, no memory needs to be freed, which is what  XsMatrix_destruct will figure out
	assert(thisPtr->m_matrix.m_flags & XSDF_FixedSize);
	(void) thisPtr;
	//XsMatrix_destruct(&thisPtr->m_matrix);
}

/*! \relates XsMatrix3x3 \brief Copy the contents of the %XsMatrix3x3 to \a copy */
void XsMatrix3x3_copy(XsMatrix* copy, XsMatrix3x3 const* src)
{
	XsMatrix_copy(copy, &src->m_matrix);
}

/*! @} */
