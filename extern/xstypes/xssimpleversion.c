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


#include "xssimpleversion.h"

/*! \class XsSimpleVersion
	\brief A class to store version information
*/

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \relates XsSimpleVersion \brief Test if this is a null-version. */
int XsSimpleVersion_empty(const XsSimpleVersion* thisPtr)
{
#ifdef __cplusplus
	return thisPtr->major() == 0 && thisPtr->minor() == 0 && thisPtr->revision() == 0;
#else
	return thisPtr->m_major == 0 && thisPtr->m_minor == 0 && thisPtr->m_revision == 0;
#endif
}

/*! \brief Swap the contents of \a a with those of \a b
*/
void XsSimpleVersion_swap(struct XsSimpleVersion* a, struct XsSimpleVersion* b)
{
	XsSimpleVersion tmp = *a;
	*a = *b;
	*b = tmp;
}

/*! \brief Compare two XsSimpleVersion objects.
	\param a The left hand side of the comparison
	\param b The right hand side of the comparison
	\return 0 when they're equal
*/
int XsSimpleVersion_compare(XsSimpleVersion const* a, XsSimpleVersion const* b)
{
#ifdef __cplusplus
	return a->major() != b->major() || a->minor() != b->minor() || a->revision() != b->revision();
#else
	return a->m_major != b->m_major || a->m_minor != b->m_minor || a->m_revision != b->m_revision;
#endif
}

/*! @} */
