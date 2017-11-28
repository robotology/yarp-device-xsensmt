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

#include "xstypedefs.h"
#include <string.h>

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \brief Convert the %XsDataFlags to a human readable string
	\param f The flags to translate
	\returns A pointer to a statically allocated memory buffer. Do not free this buffer.
	\note This function is NOT reentrant, multiple simultaneous calls may cause crashes.
	Also, later calls will invalidate the results of earlier calls.
*/
const char *XsDataFlags_toString(XsDataFlags f)
{
	static char rv[4*20];
	if (f == XSDF_None)
		return "XSDF_None";

	rv[0] = 0;
	if (f & XSDF_Managed)
		strcpy(rv, "XSDF_Managed");
	if (f & XSDF_FixedSize)
	{
		if (rv[0])
			strcat(rv, " | ");
		strcat(rv, "XSDF_FixedSize");
	}
	if (f & XSDF_Empty)
	{
		if (rv[0])
			strcat(rv, " | ");
		strcat(rv, "XSDF_Empty");
	}
	return rv;
}

/*! @} */ 
