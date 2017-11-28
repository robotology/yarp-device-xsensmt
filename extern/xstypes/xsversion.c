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

#include "xsversion.h"
#include "xsstring.h"
#include <stdio.h>

/*! \class XsVersion
	\brief A class to store version information
*/

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \relates XsVersion \brief Test if this is a null-version. */
int XsVersion_empty(const XsVersion* thisPtr)
{
	return thisPtr->m_major == 0 && thisPtr->m_minor == 0 && thisPtr->m_revision == 0;
}

/*! \relates XsVersion \brief Get a string with the version expressed in a readable format. */
void XsVersion_toString(const XsVersion* thisPtr, XsString* version)
{
	char buffer[256];
	size_t chars;

	if (thisPtr->m_build != 0 && thisPtr->m_reposVersion != 0)
		chars = sprintf(buffer, "%d.%d.%d build %d rev %d", thisPtr->m_major, thisPtr->m_minor, thisPtr->m_revision, thisPtr->m_build, thisPtr->m_reposVersion);
	else
		chars = sprintf(buffer, "%d.%d.%d", thisPtr->m_major, thisPtr->m_minor, thisPtr->m_revision);

	XsString_assign(version, chars, buffer);
	if (thisPtr->m_extra.m_size != 0)
	{
		const char space = ' ';
		XsArray_insert(version, version->m_size-1, 1, &space);	//lint !e64
		XsString_append(version, &thisPtr->m_extra);
	}
}

/*!
 *	\relates XsVersion
 *	\brief Get a string with the version expressed in a readable format.
 */
void XsVersion_toSimpleString(const XsVersion* thisPtr, XsString* version)
{
	char buffer[256];
	size_t chars;

	chars = sprintf(buffer, "%d.%d.%d", thisPtr->m_major, thisPtr->m_minor, thisPtr->m_revision);
	XsString_assign(version, chars, buffer);
}

/*!
 *	\relates XsVersion
 *	\brief Set the version to the values in the string
 */
void XsVersion_fromString(XsVersion* thisPtr, const XsString* version)
{
	int major = 0;
	int minor = 0;
	int revision = 0;
	int build = 0;
	int reposVersion = 0;
	int result = 0;
	size_t count = 0;

	assert(thisPtr);
	thisPtr->m_major = 0;
	thisPtr->m_minor = 0;
	thisPtr->m_revision = 0;
	thisPtr->m_build = 0;
	thisPtr->m_reposVersion = 0;
	XsString_resize(&thisPtr->m_extra, 0);
	if (!version || XsString_empty(version))
		return;

	result = sscanf(version->m_data, "%d.%d.%d build %d rev %d%zn", &major, &minor, &revision, &build, &reposVersion, &count);

	if (result > 0)
	{
		thisPtr->m_major = (uint8_t)major;
		thisPtr->m_minor = (uint8_t)minor;
		thisPtr->m_revision = (uint8_t)revision;
	}

	if (result > 3)
	{
		thisPtr->m_build = (uint8_t)build;
		thisPtr->m_reposVersion = (uint8_t)reposVersion;
	}

	if ((result == 5) && ((count + 1) < version->m_size))
		XsString_assignCharArray(&thisPtr->m_extra, &version->m_data[count + 1]);
}

/*! \relates XsVersion
 *	\brief Create a XsVersion a XsSimpleVersion, \a simpleVersion.
 */
void XsVersion_fromSimpleVersion(XsVersion* thisPtr, const XsSimpleVersion* simpleVersion)
{
	thisPtr->m_major = simpleVersion->m_major;
	thisPtr->m_minor = simpleVersion->m_minor;
	thisPtr->m_revision = simpleVersion->m_revision;
	thisPtr->m_build = 0;
	thisPtr->m_reposVersion = 0;
	XsString_resize(&thisPtr->m_extra, 0);
}

/*! \relates XsVersion
 *	\brief Create a XsSimpleVersion (\a version) from a XsVersion.
 */
void XsVersion_toSimpleVersion(const XsVersion* thisPtr, XsSimpleVersion* simpleVersion)
{
	simpleVersion->m_major = thisPtr->m_major;
	simpleVersion->m_minor = thisPtr->m_minor;
	simpleVersion->m_revision = thisPtr->m_revision;
}

/*! @} */
