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

#include "xsportinfo.h"
#include <ctype.h>
#include <string.h>	// strlen
#include <stdlib.h> 	// atoi

/*! \class XsPortInfo
	\brief Contains a descriptor for opening a communication port to an Xsens device.
*/
/*! \addtogroup cinterface C Interface
	@{
*/

/*! \relates XsPortInfo
	\brief Initializes the object to the empty state
*/
void XsPortInfo_clear(XsPortInfo* thisPtr)
{
	thisPtr->m_baudrate = XBR_Invalid;
	thisPtr->m_deviceId.m_deviceId = 0;
	thisPtr->m_portName[0] = '\0';
	thisPtr->m_linesOptions = XPLO_All_Ignore;
}

/*! \relates XsPortInfo
	\brief Returns true if the XsPortInfo object is empty
*/
int XsPortInfo_empty(const struct XsPortInfo* thisPtr)
{
	return (thisPtr->m_portName[0] == '\0');
}

/*! \relates XsPortInfo
	\brief The port number
	\returns Returns the port number
	\note Available on Windows only
*/
int XsPortInfo_portNumber(const struct XsPortInfo* thisPtr)
{
	size_t i;

	if (XsPortInfo_empty(thisPtr))
		return 0;

	for (i = 0; i < strlen(thisPtr->m_portName); i++) {
		if (isdigit(thisPtr->m_portName[i])) {
			return atoi(&thisPtr->m_portName[i]);
		}
	}
	return 0;
}

/*! \relates XsPortInfo
	\brief Returns true if this port info object contains a USB device
 */
int XsPortInfo_isUsb(const struct XsPortInfo* thisPtr)
{
#ifdef XSENS_WINDOWS
	return strncmp("\\\\?\\usb", thisPtr->m_portName, 7) == 0;
#else
	return strncmp("USB", thisPtr->m_portName, 3) == 0; // libusb devices start with USB
#endif
}

/*!
 * \relates XsPortInfo
 * \brief Returns true if this port info object contains a network device
 */
int XsPortInfo_isNetwork(const struct XsPortInfo* thisPtr)
{
	return strncmp("NET:", thisPtr->m_portName, 4) == 0;
}

/*!
 * \relates XsPortInfo
 * \brief Returns the network service name of this port
 */
const char* XsPortInfo_networkServiceName(const struct XsPortInfo* thisPtr)
{
	return &thisPtr->m_portName[4];
}

/*! \relates XsPortInfo
	\brief The usb bus
	\returns Returns the Usb bus number
	\note Available on Linux only
*/
int XsPortInfo_usbBus(const struct XsPortInfo* thisPtr)
{
#ifndef XSENS_WINDOWS
	if (XsPortInfo_isUsb(thisPtr))
		return atoi(&thisPtr->m_portName[3]);
#else
	(void) thisPtr;
#endif
	return 0;
}

/*! \relates XsPortInfo
	\brief The usb address
	\returns Returns the usb address
	\note Available on Linux only
*/
int XsPortInfo_usbAddress(const struct XsPortInfo* thisPtr)
{
#ifndef XSENS_WINDOWS
	if (XsPortInfo_isUsb(thisPtr))
		return atoi(&thisPtr->m_portName[7]);
#else
	(void) thisPtr;
#endif
	return 0;
}

/*! \brief Swap the contents of \a a with those of \a b
*/
void XsPortInfo_swap(struct XsPortInfo* a, struct XsPortInfo* b)
{
	int i;
	char c;
	XsPortLinesOptions pLineOpts;

	XsBaudRate t = a->m_baudrate;
	a->m_baudrate = b->m_baudrate;
	b->m_baudrate = t;

	XsDeviceId_swap(&a->m_deviceId, &b->m_deviceId);

	for (i = 0; i < 256; ++i)
	{
		c = a->m_portName[i];
		a->m_portName[i] = b->m_portName[i];
		b->m_portName[i] = c;
	}

	pLineOpts = a->m_linesOptions;
	a->m_linesOptions = b->m_linesOptions;
	b->m_linesOptions = pLineOpts;
}

/*! @} */
