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

#include "xsdeviceid.h"
#include "xsstring.h"
#include "xsdid.h"
#include <stdio.h>

/*! \class XsDeviceId
	\brief Contains an Xsens device ID and provides operations for determining the type of device
*/

/*! \addtogroup cinterface C Interface
	@{
*/

/*!	\brief Returns the value of a broadcast deviceID.
*/
uint32_t XsDeviceId_broadcast(void)
{
	return XS_DID_BROADCAST;
}

/*!	\brief Test if the device ID is a valid id (not 0).
*/
int XsDeviceId_isValid(const struct XsDeviceId* thisPtr)
{
	return thisPtr->m_deviceId != 0;
}

/*! \brief Test if this device ID represents an Xbus Master. */
int XsDeviceId_isXbusMaster(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_XM);
}

/*! \brief Test if this device ID represents an awinda1 device. */
int XsDeviceId_isAwinda1(const struct XsDeviceId* thisPtr)
{
	return XS_DID_AWINDA1(thisPtr->m_deviceId);
}

/*! \brief Test if this device ID represents an awinda2 device. */
int XsDeviceId_isAwinda2(const struct XsDeviceId* thisPtr)
{
	return XS_DID_AWINDA2(thisPtr->m_deviceId);
}

/*! \brief Test if this device ID represents an awinda device. */
int XsDeviceId_isAwinda(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isAwinda1(thisPtr) || XsDeviceId_isAwinda2(thisPtr);
}

/*! \brief Test if this device ID represents a bodypack device. */
int XsDeviceId_isBodyPack(const struct XsDeviceId* thisPtr)
{
	return XS_DID_BODYPACK(thisPtr->m_deviceId) || (thisPtr->m_deviceId == XS_DID_ABMCLOCKMASTER);
}

/*! \brief Test if this device ID represents a Wireless Master device (Awinda Station, Awinda Dongle, Awinda OEM). */
int XsDeviceId_isWirelessMaster(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_AWINDAMASTER) &&
			!XsDeviceId_isBodyPack(thisPtr) &&
			!XsDeviceId_isSyncStation(thisPtr);
}

/*! \brief Test if this device ID represents an Awinda1 Station.
*/
int XsDeviceId_isAwinda1Station(const struct XsDeviceId* thisPtr)
{
	return XS_DID_AWINDA1_STATION(thisPtr->m_deviceId);
}

/*! \brief Test if this device ID represents an Awinda1 Dongle. */
int XsDeviceId_isAwinda1Dongle(const struct XsDeviceId* thisPtr)
{
	return XS_DID_AWINDA1_DONGLE(thisPtr->m_deviceId);
}

/*! \brief Test if this device ID represents an Awinda1 OEM board. */
int XsDeviceId_isAwinda1Oem(const struct XsDeviceId* thisPtr)
{
	return XS_DID_AWINDA1_OEM(thisPtr->m_deviceId);
}

/*! \brief Test if this device ID represents an Awinda2 Station.
*/
int XsDeviceId_isAwinda2Station(const struct XsDeviceId* thisPtr)
{
	return XS_DID_AWINDA2_STATION(thisPtr->m_deviceId);
}

/*! \brief Test if this device ID represents an Awinda2 Dongle. */
int XsDeviceId_isAwinda2Dongle(const struct XsDeviceId* thisPtr)
{
	return XS_DID_AWINDA2_DONGLE(thisPtr->m_deviceId);
}

/*! \brief Test if this device ID represents an Awinda2 OEM board. */
int XsDeviceId_isAwinda2Oem(const struct XsDeviceId* thisPtr)
{
	return XS_DID_AWINDA2_OEM(thisPtr->m_deviceId);
}

/*! \brief Test if this device ID represents an Awinda Station.
*/
int XsDeviceId_isAwindaStation(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isAwinda1Station(thisPtr) || XsDeviceId_isAwinda2Station(thisPtr);
}

/*! \brief Test if this device ID represents an Awinda Dongle. */
int XsDeviceId_isAwindaDongle(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isAwinda1Dongle(thisPtr) || XsDeviceId_isAwinda2Dongle(thisPtr);
}

/*! \brief Test if this device ID represents an Awinda OEM board. */
int XsDeviceId_isAwindaOem(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isAwinda1Oem(thisPtr) || XsDeviceId_isAwinda2Oem(thisPtr);
}

/*! \brief Test if this device ID represents a SyncStation. */
int XsDeviceId_isSyncStation(const struct XsDeviceId* thisPtr)
{
	return XS_DID_SYNCSTATION(thisPtr->m_deviceId);
}

/*! \brief Test if this device ID represents a SyncStation v1. */
int XsDeviceId_isSyncStation1(const struct XsDeviceId* thisPtr)
{
	return XS_DID_SYNCSTATION1(thisPtr->m_deviceId);
}

/*! \brief Test if this device ID represents a SyncStation v2. */
int XsDeviceId_isSyncStation2(const struct XsDeviceId* thisPtr)
{
	return XS_DID_SYNCSTATION2(thisPtr->m_deviceId);
}

/*! \brief Test if this device ID represents an MTw1. */
int XsDeviceId_isMtw1(const struct XsDeviceId* thisPtr)
{
	return XS_DID_WMMT(thisPtr->m_deviceId);
}

/*! \brief Test if this device ID represents an MTw2. */
int XsDeviceId_isMtw2(const struct XsDeviceId* thisPtr)
{
	return XS_DID_MTW2(thisPtr->m_deviceId);
}

/*! \brief Test if this device ID represents an MTw */
int XsDeviceId_isMtw(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtw1(thisPtr) || XsDeviceId_isMtw2(thisPtr);
}

/*! \brief Test if this device ID represents an Xbus Master Motion Tracker. */
int XsDeviceId_isXbusMasterMotionTracker(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_TYPE_MASK) == XS_DID_TYPE_MTX_XBUS);
}

/*! \brief Test if this device ID represents an MT9c. */
int XsDeviceId_isMt9c(const struct XsDeviceId* thisPtr)
{
	return (((thisPtr->m_deviceId & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_MTI_MTX) && !XsDeviceId_isMtw1(thisPtr));
}

/*! \brief Test if this device ID represents a legacy MTi-G. */
int XsDeviceId_isLegacyMtig(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_MTIG);
}

/*! \brief Test if this device ID represents an MTi-G. */
int XsDeviceId_isMtig(const struct XsDeviceId* thisPtr)
{
	return (((thisPtr->m_deviceId & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_MTIG) || ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_700));
}

/*! \brief Test if this device ID represents an MTMk4. */
int XsDeviceId_isMtMk4(const struct XsDeviceId* thisPtr)
{
	return ( XsDeviceId_isMtMk4_X(thisPtr) ||
			 XsDeviceId_isMtMk4_X0(thisPtr) ||
			 XsDeviceId_isMtMk4_X00 (thisPtr));
}

/*! \brief Test if this device ID represents an MTMk4 1 series. */
int XsDeviceId_isMtMk4_X(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_MT_X_MPU);
}

/*! \brief Test if this device ID represents an MTMk4 1. */
int XsDeviceId_isMtMk4_1(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_1_MPU);
}

/*! \brief Test if this device ID represents an MTMk4 2. */
int XsDeviceId_isMtMk4_2(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_2_MPU);
}

/*! \brief Test if this device ID represents an MTMk4 3. */
int XsDeviceId_isMtMk4_3(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_3_MPU);
}

/*! \brief Test if this device ID represents an MTMk4 10 series. */
int XsDeviceId_isMtMk4_X0(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_MT_X0);
}

/*! \brief Test if this device ID represents an MTMk4 10. */
int XsDeviceId_isMtMk4_10(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_10);
}

/*! \brief Test if this device ID represents an MTMk4 20. */
int XsDeviceId_isMtMk4_20(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_20);
}

/*! \brief Test if this device ID represents an MTMk4 30. */
int XsDeviceId_isMtMk4_30(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_30);
}

/*! \brief Test if this device ID represents an MTMk4 100 series. */
int XsDeviceId_isMtMk4_X00(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_MT_X00);
}

/*! \brief Test if this device ID represents an MTMk4 100. */
int XsDeviceId_isMtMk4_100(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_100);
}

/*! \brief Test if this device ID represents an MTMk4 200. */
int XsDeviceId_isMtMk4_200(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_200);
}

/*! \brief Test if this device ID represents an MTMk4 300. */
int XsDeviceId_isMtMk4_300(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_300);
}

/*! \brief Test if this device ID represents an MTMk4 400. */
int XsDeviceId_isMtMk4_400(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_400);
}

/*! \brief Test if this device ID represents an MTMk4 500. */
int XsDeviceId_isMtMk4_500(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_500);
}

/*! \brief Test if this device ID represents an MTMk4 600. */
int XsDeviceId_isMtMk4_600(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_600);
}

/*! \brief Test if this device ID represents an MTMk4 700.
	\return True if the device ID represents an MTMk4 700.
*/
int XsDeviceId_isMtMk4_700(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_700);
}

/*! \brief Test if this device ID represents an MTMk4 710. */
int XsDeviceId_isMtMk4_710(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_700) && thisPtr->m_deviceId >= XS_DID_MK4TYPE_MT_710_RANGE_START;
}

/*! \brief Test if this device ID represents an MTMk4 800. */
int XsDeviceId_isMtMk4_800(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_800);
}

/*! \brief Test if this device ID represents an MTMk4 900. */
int XsDeviceId_isMtMk4_900(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_900);
}

/*! \brief Test if this device ID represents an MTMk5. */
int XsDeviceId_isMtMk5(const struct XsDeviceId* thisPtr)
{
	return ((XsDeviceId_isMtMk4_X0(thisPtr) || XsDeviceId_isMtMk4_X00(thisPtr))
			&& ((thisPtr->m_deviceId & XS_DID_TYPEL_MK5) != 0));
}

/*! \brief Test if this device ID represents an MTMk5 10 series. */
int XsDeviceId_isMtMk5_X0(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtMk5(thisPtr) && XsDeviceId_isMtMk4_X0(thisPtr);
}

/*! \brief Test if this device ID represents an MTMk5 10. */
int XsDeviceId_isMtMk5_10(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtMk5(thisPtr) && XsDeviceId_isMtMk4_10(thisPtr);
}

/*! \brief Test if this device ID represents an MTMk5 20. */
int XsDeviceId_isMtMk5_20(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtMk5(thisPtr) && XsDeviceId_isMtMk4_20(thisPtr);
}

/*! \brief Test if this device ID represents an MTMk5 30. */
int XsDeviceId_isMtMk5_30(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtMk5(thisPtr) && XsDeviceId_isMtMk4_30(thisPtr);
}

/*! \brief Test if this device ID represents an MTMk5 100 series. */
int XsDeviceId_isMtMk5_X00(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtMk5(thisPtr) && XsDeviceId_isMtMk4_X00(thisPtr);
}

/*! \brief Test if this device ID represents an MTMk5 100. */
int XsDeviceId_isMtMk5_100(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtMk5(thisPtr) && XsDeviceId_isMtMk4_100(thisPtr);
}

/*! \brief Test if this device ID represents an MTMk5 200. */
int XsDeviceId_isMtMk5_200(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtMk5(thisPtr) && XsDeviceId_isMtMk4_200(thisPtr);
}

/*! \brief Test if this device ID represents an MTMk5 300. */
int XsDeviceId_isMtMk5_300(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtMk5(thisPtr) && XsDeviceId_isMtMk4_300(thisPtr);
}

/*! \brief Test if this device ID represents an MTMk5 710. */
int XsDeviceId_isMtMk5_710(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtMk5(thisPtr) && XsDeviceId_isMtMk4_710(thisPtr);
}

/*! \brief Test if this device ID represents an MTx2 */
int XsDeviceId_isMtx2(const struct XsDeviceId* thisPtr)
{
	return XS_DID_MTX2(thisPtr->m_deviceId);
}

/*! \brief Test if this device ID represents an MTx */
int XsDeviceId_isMtx(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtx2(thisPtr);
}

/*! \brief Test if this device ID represents a Fairchild FIS1100 SDK
*/
int XsDeviceId_isFis1100EvalKit(const struct XsDeviceId* thisPtr)
{
	return XS_DID_FIS1100EVK(thisPtr->m_deviceId);
}

/*! \brief Test if this device ID represents a Fairchild FIS2100 SDK
*/
int XsDeviceId_isFis2100EvalKit(const struct XsDeviceId* thisPtr)
{
	return XS_DID_FIS2100EVK(thisPtr->m_deviceId);
}

/*! \brief Test if this device ID represents a Fairchild FIS SDK
*/
int XsDeviceId_isFisEvalKit(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isFis1100EvalKit(thisPtr) || XsDeviceId_isFis2100EvalKit(thisPtr);
}

/*! \brief Test if this device ID represents an Fmt x000 series. */
int XsDeviceId_isFmt_X000(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isFmt1000(thisPtr) ;
}

/*! \brief Test if this device ID represents an FMT1000 series. */
int XsDeviceId_isFmt1000(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_MT_X_FIS);
}

/*! \brief Test if this device ID represents an FMT1010. */
int XsDeviceId_isFmt1010(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_1_FIS);
}

/*! \brief Test if this device ID represents an FMT1020. */
int XsDeviceId_isFmt1020(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_2_FIS);
}

/*! \brief Test if this device ID represents an FMT1030. */
int XsDeviceId_isFmt1030(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_3_FIS);
}

/*! \brief Test if this device ID represents an IMU. */
int XsDeviceId_isImu(const struct XsDeviceId* thisPtr)
{
	return (((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_1_MPU) ||
		((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_10) ||
		((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_100) ||
		((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_1_FIS));
}

/*! \brief Test if this device ID represents a VRU. */
int XsDeviceId_isVru(const struct XsDeviceId* thisPtr)
{
	return (((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_2_MPU) ||
		((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_20) ||
		((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_200) ||
		((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_2_FIS));
}

/*! \brief Test if this device ID represents an AHRS. */
int XsDeviceId_isAhrs(const struct XsDeviceId* thisPtr)
{
	return (((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_3_MPU) ||
		((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_30) ||
		((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_300) ||
		((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_3_FIS));
}

/*! \brief Test if this device ID represents any of the container devices such as Xbus Master and Awinda Station
*/
int XsDeviceId_isContainerDevice(const struct XsDeviceId* thisPtr)
{
	return	XsDeviceId_isBodyPack(thisPtr) ||
			XsDeviceId_isWirelessMaster(thisPtr) ||
			XsDeviceId_isXbusMaster(thisPtr);
}

/*!	\brief Test if the device ID has the broadcast bit set
	\return True if the broadcast bit is set
*/
int XsDeviceId_containsBroadcast(const XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_BROADCAST) != 0);
}

/*! \brief Test if this device ID \e is \e the broadcast id.
	\return TRue if the deviceid is a broadcast
*/
int XsDeviceId_isBroadcast(const XsDeviceId* thisPtr)
{
	return (thisPtr->m_deviceId == XS_DID_BROADCAST);
}

/*! \brief Get a string with a readable representation of this device ID. */
void XsDeviceId_toString(const XsDeviceId* thisPtr, XsString* str)
{
	char device[9];
	sprintf(device, "%08" PRINTF_INT32_MODIFIER "X", thisPtr->m_deviceId);	//lint !e534
	XsString_assign(str, 8, device);
}

/*! \brief Read a device ID from the supplied string. */
void XsDeviceId_fromString(XsDeviceId* thisPtr, const XsString *str)
{
	int tmp = 0;
	int count = 0;
	if (!thisPtr || !str || !str->m_data)
		return;
	if (sscanf(str->m_data, "%x%n", &tmp, &count) == 1 && count >= 6)
		thisPtr->m_deviceId = (uint32_t) tmp;
}

/*! \brief Swap the contents of \a a with those of \a b
*/
void XsDeviceId_swap(XsDeviceId* a, XsDeviceId* b)
{
	uint32_t tmp = a->m_deviceId;
	a->m_deviceId = b->m_deviceId;
	b->m_deviceId = tmp;
}

/*! \brief Returns true if this is equal to \a other or this is a type-specifier that matches \a other
	\param other The deviceid to compare this deviceid to
	\return True if the deviceids are equal or the type-specifier matches
*/
int XsDeviceId_contains(XsDeviceId const* thisPtr, XsDeviceId const* other)
{
	if (thisPtr == other)
		return 1;
	if (thisPtr->m_deviceId == other->m_deviceId)
		return 1;
	if (thisPtr->m_deviceId & XS_DID_ID_MASK) // NOTE: This produces incorrect results for device ids ending with 0000. See MVN-3876
		return 0;
	if ((thisPtr->m_deviceId & XS_DID_FULLTYPE_MASK) == (other->m_deviceId & XS_DID_FULLTYPE_MASK))
		return 1;
	return 0;
}

/*! \brief Returns true if the ID is just a device type, not an actual device ID
*/
int XsDeviceId_isType(XsDeviceId const* thisPtr)
{
	// true when we have a valid type (not a broadcast) and a 0 ID
	return	(thisPtr->m_deviceId & (XS_DID_FULLTYPE_MASK & ~XS_DID_BROADCAST))
			&& !(thisPtr->m_deviceId & XS_DID_ID_MASK);
}

/*! \brief Returns the name of the type of device identified by this id
*/
void XsDeviceId_typeName(XsDeviceId const* thisPtr, XsString* str)
{
	if (!str)
		return;

	if (!thisPtr || thisPtr->m_deviceId == 0)
	{
		XsString_assignCharArray(str, "invalid");
		return;
	}

	if (XsDeviceId_isXbusMaster(thisPtr))
		XsString_assignCharArray(str, "Xbus Master");
	else if(XsDeviceId_isAwinda1Station(thisPtr))
		XsString_assignCharArray(str, "Awinda Station v1");
	else if(XsDeviceId_isAwinda1Dongle(thisPtr))
		XsString_assignCharArray(str, "Awinda Dongle v1");
	else if(XsDeviceId_isAwinda1Oem(thisPtr))
		XsString_assignCharArray(str, "Awinda OEM v1");
	else if (XsDeviceId_isAwinda2Station(thisPtr))
		XsString_assignCharArray(str, "Awinda Station v2");
	else if(XsDeviceId_isAwinda2Dongle(thisPtr))
		XsString_assignCharArray(str, "Awinda Dongle v2");
	else if(XsDeviceId_isAwinda2Oem(thisPtr))
		XsString_assignCharArray(str, "Awinda OEM v2");
	else if (XsDeviceId_isMtw1(thisPtr))
		XsString_assignCharArray(str, "MTw");
	else if (XsDeviceId_isMtw2(thisPtr))
		XsString_assignCharArray(str, "MTw2");
	else if (XsDeviceId_isMtx(thisPtr))
		XsString_assignCharArray(str, "MTx Xbus");
	else if (XsDeviceId_isMtx2(thisPtr))
		XsString_assignCharArray(str, "MTx2");
	else if (XsDeviceId_isBodyPack(thisPtr))
		XsString_assignCharArray(str, "Bodypack");
	else if (XsDeviceId_isFis1100EvalKit(thisPtr))
		XsString_assignCharArray(str, "FIS1100 EVK");
	else if (XsDeviceId_isFis2100EvalKit(thisPtr))
		XsString_assignCharArray(str, "FIS2100 EVK");
	else if (XsDeviceId_isFmt1010(thisPtr))
		XsString_assignCharArray(str, "FMT1010");
	else if (XsDeviceId_isFmt1020(thisPtr))
		XsString_assignCharArray(str, "FMT1020");
	else if (XsDeviceId_isFmt1030(thisPtr))
		XsString_assignCharArray(str, "FMT1030");
	else if (XsDeviceId_isSyncStation1(thisPtr))
		XsString_assignCharArray(str, "Sync Station v1");
	else if (XsDeviceId_isSyncStation2(thisPtr))
		XsString_assignCharArray(str, "Sync Station v2");
	else if (XsDeviceId_isMtMk4_1(thisPtr))
		XsString_assignCharArray(str, "MTi-1");
	else if (XsDeviceId_isMtMk4_2(thisPtr))
		XsString_assignCharArray(str, "MTi-2");
	else if (XsDeviceId_isMtMk4_3(thisPtr))
		XsString_assignCharArray(str, "MTi-3");
	else if (XsDeviceId_isMtMk4_10(thisPtr))
		XsString_assignCharArray(str, "MTi-10");
	else if (XsDeviceId_isMtMk4_20(thisPtr))
		XsString_assignCharArray(str, "MTi-20");
	else if (XsDeviceId_isMtMk4_30(thisPtr))
		XsString_assignCharArray(str, "MTi-30");
	else if (XsDeviceId_isMtMk4_100(thisPtr))
		XsString_assignCharArray(str, "MTi-100");
	else if (XsDeviceId_isMtMk4_200(thisPtr))
		XsString_assignCharArray(str, "MTi-200");
	else if (XsDeviceId_isMtMk4_300(thisPtr))
		XsString_assignCharArray(str, "MTi-300");
	else if ( XsDeviceId_isMtMk4_400(thisPtr))
		XsString_assignCharArray(str, "MTi-400");
	else if (XsDeviceId_isMtMk4_500(thisPtr))
		XsString_assignCharArray(str, "MTi-500");
	else if (XsDeviceId_isMtMk4_600(thisPtr))
		XsString_assignCharArray(str, "MTi-600");
	else if (XsDeviceId_isMtMk4_700(thisPtr))
		XsString_assignCharArray(str, "MTi-G-7XX");
	else if (XsDeviceId_isMtMk4_800(thisPtr))
		XsString_assignCharArray(str, "MTi-G-800");
	else if (XsDeviceId_isMtMk4_900(thisPtr))
		XsString_assignCharArray(str, "MTi-G-900");
	else
		XsString_assignCharArray(str, "Unknown");

	if (thisPtr->m_deviceId & XS_DID_BROADCAST)
	{
		XsString bc;
		XsString_construct(&bc);
		XsString_assignCharArray(&bc, " broadcast");
		XsString_append(str, &bc);
	}
}

/*! \brief Returns the type of device identified by this id
*/
uint32_t XsDeviceId_type(struct XsDeviceId const* thisPtr)
{
	return thisPtr->m_deviceId & XS_DID_FULLTYPE_MASK;
}

/*! \brief Returns the device type identified by this id (eg 10, 300 and Awinda2 Master)
*/
uint32_t XsDeviceId_deviceType(struct XsDeviceId const* thisPtr, int detailed)
{
	uint32_t deviceMask = XsDeviceId_deviceTypeMask(thisPtr, detailed);
	return thisPtr->m_deviceId & deviceMask;
}

/*! \brief Returns the mask which can be used to get the detailed device type (eg 10, 300 and Awinda2 Master)
*/
uint32_t XsDeviceId_deviceTypeMask(struct XsDeviceId const* thisPtr, int detailed)
{
	uint32_t deviceMask;

	if (XsDeviceId_isMtMk4_X(thisPtr) || XsDeviceId_isFmt_X000(thisPtr))
		deviceMask = (XS_DID_TYPEH_MASK | (detailed ? (XS_DID_GPH_MASK | XS_DID_GPL_MASK | XS_DID_TYPEL_MASK) : 0));
	else if (XsDeviceId_isMtMk4(thisPtr))
		deviceMask = (XS_DID_TYPEH_MASK | (detailed ? (XS_DID_GPH_MASK | XS_DID_GPL_MASK | XS_DID_TYPEL_MK5) : 0));
	else if (XsDeviceId_isAwinda(thisPtr))
		deviceMask = (XS_DID_TYPEH_MASK | (detailed ? (XS_DID_GPH_MASK | XS_DID_GPL_MASK) : 0));
	else if (XsDeviceId_isSyncStation(thisPtr))
		deviceMask = (XS_DID_TYPEH_MASK | (detailed ? (XS_DID_GPH_MASK | XS_DID_GPL_MASK) : 0));
	else if (XsDeviceId_isMtw(thisPtr) || XsDeviceId_isMtx(thisPtr))
		deviceMask = (XS_DID_TYPE_MASK | (detailed ? (XS_DID_GPH_MASK | XS_DID_GPL_MASK) : 0));
	else if (thisPtr->m_deviceId == XS_DID_ABMCLOCKMASTER)
		deviceMask = XS_DID_ABMCLOCKMASTER;
	else
		deviceMask = XS_DID_TYPEH_MASK;

	return deviceMask;
}

/*! @} */
