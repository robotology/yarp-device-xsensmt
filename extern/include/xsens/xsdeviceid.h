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

#ifndef XSDEVICEID_H
#define XSDEVICEID_H

#include "xstypesconfig.h"
#include "pstdint.h"
#include "xsstring.h"
#ifdef __cplusplus
extern "C" {
#else
#define XSDEVICEID_INITIALIZER	{ 0 }
#endif

struct XsDeviceId;

XSTYPES_DLL_API void XsDeviceId_toString(struct XsDeviceId const* thisPtr, XsString* str);
XSTYPES_DLL_API void XsDeviceId_fromString(struct XsDeviceId* thisPtr, XsString const* str);
XSTYPES_DLL_API int XsDeviceId_isValid(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isXbusMaster(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isBodyPack(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isXbusMasterMotionTracker(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isWirelessMaster(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtw1(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtw2(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtw(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMt9c(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtig(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isLegacyMtig(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isAwinda1(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isAwinda2(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isAwinda(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isAwinda1Station(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isAwinda1Dongle(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isAwinda1Oem(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isAwinda2Station(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isAwinda2Dongle(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isAwinda2Oem(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isAwindaStation(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isAwindaDongle(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isAwindaOem(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isSyncStation(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isSyncStation1(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isSyncStation2(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_X(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_1(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_2(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_3(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_X0(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_10(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_20(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_30(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_X00(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_100(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_200(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_300(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_400(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_500(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_600(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_700(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_710(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_800(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_900(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk5(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk5_X0(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk5_10(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk5_20(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk5_30(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk5_X00(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk5_100(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk5_200(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk5_300(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk5_710(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtx(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtx2(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isFis1100EvalKit(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isFis2100EvalKit(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isFisEvalKit(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isFmt_X000(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isFmt1000(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isFmt1010(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isFmt1020(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isFmt1030(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isImu(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isVru(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isAhrs(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isContainerDevice(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_containsBroadcast(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isBroadcast(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API uint32_t XsDeviceId_broadcast(void);
XSTYPES_DLL_API void XsDeviceId_swap(struct XsDeviceId* a, struct XsDeviceId* b);
XSTYPES_DLL_API int XsDeviceId_contains(struct XsDeviceId const* a, struct XsDeviceId const* b);
XSTYPES_DLL_API int XsDeviceId_isType(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API void XsDeviceId_typeName(struct XsDeviceId const* thisPtr, XsString* str);
XSTYPES_DLL_API uint32_t XsDeviceId_type(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API uint32_t XsDeviceId_deviceType(struct XsDeviceId const* thisPtr, int detailed);
XSTYPES_DLL_API uint32_t XsDeviceId_deviceTypeMask(struct XsDeviceId const* thisPtr, int detailed);

#ifdef __cplusplus
} // extern "C"
#endif

struct XsDeviceId {
#ifdef __cplusplus
	/*! \brief Constructor that creates an XsDeviceId from the supplied \a deviceId */
	inline XsDeviceId(uint32_t deviceId = 0)
		: m_deviceId(deviceId)
	{
	}
	/*! \brief Constructor that creates an XsDeviceId from the supplied XsDeviceId \a other */
	inline XsDeviceId(const XsDeviceId& other)
		: m_deviceId(other.m_deviceId)
	{
	}

	/*! \brief Assign the \a other deviceId to this XsDeviceId */
	inline const XsDeviceId& operator=(const XsDeviceId& other)
	{
		//lint --e{1529} trivial assignment
		m_deviceId = other.m_deviceId;
		return *this;
	}
	/*! \brief Assign the \a deviceId to this XsDeviceId */
	inline const XsDeviceId& operator=(uint32_t deviceId)
	{
		m_deviceId = deviceId;
		return *this;
	}

	/*! \brief Returns the deviceId as an unsigned integer */
	inline uint32_t toInt() const
	{
		return m_deviceId;
	}

	/*! \brief Returns the deviceId as an XsString */
	inline XsString toString() const
	{
		XsString tmp;
		XsDeviceId_toString(this, &tmp);
		return tmp;
	}

	/*! \brief Fills the deviceId with the parsed value from the supplied string \a s
		\param s The string containing the device ID
	*/
	inline void fromString(const XsString &s)
	{
		XsDeviceId_fromString(this, &s);
	}

	/*! \brief \copybrief XsDeviceId_isValid(const struct XsDeviceId*) */
	inline bool isValid() const
	{
		return 0 != XsDeviceId_isValid(this);
	}
	/*! \brief \copybrief XsDeviceId_isXbusMaster(const struct XsDeviceId*) */
	inline bool isXbusMaster() const
	{
		return 0 != XsDeviceId_isXbusMaster(this);
	}
	/*! \brief \copybrief XsDeviceId_isAwinda1(const struct XsDeviceId*) */
	inline bool isAwinda1() const
	{
		return 0 != XsDeviceId_isAwinda1(this);
	}
	/*! \brief \copybrief XsDeviceId_isAwinda2(const struct XsDeviceId*) */
	inline bool isAwinda2() const
	{
		return 0 != XsDeviceId_isAwinda2(this);
	}
	/*! \brief \copybrief XsDeviceId_isAwinda(const struct XsDeviceId*) */
	inline bool isAwinda() const
	{
		return 0 != XsDeviceId_isAwinda(this);
	}
	/*! \brief \copybrief XsDeviceId_isXbusMasterMotionTracker(const struct XsDeviceId*) */
	inline bool isXbusMasterMotionTracker() const
	{
		return 0 != XsDeviceId_isXbusMasterMotionTracker(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtw1(const struct XsDeviceId*) */
	inline bool isMtw1() const
	{
		return 0 != XsDeviceId_isMtw1(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtw2(const struct XsDeviceId*) */
	inline bool isMtw2() const
	{
		return 0 != XsDeviceId_isMtw2(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtw(const struct XsDeviceId*) */
	inline bool isMtw() const
	{
		return 0 != XsDeviceId_isMtw(this);
	}
	/*! \brief \copybrief XsDeviceId_isMt9c(const struct XsDeviceId*) */
	inline bool isMt9c() const
	{
		return 0 != XsDeviceId_isMt9c(this);
	}
	/*! \brief \copybrief XsDeviceId_isLegacyMtig(const struct XsDeviceId*) */
	inline bool isLegacyMtig() const
	{
		return 0 != XsDeviceId_isLegacyMtig(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtig(const struct XsDeviceId*) */
	inline bool isMtig() const
	{
		return 0 != XsDeviceId_isMtig(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4(const struct XsDeviceId*) */
	inline bool isMtMk4() const
	{
		return 0 != XsDeviceId_isMtMk4(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_X(const struct XsDeviceId*) */
	inline bool isMtMk4_X() const
	{
		return 0 != XsDeviceId_isMtMk4_X(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_1(const struct XsDeviceId*) */
	inline bool isMtMk4_1() const
	{
		return 0 != XsDeviceId_isMtMk4_1(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_2(const struct XsDeviceId*) */
	inline bool isMtMk4_2() const
	{
		return 0 != XsDeviceId_isMtMk4_2(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_3(const struct XsDeviceId*) */
	inline bool isMtMk4_3() const
	{
		return 0 != XsDeviceId_isMtMk4_3(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_X0(const struct XsDeviceId*) */
	inline bool isMtMk4_X0() const
	{
		return 0 != XsDeviceId_isMtMk4_X0(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_10(const struct XsDeviceId*) */
	inline bool isMtMk4_10() const
	{
		return 0 != XsDeviceId_isMtMk4_10(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_20(const struct XsDeviceId*) */
	inline bool isMtMk4_20() const
	{
		return 0 != XsDeviceId_isMtMk4_20(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_30(const struct XsDeviceId*) */
	inline bool isMtMk4_30() const
	{
		return 0 != XsDeviceId_isMtMk4_30(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_X00(const struct XsDeviceId*) */
	inline bool isMtMk4_X00() const
	{
		return 0 != XsDeviceId_isMtMk4_X00(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_100(const struct XsDeviceId*) */
	inline bool isMtMk4_100() const
	{
		return 0 != XsDeviceId_isMtMk4_100(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_200(const struct XsDeviceId*) */
	inline bool isMtMk4_200() const
	{
		return 0 != XsDeviceId_isMtMk4_200(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_300(const struct XsDeviceId*) */
	inline bool isMtMk4_300() const
	{
		return 0 != XsDeviceId_isMtMk4_300(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_400(const struct XsDeviceId*) */
	inline bool isMtMk4_400() const
	{
		return 0 != XsDeviceId_isMtMk4_400(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_500(const struct XsDeviceId*) */
	inline bool isMtMk4_500() const
	{
		return 0 != XsDeviceId_isMtMk4_500(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_600(const struct XsDeviceId*) */
	inline bool isMtMk4_600() const
	{
		return 0 != XsDeviceId_isMtMk4_600(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_700(const struct XsDeviceId*) */
	inline bool isMtMk4_700() const
	{
		return 0 != XsDeviceId_isMtMk4_700(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_710(const struct XsDeviceId*) */
	inline bool isMtMk4_710() const
	{
		return 0 != XsDeviceId_isMtMk4_710(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_800(const struct XsDeviceId*) */
	inline bool isMtMk4_800() const
	{
		return 0 != XsDeviceId_isMtMk4_800(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_900(const struct XsDeviceId*) */
	inline bool isMtMk4_900() const
	{
		return 0 != XsDeviceId_isMtMk4_900(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk5(const struct XsDeviceId*) */
	inline bool isMtMk5() const
	{
		return 0 != XsDeviceId_isMtMk5(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk5_X0(const struct XsDeviceId*) */
	inline bool isMtMk5_X0() const
	{
		return 0 != XsDeviceId_isMtMk5_X0(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk5_10(const struct XsDeviceId*) */
	inline bool isMtMk5_10() const
	{
		return 0 != XsDeviceId_isMtMk5_10(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk5_20(const struct XsDeviceId*) */
	inline bool isMtMk5_20() const
	{
		return 0 != XsDeviceId_isMtMk5_20(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk5_30(const struct XsDeviceId*) */
	inline bool isMtMk5_30() const
	{
		return 0 != XsDeviceId_isMtMk5_30(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk5_X00(const struct XsDeviceId*) */
	inline bool isMtMk5_X00() const
	{
		return 0 != XsDeviceId_isMtMk5_X00(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk5_100(const struct XsDeviceId*) */
	inline bool isMtMk5_100() const
	{
		return 0 != XsDeviceId_isMtMk5_100(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk5_200(const struct XsDeviceId*) */
	inline bool isMtMk5_200() const
	{
		return 0 != XsDeviceId_isMtMk5_200(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk5_300(const struct XsDeviceId*) */
	inline bool isMtMk5_300() const
	{
		return 0 != XsDeviceId_isMtMk5_300(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk5_710(const struct XsDeviceId*) */
	inline bool isMtMk5_710() const
	{
		return 0 != XsDeviceId_isMtMk5_710(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtx2(const struct XsDeviceId*) */
	inline bool isMtx2() const
	{
		return 0 != XsDeviceId_isMtx2(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtx(const struct XsDeviceId*) */
	inline bool isMtx() const
	{
		return 0 != XsDeviceId_isMtx(this);
	}
	/*! \brief \copybrief XsDeviceId_isWirelessMaster(const struct XsDeviceId*) */
	inline bool isWirelessMaster() const
	{
		return 0 != XsDeviceId_isWirelessMaster(this);
	}
	/*! \brief \copybrief XsDeviceId_isAwinda1Station(const struct XsDeviceId*) */
	inline bool isAwinda1Station() const
	{
		return 0 != XsDeviceId_isAwinda1Station(this);
	}
	/*! \brief \copybrief XsDeviceId_isAwinda1Dongle(const struct XsDeviceId*) */
	inline bool isAwinda1Dongle() const
	{
		return 0 != XsDeviceId_isAwinda1Dongle(this);
	}
	/*! \brief \copybrief XsDeviceId_isAwinda1Oem(const struct XsDeviceId*)*/
	inline bool isAwinda1Oem() const
	{
		return 0 != XsDeviceId_isAwinda1Oem(this);
	}
	/*! \brief \copybrief XsDeviceId_isAwinda2Station(const struct XsDeviceId*) */
	inline bool isAwinda2Station() const
	{
		return 0 != XsDeviceId_isAwinda2Station(this);
	}
	/*! \brief \copybrief XsDeviceId_isAwinda2Dongle(const struct XsDeviceId*) */
	inline bool isAwinda2Dongle() const
	{
		return 0 != XsDeviceId_isAwinda2Dongle(this);
	}
	/*! \brief \copybrief XsDeviceId_isAwinda2Oem(const struct XsDeviceId*) */
	inline bool isAwinda2Oem() const
	{
		return 0 != XsDeviceId_isAwinda2Oem(this);
	}
	/*! \brief \copybrief XsDeviceId_isAwindaStation(const struct XsDeviceId*) */
	inline bool isAwindaStation() const
	{
		return 0 != XsDeviceId_isAwindaStation(this);
	}
	/*! \brief \copybrief XsDeviceId_isAwindaDongle(const struct XsDeviceId*) */
	inline bool isAwindaDongle() const
	{
		return 0 != XsDeviceId_isAwindaDongle(this);
	}
	/*! \brief \copybrief XsDeviceId_isAwindaOem(const struct XsDeviceId*) */
	inline bool isAwindaOem() const
	{
		return 0 != XsDeviceId_isAwindaOem(this);
	}
	/*! \brief \copybrief XsDeviceId_isSyncStation(const struct XsDeviceId*) */
	inline bool isSyncStation() const
	{
		return 0 != XsDeviceId_isSyncStation(this);
	}
	/*! \brief \copybrief XsDeviceId_isBodyPack(const struct XsDeviceId*) */
	inline bool isBodyPack() const
	{
		return 0 != XsDeviceId_isBodyPack(this);
	}
	/*! \brief \copybrief XsDeviceId_isFis1100EvalKit(const struct XsDeviceId*) */
	inline bool isFis1100EvalKit() const
	{
		return 0 != XsDeviceId_isFis1100EvalKit(this);
	}
	/*! \brief \copybrief XsDeviceId_isFis2100EvalKit(const struct XsDeviceId*) */
	inline bool isFis2100EvalKit() const
	{
		return 0 != XsDeviceId_isFis2100EvalKit(this);
	}
	/*! \brief \copybrief XsDeviceId_isFisEvalKit(const struct XsDeviceId*) */
	inline bool isFisEvalKit() const
	{
		return 0 != XsDeviceId_isFisEvalKit(this);
	}
	/*! \brief \copybrief XsDeviceId_isFmt_X000(const struct XsDeviceId*) */
	inline bool isFmt_X000() const
	{
		return 0 != XsDeviceId_isFmt1000(this);
	}
	/*! \brief \copybrief XsDeviceId_isFmt1000(const struct XsDeviceId*) */
	inline bool isFmt1000() const
	{
		return 0 != XsDeviceId_isFmt1000(this);
	}
	/*! \brief \copybrief XsDeviceId_isFmt1010(const struct XsDeviceId*) */
	inline bool isFmt1010() const
	{
		return 0 != XsDeviceId_isFmt1010(this);
	}
	/*! \brief \copybrief XsDeviceId_isFmt1020(const struct XsDeviceId*) */
	inline bool isFmt1020() const
	{
		return 0 != XsDeviceId_isFmt1020(this);
	}
	/*! \brief \copybrief XsDeviceId_isFmt1030(const struct XsDeviceId*) */
	inline bool isFmt1030() const
	{
		return 0 != XsDeviceId_isFmt1030(this);
	}
	/*! \brief \copybrief XsDeviceId_isImu(const struct XsDeviceId*) */
	inline bool isImu() const
	{
		return 0 != XsDeviceId_isImu(this);
	}
	/*! \brief \copybrief XsDeviceId_isVru(const struct XsDeviceId*) */
	inline bool isVru() const
	{
		return 0 != XsDeviceId_isVru(this);
	}
	/*! \brief \copybrief XsDeviceId_isAhrs(const struct XsDeviceId*) */
	inline bool isAhrs() const
	{
		return 0 != XsDeviceId_isAhrs(this);
	}
	/*! \brief \copybrief XsDeviceId_isContainerDevice(const struct XsDeviceId*) */
	inline bool isContainerDevice() const
	{
		return 0 != XsDeviceId_isContainerDevice(this);
	}
	/*! \copydoc XsDeviceId_containsBroadcast(const XsDeviceId*) */
	inline bool containsBroadcast() const
	{
		return 0 != XsDeviceId_containsBroadcast(this);
	}
	/*! \copydoc XsDeviceId_isBroadcast(const XsDeviceId*) */
	inline bool isBroadcast() const
	{
		return 0 != XsDeviceId_isBroadcast(this);
	}

	/*! \brief Returns true if the \a other deviceId matches this deviceId */
	inline bool operator==(const XsDeviceId& other) const { return m_deviceId == other.m_deviceId; }
	/*! \brief Returns true if the \a other deviceId does not match this deviceId */
	inline bool operator!=(const XsDeviceId& other) const { return m_deviceId != other.m_deviceId; }
	/*! \brief Returns true if this deviceId is less than the \a other deviceId */
	inline bool operator<(const XsDeviceId& other) const { return m_deviceId < other.m_deviceId; }
	/*! \brief Returns true if this deviceId is less or equal to the \a other deviceId */
	inline bool operator<=(const XsDeviceId& other) const { return m_deviceId <= other.m_deviceId; }
	/*! \brief Returns true if this deviceId is larger than the \a other deviceId */
	inline bool operator>(const XsDeviceId& other) const { return m_deviceId > other.m_deviceId; }
	/*! \brief Returns true if this deviceId is larger or equal to the \a other deviceId */
	inline bool operator>=(const XsDeviceId& other) const { return m_deviceId >= other.m_deviceId; }

	/*! \brief Creates and returns a XsDeviceId representing the broadcast deviceId */
	inline static XsDeviceId broadcast()
	{
		return XsDeviceId(XsDeviceId_broadcast());
	}

	/*! \copydoc XsDeviceId_contains(XsDeviceId const*, XsDeviceId const*) */
	inline bool contains(const XsDeviceId& other) const
	{
		return 0 != XsDeviceId_contains(this, &other);
	}

	/*! \brief Returns true if the ID is just a device type, not an actual device ID */
	inline bool isType() const
	{
		return 0 != XsDeviceId_isType(this);
	}

	/*! \brief Returns the type of device identified by this id */
	inline uint32_t type() const
	{
		return XsDeviceId_type(this);
	}

	/*! \brief Returns the (detailed) device type of this id
		\param detailed Boolean whether detailed information is returned
		\return The requested device type
	*/
	inline uint32_t deviceType(bool detailed = true) const
	{
		return XsDeviceId_deviceType(this, detailed ? 1 : 0);
	}

	/*! \brief Returns the detailed device type mask of this id
		\param detailed Boolean whether detailed information is returned
		\return The requested device type mask
	*/
	inline uint32_t deviceTypeMask(bool detailed = true) const
	{
		return XsDeviceId_deviceTypeMask(this, detailed ? 1 : 0);
	}

	/*! \brief Returns the name of the type of device identified by this id */
	inline XsString typeName() const
	{
		XsString rv;
		XsDeviceId_typeName(this, &rv);
		return rv;
	}

private:
#endif
	uint32_t m_deviceId;	//!< The actual device id
};

typedef struct XsDeviceId XsDeviceId;

#if defined(__cplusplus) && !defined(XSENS_NO_STL)
namespace std {
template<typename _CharT, typename _Traits>
basic_ostream<_CharT, _Traits>& operator<<(basic_ostream<_CharT, _Traits>& o, XsDeviceId const& xd)
{
	return (o << xd.toString());
}
}

inline XsString& operator<<(XsString& o, XsDeviceId const& xd)
{
	o.append(xd.toString());
	return o;
}

#endif

#endif
