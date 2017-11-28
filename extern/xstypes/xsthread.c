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

#include "xsthread.h"

#ifdef _WIN32
#define MS_VC_EXCEPTION 0x406D1388

#pragma pack(push,8)
// Copied from windows API
struct THREADNAME_INFO
{
	DWORD dwType;		// Must be 0x1000.
	LPCSTR szName;		// Pointer to name (in user addr space).
	DWORD dwThreadID;	// XsThread ID (-1=caller thread).
	DWORD dwFlags;		// Reserved for future use, must be zero.
};
#pragma pack(pop)
typedef struct THREADNAME_INFO THREADNAME_INFO;

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \brief Set the name of the current thread to \a threadName */
void XSTYPES_DLL_API xsNameThisThread(const char* threadName)
{
	DWORD dwThreadID = GetCurrentThreadId();

	//Sleep(10);
	THREADNAME_INFO info;
	info.dwType = 0x1000;
	info.szName = threadName;
	info.dwThreadID = dwThreadID;
	info.dwFlags = 0;

	__try
	{
		RaiseException( MS_VC_EXCEPTION, 0, sizeof(info)/sizeof(ULONG_PTR), (ULONG_PTR*)&info );
	}
	__except(EXCEPTION_EXECUTE_HANDLER)
	{
	}
}
#else
/*! \brief Set the name of the current thread to \a threadName
	\note Not implemented in POSIX
*/
void XSTYPES_DLL_API xsNameThisThread(const char* threadName)
{
	// no implementation for this in POSIX -- pthread_key_t should be known.
	// adding this function does remove some
	// checking from xs4 though.
	(void)threadName;
}

pthread_t XSTYPES_DLL_API xsStartThread(void *(func)(void *), void *param, void *pid) {
	(void)pid;
	pthread_t thread;
	if (pthread_create(&thread, NULL, func, param)) {
		return XSENS_INVALID_THREAD;
	}
	return thread;
}
#endif

/*! @} */
