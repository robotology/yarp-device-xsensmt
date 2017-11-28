/*	Copyright (c) 2003-2017 Xsens Technologies B.V. or subsidiaries worldwide.
	All rights reserved.

	Redistribution and use in source and binary forms, with or without modification,
	are permitted provided that the following conditions are met:

	1.	Redistributions of source code must retain the above copyright notice,
		this list of conditions and the following disclaimer.

	2.	Redistributions in binary form must reproduce the above copyright notice,
		this list of conditions and the following disclaimer in the documentation
		and/or other materials provided with the distribution.

	3.	Neither the names of the copyright holders nor the names of their contributors
		may be used to endorse or promote products derived from this software without
		specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
	EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
	MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
	THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
	SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
	OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
	HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
	TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#define XSENS_NO_AUTOLIB
// include this file in Visual Studio using C/C++->Advanced->Force Includes (the /FI option)
#ifndef XCOMMUNICATION_CONFIG_H
#define XCOMMUNICATION_CONFIG_H

// define to build with Journaller
#ifndef HAVE_JOURNALLER
//#define HAVE_JOURNALLER
#endif

//////////////////////////////////////////////////
// generic preprocessor defines

// make sure both _WIN32 and WIN32 are defined if either of them is.
#if defined(_WIN32) || defined(_M_IX86)
#	ifndef WIN32
#		define WIN32
#	endif
#	define XSENS_WINDOWS
#endif

#ifdef WIN32
#	ifndef _WIN32
#		define _WIN32
#		define XSENS_WINDOWS
#	endif
#endif

#ifdef _WIN32
#	define USE_WINUSB
#else
#	define XSENS_NO_PORT_NUMBERS
#endif

// make things as secure as possible without modifying the code...
#ifndef _CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES
#define _CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES 1
#endif
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif

#ifdef __GNUC__
#include <limits.h>
#if __WORDSIZE == 64
#	define XSENS_64BIT
#else
#	define XSENS_32BIT
#endif
#endif

#if defined(_WIN64) || defined(_M_X64) || defined(_M_IA64)
#	ifndef XSENS_64BIT
#		define XSENS_64BIT
#	endif
#	ifndef XSENS_WINDOWS
#		define XSENS_WINDOWS
#	endif
#	ifndef WIN64
#		define WIN64
#	endif
#else
#	ifndef XSENS_32BIT
#		define XSENS_32BIT
#	endif
#endif

// all xsens libraries should use unicode
#ifndef UNICODE
#define UNICODE
#endif

// use XSENS_32BIT and XSENS_64BIT to check for 32/64 bit builds in your application
// on non-windows systems these should be defined in this file

/*
Configuration | Runtime | DebInfo | Defines
--------------+---------------------------------------
Debug         | MDd     | Yes     | XSENS_DEBUG;_DEBUG
RelWithDeb    | MD      | Yes     | XSENS_DEBUG;XSENS_RELEASE;_DEBUG
Release       | MD      | No      | XSENS_RELEASE;NDEBUG

The common way to setup configuration-dependent defines:
#if defined(XSENS_DEBUG)
	//// Debug or RelWithDeb build
	#if defined(XSENS_RELEASE)
		//// RelWithDeb build
	#else
		//// Debug build
	#endif
#else
	//// Release build
#endif
*/

#if defined(XSENS_DEBUG) && !defined(JLLOGLEVEL)
	//// Debug or RelWithDeb build
	//#define LOG_RX_TX			// Lowest level byte receive and send (binary log)
	//#define LOG_RX_TX_UNIQUE	// Use unique file names for rx/tx logs
	//#define LOG_RX_TX_FLUSH		// Flush after each log operation (can cause hickups in timing, 300ms is not unheard of)
	#define LOG_RX_TX_PER_STATE	// Use new unique log file after switching to a new state (config/measurement/operational/recording) for rx/tx logs, override LOG_RX_TX_UNIQUE, automatically disabled if LOG_RX_TX is disabled

	#if defined(XSENS_RELEASE)
		//// RelWithDeb build
		#if !defined(QT_DEBUG) && !defined(QT_NO_DEBUG)
		#define QT_NO_DEBUG	1
		#endif
		#ifdef HAVE_JOURNALLER
		//#define JLLOGLEVEL	JLL_DEBUG
		#define RUN_LL		JLL_DEBUG	// JLL_ALERT
		#define DEBUG_LL	JLL_ERROR
		#endif
	#else
		//// Debug build
		#if !defined(QT_DEBUG) && !defined(QT_NO_DEBUG)
		#define QT_DEBUG	1
		#endif
		#ifdef HAVE_JOURNALLER
		//#define JLLOGLEVEL	JLL_TRACE
		#define RUN_LL		JLL_DEBUG
		#define DEBUG_LL	JLL_ERROR
		#endif
	#endif
#else
#	if !defined(JLLOGLEVEL)
	//// Release build
	#if !defined(QT_DEBUG) && !defined(QT_NO_DEBUG)
	#define QT_NO_DEBUG	1
	#endif
	#ifdef HAVE_JOURNALLER
		//#define JLLOGLEVEL	JLL_ALERT
		#define RUN_LL		JLL_ERROR
		#define DEBUG_LL	JLL_FATAL
		#define JLNOLINEINFO
	#endif
#	endif
#endif

#if defined(LOG_RX_TX_PER_STATE) && defined(LOG_RX_TX_UNIQUE)
#undef LOG_RX_TX_UNIQUE
#endif

//////////////////////////////////////////////////
// more generic preprocessor defines
//! Add this macro to the start of a class definition to prevent automatic creation of copy functions
#define XSENS_DISABLE_COPY(className) \
private: \
	className(const className &); \
	className &operator = (const className &);

#ifdef __cplusplus
#ifdef HAVE_JOURNALLER
#include <journaller/journaller.h>
#else
class Journaller;
#endif
extern Journaller* gJournal;
#endif

#if defined(HAVE_JOURNALLER) && (defined(XSENS_DEBUG) || defined(XS_LOG_ALWAYS))
#	define XSEXITLOGC(j)	JournalValueJanitor<XsResultValue> _xsExitLogC(j, m_lastResult, std::string(__FUNCTION__) + " exit, lastResult = ")
#	define XSEXITLOGD(j)	XSEXITLOGC(j)	//JournalValueJanitor<int> _xsExitLogD(j, d->m_lastResult, std::string(__FUNCTION__) + " exit, lastResult = ")
#	define XSEXITLOGN(j)	JournalValueJanitor<std::string> _xsExitLogN(j, std::string(), std::string(__FUNCTION__) + " exit")
#	ifndef DIDLOG
#		define DIDLOG(d)		JLHEXLOG_BARE(d)
#	endif
#else
#	define XSEXITLOGC(j)	((void) 0)
#	define XSEXITLOGD(j)	((void) 0)
#	define XSEXITLOGN(j)	((void) 0)
#	ifndef DIDLOG
#		define DIDLOG(d)		""
#	endif

#if !defined(HAVE_JOURNALLER) && !defined(JLDEBUG)
#define JLTRACE(...)		((void)0)
#define JLTRACE_NODEC(...)	((void)0)
#define JLDEBUG(...)		((void)0)
#define JLDEBUG_NODEC(...)	((void)0)
#define JLALERT(...)		((void)0)
#define JLALERT_NODEC(...)	((void)0)
#define JLERROR(...)		((void)0)
#define JLERROR_NODEC(...)	((void)0)
#define JLFATAL(...)		((void)0)
#define JLFATAL_NODEC(...)	((void)0)
#endif

#endif

#endif
