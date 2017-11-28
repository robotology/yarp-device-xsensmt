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

/*! \file
	This file contains platform-specific memory allocation routines
*/

#include "xsmalloc.h"
#if !(defined __ICCARM__) && !(defined _ADI_COMPILER) && !defined(__APPLE__) && !defined(__CRCC__) && !(defined(__arm__) && defined(__ARMCC_VERSION))
#include <malloc.h>
#endif
#include <stdlib.h>

#ifdef XSENS_ASSERT_MALLOC
#include <assert.h>
#undef XSENS_ASSERT_MALLOC
#define XSENS_ASSERT_MALLOC(x) assert(x)
#else
#define XSENS_ASSERT_MALLOC(x)
#endif

#if !(defined __ICCARM__) && !(defined _ADI_COMPILER) && defined(XSENS_DEBUG)
//#define TRACK_ALLOCS	32
#endif

#ifdef TRACK_ALLOCS
int lastAllocIdx = -1;
void* lastAllocs[TRACK_ALLOCS];
int lastFreeIdx = -1;
void* lastFrees[TRACK_ALLOCS];

int lastAlignedAllocIdx = -1;
void* lastAlignedAllocs[TRACK_ALLOCS];
int lastAlignedFreeIdx = -1;
void* lastAlignedFrees[TRACK_ALLOCS];
#endif

#ifndef _MSC_VER
#	ifdef __ANDROID__
#		define _aligned_malloc(size, align) memalign(align, size)
#	elif (defined __ICCARM__) || (defined _ADI_COMPILER) || (defined __CRCC__) || (defined IAR_ARM_CM3) || (defined __ARMEL__) || (defined(__arm__) && defined(__ARMCC_VERSION))
#		define _aligned_malloc(a, b) malloc(a)
#	else

void* __cdecl _aligned_malloc(size_t _Size, size_t _Alignment)
{
	void* rv = 0;
	int err = posix_memalign(&rv, _Alignment, _Size);
	if (err == 0)
		return rv;
	return NULL;
}

#	endif

#define _aligned_realloc(p, n, a)	realloc(p, n)
#define _aligned_free(_Memory)		free(_Memory)

#endif //!_MSC_VER


//! \brief Allocates \a sz bytes of memory, optionally tracking the allocation
void* xsMalloc(size_t sz)
{
#ifdef TRACK_ALLOCS
	void* ptr = malloc(sz);
	XSENS_ASSERT_MALLOC(ptr);
	lastAllocIdx = (lastAllocIdx + 1) & (TRACK_ALLOCS-1);
	lastAllocs[lastAllocIdx] = ptr;
	return ptr;
#else
	void* ptr = malloc(sz);
	XSENS_ASSERT_MALLOC(ptr);
	return ptr;
#endif
}

//! \brief Reallocates \a sz bytes of memory, optionally tracking the allocation
void* xsRealloc(void* ptr, size_t sz)
{
#ifdef TRACK_ALLOCS
	lastFreeIdx = (lastFreeIdx + 1) & (TRACK_ALLOCS-1);
	lastFrees[lastFreeIdx] = ptr;

	ptr = realloc(ptr, sz);
	XSENS_ASSERT_MALLOC(ptr);
	lastAllocIdx = (lastAllocIdx + 1) & (TRACK_ALLOCS-1);
	lastAllocs[lastAllocIdx] = ptr;
	return ptr;
#else
	void* mem = realloc(ptr, sz);
	XSENS_ASSERT_MALLOC(mem);
	return mem;
#endif
}

//! \brief Frees the memory pointed to by \a ptr, optionally tracking the allocation
void  xsFree(void* ptr)
{
#ifdef TRACK_ALLOCS
	lastFreeIdx = (lastFreeIdx + 1) & (TRACK_ALLOCS-1);
	lastFrees[lastFreeIdx] = ptr;
#endif
	free(ptr);
}

//! \brief Allocates \a sz bytes of memory on a 16 byte boundary, optionally tracking the allocation
void* xsAlignedMalloc(size_t sz)
{
#ifdef TRACK_ALLOCS
	void* ptr = _aligned_malloc(sz, 16);
	XSENS_ASSERT_MALLOC(ptr);
	lastAlignedAllocIdx = (lastAlignedAllocIdx + 1) & (TRACK_ALLOCS-1);
	lastAlignedAllocs[lastAlignedAllocIdx] = ptr;
	return ptr;
#else
	void* ptr = _aligned_malloc(sz, 16);
	XSENS_ASSERT_MALLOC(ptr);
	return ptr;
#endif
}

//! \brief Reallocates \a sz bytes of memory on a 16 byte boundary, optionally tracking the allocation
void* xsAlignedRealloc(void* ptr, size_t sz)
{
#ifdef TRACK_ALLOCS
	lastFreeIdx = (lastAlignedFreeIdx + 1) & (TRACK_ALLOCS-1);
	lastAlignedFrees[lastAlignedFreeIdx] = ptr;

	ptr = _aligned_realloc(ptr, sz, 16);
	XSENS_ASSERT_MALLOC(ptr);
	lastAlignedAllocIdx = (lastAlignedAllocIdx + 1) & (TRACK_ALLOCS-1);
	lastAlignedAllocs[lastAlignedAllocIdx] = ptr;
	return ptr;
#else
	void* mem = _aligned_realloc(ptr, sz, 16);
	XSENS_ASSERT_MALLOC(mem);
	return mem;
#endif
}

//! \brief Frees the (aligned) memory pointed to by \a ptr, optionally tracking the allocation
void  xsAlignedFree(void* ptr)
{
#ifdef TRACK_ALLOCS
	lastAlignedFreeIdx = (lastAlignedFreeIdx + 1) & (TRACK_ALLOCS-1);
	lastAlignedFrees[lastAlignedFreeIdx] = ptr;
#endif
	_aligned_free(ptr);
}



