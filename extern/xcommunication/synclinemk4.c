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

#include "synclinemk4.h"

/*! \addtogroup cinterface C Interface
	@{
*/

//! \brief Translate an SyncLineMk4 into a generic XsSyncLine
XsSyncLine xsl4ToXsl(SyncLineMk4 mk4Line)
{
	switch (mk4Line)
	{
	case XSL4_ClockIn:			return XSL_ClockIn;
	case XSL4_GpsClockIn:		return XSL_GpsClockIn;
	case XSL4_ReqData:			return XSL_ReqData;
	case XSL4_In:				return XSL_In1;
	case XSL4_BiIn:				return XSL_Bi1In;
	case XSL4_BiOut:			return XSL_Bi1Out;
	case XSL4_ExtTimepulseIn:	return XSL_ExtTimepulseIn;
	default:					return XSL_Invalid;
	}
}

//! \brief Translate an XsSyncLine into a Mk4-specififc SyncLineMk4
SyncLineMk4 xslToXsl4(XsSyncLine line)
{
	switch (line)
	{
	case XSL_ClockIn:			return XSL4_ClockIn;
	case XSL_GpsClockIn:		return XSL4_GpsClockIn;
	case XSL_In1:				return XSL4_In;
	case XSL_ReqData:			return XSL4_ReqData;
	case XSL_Bi1In:				return XSL4_BiIn;
	case XSL_Bi1Out:			return XSL4_BiOut;
	case XSL_ExtTimepulseIn:	return XSL4_ExtTimepulseIn;
	default:					return XSL4_Invalid;
	}
}

/*! @} */ 
