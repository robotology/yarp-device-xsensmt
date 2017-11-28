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

#include <xsens/xsdatapacket.h>
#include "packetstamper.h"

/*! \class PacketStamper
	\brief Supplies functionality for timestamping data packets.
	\details This class can analyze a data packet and create a proper packet id for it.
*/

//! \brief 32 bit MT Sample Counter boundary
const int64_t PacketStamper::AWINDABOUNDARY = 0x100000000LL;

//! \brief 16 bit MT Sample Counter boundary
const int64_t PacketStamper::MTSCBOUNDARY = 0x00010000LL;

//! \brief 8 bit Sample Counter boundary
const int64_t PacketStamper::SC8BOUNDARY = 0x00000100LL;

/*! \brief Calculate the new large packet counter value based on \a frameCounter and the \a lastCounter
	\details Wraparound is at the given \a boundary
	\param[in] frameCounter The frame counter
	\param[in] lastCounter The last counter
	\param[in] boundary the boundary at which to assume a wrap-around
	\returns The computed packet counter value
	\note If lastCounter < 0, returns frameCounter
*/
int64_t PacketStamper::calculateLargePacketCounter(int64_t frameCounter, int64_t lastCounter, int64_t boundary)
{
	if (lastCounter < 0)
		return frameCounter;

	const int64_t lowMask = boundary - 1;
	const int64_t boundaryHalf = boundary / 2;

	int64_t low = lastCounter & lowMask;
	int64_t dt = frameCounter - low;
	if (dt < -boundaryHalf)
		return lastCounter + dt + boundary;	// positive wraparound
	if (dt < boundaryHalf)
		return lastCounter + dt;				// normal increment

	return lastCounter + dt - boundary;		// negative wraparound
}

/*! \brief Create 64 bit counter for a packet.
	\details Wrap when new XsDataPacket is too far away from the previous XsDataPacket in time.
	Use half cache size as reasonable time difference
	When infinite cache, simply wrap when new is lower than old
	\param pack The XsDataPacket that needs its 64-bit sample counter updated
	\param highestPacket The highest packet available for the current device, it will be updated if
		the new counter is higher than the stored value.
	\returns The computed counter for the packet.
*/
int64_t PacketStamper::stampPacket(XsDataPacket& pack, XsDataPacket& highestPacket)
{
	//! \todo This could be a (couple of) milliseconds too late, this should be set as soon as the source message arrives: mantis 7157
	pack.setTimeOfArrival(XsTimeStamp::now());
	int64_t newCounter, lastCounter = -1;

	if (!highestPacket.empty())
		lastCounter = highestPacket.packetId();

	if (pack.packetId() > 0)
		newCounter = pack.packetId();
	else if (pack.containsPacketCounter())
		newCounter = calculateLargePacketCounter(pack.packetCounter(), lastCounter, MTSCBOUNDARY);
	else if (pack.containsSampleTimeFine())
	{
		newCounter = lastCounter + 1;
		//if (pack.containsSampleTimeCoarse())
		//	newCounter = (int64_t) pack.sampleTime64();
		//else
		//	newCounter = calculateLargeSampleTime((int32_t) pack.sampleTimeFine(), lastCounter);
	}
	else if (pack.containsPacketCounter8())
		newCounter = calculateLargePacketCounter(pack.packetCounter8(), lastCounter, SC8BOUNDARY);
	else if (pack.containsAwindaSnapshot())
		newCounter = calculateLargePacketCounter(pack.awindaSnapshot().m_frameNumber, lastCounter, AWINDABOUNDARY);
	else
		newCounter = lastCounter + 1;

//	JLDEBUG(gJournal, "XsensDeviceAPI", "%s [%08x] old = %I64d new = %I64d diff = %I64d\n", __FUNCTION__, did, lastCounter, newCounter, (newCounter-lastCounter));

	pack.setPacketId(newCounter);
	if (newCounter > lastCounter)
		highestPacket = pack;

	return newCounter;
}

/*! \brief Calculate the new large sample time value based on \a frameTime and the \a lastTime
	\details Wraparound is at 864000000 (1 day @ 10kHz)
	\param[in] frameTime The frame time
	\param[in] lastTime The last time
	\returns The computed packet counter value
	\note If lastTime < 0, returns frameTime
*/
int64_t PacketStamper::calculateLargeSampleTime(int64_t frameTime, int64_t lastTime)
{
	if (lastTime < 0)
		return frameTime;

	int64_t low = lastTime % 864000000;
	int64_t dt = frameTime - low;
	if (dt < (-864000000/2))
		return lastTime + dt + 864000000;	// positive wraparound
	if (dt < (864000000/2))
		return lastTime + dt;				// normal increment

	return lastTime + dt - 864000000;		// negative wraparound
}
