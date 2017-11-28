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

#include <xsens/xsdataformat.h>
#include "packetfixeddata.h"

#ifdef LOG_PACKET
#	include "xslog.h"
#	define PACKETLOG	XSENSLOG
#else
#	define PACKETLOG(...)
#endif

/*! \brief Default constructor, creates an empty (invalid) object
*/
PacketFixedData::PacketFixedData()
	: m_infoList(NULL)
	, m_formatList(NULL)
	, m_idList(NULL)
	, m_xm(false)
	, m_itemCount(0)
{
	PACKETLOG("%s creating default %p\n", __FUNCTION__, this);
}

/*! \brief Sized constructor, creates an object with room for \a count device's worth of data
	\details The constructor sets the xbus flag to false
	\param count The number of devices whose metadata is stored in the object
*/
PacketFixedData::PacketFixedData(XsSize count)
	: m_infoList(NULL)
	, m_formatList(NULL)
	, m_idList(NULL)
	, m_xm(false)
	, m_itemCount(count)
{
	PACKETLOG("%s creating %p with %d items\n", __FUNCTION__, this, count);
	m_formatList = new XsDataFormat[m_itemCount];
	m_infoList = new PacketInfo[m_itemCount];
	m_idList = new XsDeviceId[m_itemCount];
}

/*! \brief Copy constructor
	\param p The object to copy the contents from
*/
PacketFixedData::PacketFixedData(const PacketFixedData& p)
	: m_infoList(NULL)
	, m_formatList(NULL)
	, m_idList(NULL)
	, m_xm(false)
	, m_itemCount(0)
{
	PACKETLOG("%s creating %p from %p\n", __FUNCTION__, this, &p);
	*this = p;
	PACKETLOG("%s done creating %p\n", __FUNCTION__, this);
}

/*! \brief Destructor
*/
PacketFixedData::~PacketFixedData()
{
	PACKETLOG("%s %p\n", __FUNCTION__, this);
	m_itemCount = 0;
	delete[] m_formatList;
	delete[] m_infoList;
	delete[] m_idList;
	PACKETLOG("%s %p exit\n", __FUNCTION__, this);
}

/*! \brief Assignment operator, copies contents from \a data
	\param data The object to copy from
*/
void PacketFixedData::operator = (const PacketFixedData& data)
{
	if (this == &data)
		return;

	PACKETLOG("%s copy from %p to %p\n", __FUNCTION__, &data, this);

	delete[] m_formatList;
	delete[] m_idList;
	delete[] m_infoList;
	m_formatList = NULL;
	m_idList = NULL;
	m_infoList = NULL;

	m_itemCount = data.m_itemCount;
	m_formatList = new XsDataFormat[data.m_itemCount];
	m_idList = new XsDeviceId[data.m_itemCount];
	m_infoList = new PacketInfo[data.m_itemCount];

	for (uint16_t i = 0; i < data.m_itemCount; ++i)
	{
		m_formatList[i] = data.m_formatList[i];
		m_infoList[i] = data.m_infoList[i];
		m_idList[i] = data.m_idList[i];
	}
	m_xm = data.m_xm;

	PACKETLOG("%s exit\n", __FUNCTION__);
}

